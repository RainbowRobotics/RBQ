mod wifi;

use std::sync::atomic::{AtomicBool, AtomicU16, AtomicU64, Ordering};
use std::sync::Mutex;
use tauri::{Emitter, Listener, Manager, RunEvent};
use tauri_plugin_shell::process::{CommandChild, CommandEvent};
use tauri_plugin_shell::ShellExt;

pub struct ProxyState {
    pub child: Mutex<Option<CommandChild>>,
    pub generation: AtomicU64,
    pub current_ip: Mutex<(String, String)>,
    pub requested_ip: Mutex<(String, String)>,
    pub requested_rv: Mutex<(String, String, String)>,
    pub current_rv: Mutex<(String, String, String)>,
    pub restart_gate: Mutex<()>,
    pub port: AtomicU16,
    pub token: String,
    pub repicked: AtomicBool,
}

const PORT_MIN: u16 = 8090;
const PORT_MAX: u16 = 8099;

fn is_nightly_tree() -> bool {
    std::env::var("APPIMAGE")
        .map(|p| p.contains("/RBQ-nightly/") || p.contains("/rbq-controller-nightly/"))
        .unwrap_or(false)
}

const LOCK_PORT_RELEASE: u16 = 8088;
const LOCK_PORT_NIGHTLY: u16 = 8089;
const LOCK_MAGIC: &[u8] = b"rbq-single-instance";

const NAV_BOOT_WAIT_MS: u64 = 5_000;
const NAV_MAX_ATTEMPTS: u32 = 3;

enum SingleInstance {
    Acquired(std::net::TcpListener),
    AlreadyRunning,
    Unavailable,
}

fn acquire_single_instance_lock() -> SingleInstance {
    acquire_lock_on(if is_nightly_tree() { LOCK_PORT_NIGHTLY } else { LOCK_PORT_RELEASE })
}

fn acquire_lock_on(port: u16) -> SingleInstance {
    match std::net::TcpListener::bind(("127.0.0.1", port)) {
        Ok(listener) => SingleInstance::Acquired(listener),
        Err(_) if matches!(probe_lock_holder(port), LockHolder::Showed) => SingleInstance::AlreadyRunning,
        Err(_) => SingleInstance::Unavailable,
    }
}

enum LockHolder {
    Showed,
    NoWindow,
    Foreign,
}

fn probe_lock_holder(port: u16) -> LockHolder {
    use std::io::Read;
    let Ok(mut sock) = std::net::TcpStream::connect(("127.0.0.1", port)) else { return LockHolder::Foreign };
    let _ = sock.set_read_timeout(Some(std::time::Duration::from_millis(1500)));
    let want = LOCK_MAGIC.len() + 1;
    let mut buf = Vec::with_capacity(want);
    let mut chunk = [0u8; 32];
    while buf.len() < want {
        match sock.read(&mut chunk) {
            Ok(0) => break,
            Ok(n) => buf.extend_from_slice(&chunk[..n]),
            Err(_) => break,
        }
    }
    if !buf.starts_with(LOCK_MAGIC) { return LockHolder::Foreign; }
    match buf.get(LOCK_MAGIC.len()) {
        Some(b'1') => LockHolder::Showed,
        _ => LockHolder::NoWindow,
    }
}

type AppSlot = std::sync::Arc<Mutex<Option<tauri::AppHandle>>>;

fn serve_single_instance_lock(listener: std::net::TcpListener, app: AppSlot) {
    std::thread::spawn(move || {
        for stream in listener.incoming() {
            let Ok(mut sock) = stream else { continue };
            use std::io::Write;
            let _ = sock.set_write_timeout(Some(std::time::Duration::from_millis(700)));
            let shown = match app.lock().ok().map(|a| a.as_ref().map(|h| h.get_webview_window("main"))) {
                None | Some(None) => true,
                Some(Some(None)) => false,
                Some(Some(Some(win))) => {
                    let _ = win.unminimize();
                    let _ = win.show();
                    win.set_focus().is_ok()
                }
            };
            let mut reply = LOCK_MAGIC.to_vec();
            reply.push(if shown { b'1' } else { b'0' });
            let _ = sock.write_all(&reply);
        }
    });
}

fn pick_port() -> u16 {
    let nightly = is_nightly_tree();
    let base = if nightly { PORT_MIN + 1 } else { PORT_MIN };
    pick_free_port(base).unwrap_or(base)
}

fn pick_free_port(base: u16) -> Option<u16> {
    if base > PORT_MAX || base < PORT_MIN { return None; }
    (base..=PORT_MAX).find(|p| std::net::TcpListener::bind(("127.0.0.1", *p)).is_ok())
}

fn new_token() -> String {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    format!("{:x}-{:x}", std::process::id(), nanos)
}

fn proxy_is_ours(port: u16, token: &str) -> bool {
    use std::io::{Read, Write};
    let Ok(mut sock) = std::net::TcpStream::connect(("127.0.0.1", port)) else { return false };
    let t = std::time::Duration::from_millis(700);
    let _ = sock.set_read_timeout(Some(t));
    let _ = sock.set_write_timeout(Some(t));
    if sock
        .write_all(b"GET /whoami HTTP/1.0\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n")
        .is_err()
    {
        return false;
    }
    let mut buf = Vec::new();
    let _ = sock.read_to_end(&mut buf);
    String::from_utf8_lossy(&buf).contains(token)
}

fn dist_path(app: &tauri::AppHandle) -> String {
    if cfg!(debug_assertions) {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .expect("repo 루트 경로 해석 실패")
            .join("dist")
            .to_str()
            .expect("dist 경로 문자열 변환 실패")
            .to_string()
    } else {
        app.path()
            .resolve("dist", tauri::path::BaseDirectory::Resource)
            .expect("dist 리소스 경로 해석 실패")
            .to_str()
            .expect("dist 경로 문자열 변환 실패")
            .to_string()
    }
}

pub fn spawn_proxy(app: &tauri::AppHandle, ip: &str, vision_ip: &str,
                   rv: &(String, String, String)) -> Result<CommandChild, String> {
    let my_gen = app.state::<ProxyState>().generation.load(Ordering::SeqCst);
    let port = app.state::<ProxyState>().port.load(Ordering::SeqCst);
    let dist = dist_path(app);

    let spawn_result = app
        .shell()
        .sidecar("rbq-proxy")
        .map_err(|e| format!("사이드카 rbq-proxy 없음: {e}"))
        .and_then(|cmd| {
            let mut args: Vec<String> = vec![
                "--robot".into(), ip.into(), "--robot-vision".into(), vision_ip.into(),
                "--port".into(), port.to_string(), "--host".into(), "127.0.0.1".into(),
                "--dist".into(), dist.clone(),
                "--exit-with-parent".into(),
            ];
            let (rv_url, rv_robot, rv_token) = rv.clone();
            let mut envs: Vec<(String, String)> = vec![(
                "RBQ_PROXY_TOKEN".into(),
                app.state::<ProxyState>().token.clone(),
            )];
            if !rv_url.trim().is_empty() && !rv_robot.trim().is_empty() {
                args.push("--rendezvous".into()); args.push(rv_url);
                args.push("--robot-id".into());   args.push(rv_robot);
                if !rv_token.trim().is_empty() { envs.push(("RBQ_WEBRTC_TOKEN".into(), rv_token)); }
            }
            cmd.args(&args)
                .envs(envs)
                .spawn()
                .map_err(|e| format!("프록시 사이드카 spawn 실패: {e}"))
        });

    let (mut rx, child) = match spawn_result {
        Ok(v) => v,
        Err(e) => {
            eprintln!("[proxy] spawn 실패: {e}");
            let _ = app.emit(
                "proxy-status",
                serde_json::json!({ "running": false, "detail": e }),
            );
            return Err(e);
        }
    };

    *app.state::<ProxyState>().current_ip.lock().unwrap() = (ip.to_string(), vision_ip.to_string());
    *app.state::<ProxyState>().current_rv.lock().unwrap() = rv.clone();

    let app_ready = app.clone();
    let token = app.state::<ProxyState>().token.clone();
    std::thread::spawn(move || {
        for _ in 0..600 {
            let st = app_ready.state::<ProxyState>();
            if st.generation.load(Ordering::SeqCst) != my_gen || st.port.load(Ordering::SeqCst) != port {
                return;
            }
            if proxy_is_ours(port, &token) {
                let _ = app_ready.emit("proxy-status", serde_json::json!({ "running": true }));
                return;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        let _ = app_ready.emit(
            "proxy-status",
            serde_json::json!({ "running": false, "detail": format!("프록시가 {port}에 준비되지 않음(30초 초과)") }),
        );
    });

    let app_evt = app.clone();
    tauri::async_runtime::spawn(async move {
        while let Some(event) = rx.recv().await {
            match event {
                CommandEvent::Stdout(b) | CommandEvent::Stderr(b) => {
                    eprintln!("[proxy] {}", String::from_utf8_lossy(&b));
                }
                CommandEvent::Terminated(payload) => {
                    if app_evt.state::<ProxyState>().generation.load(Ordering::SeqCst) == my_gen {
                        let st = app_evt.state::<ProxyState>();
                        let port_taken = std::net::TcpStream::connect(("127.0.0.1", port)).is_ok();
                        if port_taken
                            && !proxy_is_ours(port, &st.token)
                            && !st.repicked.swap(true, Ordering::SeqCst)
                        {
                            let Some(next) = pick_free_port(port + 1) else {
                                let _ = app_evt.emit(
                                    "proxy-status",
                                    serde_json::json!({
                                        "running": false,
                                        "detail": format!("사용 가능한 포트가 없습니다({PORT_MIN}~{PORT_MAX}) — 다른 컨트롤러 인스턴스를 닫고 다시 실행하세요")
                                    }),
                                );
                                continue;
                            };
                            st.port.store(next, Ordering::SeqCst);
                            eprintln!("[proxy] {port} 를 다른 인스턴스가 쓰고 있음 → {next} 로 재기동");
                            let (ip, vip) = st.current_ip.lock().unwrap().clone();
                            let rv = st.current_rv.lock().unwrap().clone();
                            match spawn_proxy(&app_evt, &ip, &vip, &rv) {
                                Ok(child) => *st.child.lock().unwrap() = Some(child),
                                Err(e) => eprintln!("[proxy] 재기동 실패: {e}"),
                            }
                            continue;
                        }
                        let _ = app_evt.emit(
                            "proxy-status",
                            serde_json::json!({
                                "running": false,
                                "detail": format!("proxy exited: {:?}", payload.code)
                            }),
                        );
                    }
                }
                _ => {}
            }
        }
    });
    Ok(child)
}

fn restart_proxy_to(app: &tauri::AppHandle, ip: &str, vision_ip: &str) {
    let app = app.clone();
    *app.state::<ProxyState>().requested_ip.lock().unwrap() = (ip.to_string(), vision_ip.to_string());
    std::thread::spawn(move || {
        let state = app.state::<ProxyState>();
        let _gate = state.restart_gate.lock().unwrap();
        let (ip, vision_ip) = state.requested_ip.lock().unwrap().clone();
        let rv = state.requested_rv.lock().unwrap().clone();
        let same = *state.current_ip.lock().unwrap() == (ip.clone(), vision_ip.clone())
            && *state.current_rv.lock().unwrap() == rv;
        let port = state.port.load(Ordering::SeqCst);
        if same && proxy_is_ours(port, &state.token) {
            let _ = app.emit("proxy-status", serde_json::json!({ "running": true }));
            return;
        }
        state.generation.fetch_add(1, Ordering::SeqCst);
        if let Some(old) = state.child.lock().unwrap().take() {
            let _ = old.kill();
        }
        for _ in 0..40 {
            if std::net::TcpStream::connect(("127.0.0.1", port)).is_err() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        match spawn_proxy(&app, &ip, &vision_ip, &rv) {
            Ok(child) => *state.child.lock().unwrap() = Some(child),
            Err(e) => eprintln!("[restart_proxy_to] 재시작 실패(배지로 통지됨): {e}"),
        }
    });
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let app_slot: AppSlot = Default::default();
    match acquire_single_instance_lock() {
        SingleInstance::AlreadyRunning => {
            eprintln!("[single-instance] 이미 실행 중 — 기존 창을 띄우고 종료합니다");
            return;
        }
        SingleInstance::Acquired(listener) => serve_single_instance_lock(listener, app_slot.clone()),
        SingleInstance::Unavailable => {
            eprintln!("[single-instance] 잠금 포트를 남이 쓰고 있음 — 중복 실행 검사 없이 진행합니다");
        }
    }
    #[cfg(target_os = "linux")]
    {
        for (k, v) in [("WEBKIT_DISABLE_DMABUF_RENDERER", "1"), ("GDK_BACKEND", "x11")] {
            if std::env::var_os(k).is_none() {
                unsafe { std::env::set_var(k, v) };
            }
        }
        let is_steamos = std::fs::read_to_string("/etc/os-release")
            .map(|s| s.contains("ID=steamos"))
            .unwrap_or(false);
        if is_steamos && std::env::var_os("GTK_IM_MODULE").is_none() {
            unsafe { std::env::set_var("GTK_IM_MODULE", "gtk-im-context-simple") };
        }
        for k in ["GST_PLUGIN_SYSTEM_PATH", "GST_PLUGIN_SYSTEM_PATH_1_0"] {
            if let Ok(p) = std::env::var(k) {
                let bundled = std::env::var("APPDIR")
                    .map(|a| format!("{a}/usr/lib/gstreamer-1.0:"))
                    .unwrap_or_default();
                let sys = "/usr/lib/x86_64-linux-gnu/gstreamer-1.0:/usr/lib/gstreamer-1.0";
                unsafe { std::env::set_var(k, format!("{p}:{bundled}{sys}")) };
            }
        }
    }
    tauri::Builder::default()
        .plugin(tauri_plugin_shell::init())
        .manage(ProxyState {
            child: Mutex::new(None),
            generation: AtomicU64::new(0),
            current_ip: Mutex::new((String::new(), String::new())),
            requested_ip: Mutex::new((String::new(), String::new())),
            requested_rv: Mutex::new((String::new(), String::new(), String::new())),
            current_rv: Mutex::new((String::new(), String::new(), String::new())),
            restart_gate: Mutex::new(()),
            port: AtomicU16::new(pick_port()),
            token: new_token(),
            repicked: AtomicBool::new(false),
        })
        .setup(move |app| {
            if let Ok(mut slot) = app_slot.lock() { *slot = Some(app.handle().clone()); }
            let frontend_booted = std::sync::Arc::new(AtomicBool::new(false));
            if cfg!(debug_assertions) {
                app.handle().plugin(
                    tauri_plugin_log::Builder::default()
                        .level(log::LevelFilter::Info)
                        .build(),
                )?;
            }
            let handle = app.handle().clone();
            match spawn_proxy(&handle, "127.0.0.1", "127.0.0.1",
                              &(String::new(), String::new(), String::new())) {
                Ok(child) => *app.state::<ProxyState>().child.lock().unwrap() = Some(child),
                Err(e) => eprintln!("[setup] 초기 프록시 spawn 실패(배지로 통지됨): {e}"),
            }

            let win = app.get_webview_window("main").expect("main window 없음");

            if is_nightly_tree() {
                let _ = win.set_title("RBQ-nightly");
            }

            let deck_zoom = std::fs::read_to_string("/etc/os-release")
                .map(|s| s.contains("ID=steamos"))
                .unwrap_or(false);
            if deck_zoom {
                let _ = win.set_zoom(1.25);
                let _ = win.set_fullscreen(true);
            }
            let fs_win = win.clone();
            app.listen("toggle-fullscreen", move |_event| {
                let on = fs_win.is_fullscreen().unwrap_or(false);
                let _ = fs_win.set_fullscreen(!on);
            });
            let quit_handle = app.handle().clone();
            app.listen("app-quit", move |_event| {
                quit_handle.exit(0);
            });

            #[cfg(unix)]
            if deck_zoom {
                let watch_handle = app.handle().clone();
                let parent0 = std::os::unix::process::parent_id();
                std::thread::spawn(move || loop {
                    std::thread::sleep(std::time::Duration::from_secs(2));
                    let now = std::os::unix::process::parent_id();
                    if now != parent0 || now <= 1 {
                        eprintln!("[exit] 부모 종료 감지({parent0} -> {now}) — 앱을 끝낸다");
                        watch_handle.exit(0);
                        break;
                    }
                });
            }
            let zoom_win = win.clone();
            app.listen("ui-zoom", move |event| {
                if let Ok(v) = serde_json::from_str::<serde_json::Value>(event.payload()) {
                    let scale = v
                        .get("scale")
                        .and_then(|x| x.as_f64())
                        .map(|s| s.clamp(1.0, 2.0))
                        .unwrap_or(if deck_zoom { 1.25 } else { 1.0 });
                    let _ = zoom_win.set_zoom(scale);
                }
            });

            #[cfg(target_os = "linux")]
            {
                use webkit2gtk::{PermissionRequestExt, SettingsExt, UserMediaPermissionRequest, WebViewExt};
                let wv_res = win.with_webview(|webview| {
                    let wv = webview.inner();
                    if let Some(settings) = wv.settings() {
                        settings.set_enable_media_stream(true);
                        settings.set_enable_mediasource(false);
                        settings.set_default_font_family("sans-serif");
                    }
                    wv.connect_permission_request(|_, req| {
                        use webkit2gtk::glib::object::Cast;
                        if req.clone().dynamic_cast::<UserMediaPermissionRequest>().is_ok() {
                            eprintln!("[mic] UserMediaPermissionRequest 허용");
                            req.allow();
                            true
                        } else {
                            false
                        }
                    });
                });
                if let Err(e) = wv_res {
                    eprintln!("[mic] with_webview 실패: {e}");
                }
            }
            if let Ok(blank) = "about:blank".parse() {
                let _ = win.navigate(blank);
            }

            let nav_app = app.handle().clone();
            let nav_booted = frontend_booted.clone();
            std::thread::spawn(move || {
                let token = nav_app.state::<ProxyState>().token.clone();
                let mut p = 0u16;
                let mut ready = false;
                for _ in 0..200 {
                    p = nav_app.state::<ProxyState>().port.load(Ordering::SeqCst);
                    if proxy_is_ours(p, &token) { ready = true; break; }
                    std::thread::sleep(std::time::Duration::from_millis(150));
                }
                if !ready {
                    eprintln!("[proxy] 30초 안에 우리 프록시가 뜨지 않음 — 빈 화면 유지(남의 origin 을 띄우지 않는다)");
                    return;
                }
                for attempt in 1..=NAV_MAX_ATTEMPTS {
                    p = nav_app.state::<ProxyState>().port.load(Ordering::SeqCst);
                    if !proxy_is_ours(p, &token) {
                        eprintln!("[proxy] :{p} 가 우리 것이 아니다 — 이번 회차는 띄우지 않는다");
                    } else if let Ok(url) = format!("http://localhost:{p}").parse() {
                        if let Err(e) = win.navigate(url) {
                            eprintln!("[proxy] navigate 실패(:{p}): {e}");
                        }
                    }
                    for _ in 0..(NAV_BOOT_WAIT_MS / 200) {
                        if nav_booted.load(Ordering::SeqCst) { return; }
                        std::thread::sleep(std::time::Duration::from_millis(200));
                    }
                    eprintln!("[proxy] 프론트가 뜨지 않음 — 다시 띄운다 ({attempt}/{NAV_MAX_ATTEMPTS})");
                }
                eprintln!("[proxy] 재시도했지만 프론트가 뜨지 않았다 — 앱을 다시 실행해 주세요");
            });

            let listen_handle = app.handle().clone();
            let listen_booted = frontend_booted.clone();
            app.listen("restart-proxy", move |event| {
                listen_booted.store(true, Ordering::SeqCst);
                if let Ok(v) = serde_json::from_str::<serde_json::Value>(event.payload()) {
                    if let Some(ip) = v.get("ip").and_then(|x| x.as_str()) {
                        let vision = v
                            .get("visionIp")
                            .and_then(|x| x.as_str())
                            .filter(|s| !s.trim().is_empty())
                            .unwrap_or(ip);
                        let rv_url = v.get("rendezvousUrl").and_then(|x| x.as_str()).unwrap_or("");
                        let rv_robot = v.get("robotId").and_then(|x| x.as_str()).unwrap_or("");
                        let rv_token = v.get("webrtcToken").and_then(|x| x.as_str()).unwrap_or("");
                        *listen_handle.state::<ProxyState>().requested_rv.lock().unwrap() =
                            (rv_url.to_string(), rv_robot.to_string(), rv_token.to_string());
                        eprintln!("[restart-proxy event] ip={ip} vision={vision} rv={rv_url}");
                        restart_proxy_to(&listen_handle, ip, vision);
                    }
                }
            });

            let status_handle = app.handle().clone();
            app.listen("request-proxy-status", move |_event| {
                let h = status_handle.clone();
                std::thread::spawn(move || {
                    let st = h.state::<ProxyState>();
                    let running = proxy_is_ours(st.port.load(Ordering::SeqCst), &st.token);
                    let _ = h.emit(
                        "proxy-status",
                        serde_json::json!({
                            "running": running,
                            "detail": if running { "" } else { "프록시 미응답(상태 요청)" }
                        }),
                    );
                });
            });

            let scan_h = app.handle().clone();
            app.listen("wifi-scan", move |event| {
                let h = scan_h.clone();
                let req_id = wifi::req_id_of(event.payload());
                std::thread::spawn(move || {
                    let payload = match wifi::backend().scan() {
                        Ok(nets) => wifi::result_ok(
                            &req_id,
                            serde_json::to_value(nets).unwrap_or_else(|_| serde_json::json!([])),
                        ),
                        Err(e) => wifi::result_err(&req_id, e.message()),
                    };
                    let _ = h.emit("wifi-scan-result", payload);
                });
            });

            let conn_h = app.handle().clone();
            app.listen("wifi-connect", move |event| {
                let h = conn_h.clone();
                let raw = event.payload().to_string();
                std::thread::spawn(move || {
                    let v: serde_json::Value = serde_json::from_str(&raw).unwrap_or_default();
                    let req_id = v.get("reqId").and_then(|x| x.as_str()).unwrap_or("").to_string();
                    let ssid = v.get("ssid").and_then(|x| x.as_str()).unwrap_or("").to_string();
                    let password = v.get("password").and_then(|x| x.as_str());
                    let payload = if ssid.is_empty() {
                        wifi::result_err(&req_id, "SSID 없음".into())
                    } else {
                        match wifi::backend().connect(&ssid, password) {
                            Ok(()) => wifi::result_ok(&req_id, serde_json::Value::Null),
                            Err(e) => wifi::result_err(&req_id, e.message()),
                        }
                    };
                    let _ = h.emit("wifi-connect-result", payload);
                });
            });

            let cur_h = app.handle().clone();
            app.listen("wifi-current", move |event| {
                let h = cur_h.clone();
                let req_id = wifi::req_id_of(event.payload());
                std::thread::spawn(move || {
                    let payload = match wifi::backend().current() {
                        Ok(cur) => wifi::result_ok(&req_id, serde_json::to_value(cur).unwrap_or_default()),
                        Err(e) => wifi::result_err(&req_id, e.message()),
                    };
                    let _ = h.emit("wifi-current-result", payload);
                });
            });
            Ok(())
        })
        .build(tauri::generate_context!())
        .expect("error while building tauri application")
        .run(|app_handle, event| {
            if let RunEvent::Exit = event {
                if let Some(child) = app_handle.state::<ProxyState>().child.lock().unwrap().take() {
                    let _ = child.kill();
                }
            }
        });
}

#[cfg(test)]
mod single_instance_tests {
    use super::*;
    use std::io::Write;
    use std::net::TcpListener;

    fn holder(reply: &'static [u8]) -> u16 {
        let l = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let port = l.local_addr().unwrap().port();
        std::thread::spawn(move || {
            for s in l.incoming() {
                if let Ok(mut s) = s {
                    let _ = s.write_all(reply);
                }
            }
        });
        port
    }

    fn free_port() -> u16 {
        let l = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        l.local_addr().unwrap().port()
    }

    #[test]
    fn 빈_포트면_잠금을_잡는다() {
        assert!(matches!(acquire_lock_on(free_port()), SingleInstance::Acquired(_)));
    }

    #[test]
    fn 창을_올린_인스턴스가_있으면_중복_실행이다() {
        assert!(matches!(acquire_lock_on(holder(b"rbq-single-instance1")), SingleInstance::AlreadyRunning));
    }

    #[test]
    fn 창_없는_인스턴스가_쥐고_있으면_새로_뜬다() {
        assert!(matches!(acquire_lock_on(holder(b"rbq-single-instance0")), SingleInstance::Unavailable));
    }

    #[test]
    fn 표식만_오고_상태가_없으면_새로_뜬다() {
        assert!(matches!(acquire_lock_on(holder(LOCK_MAGIC)), SingleInstance::Unavailable));
    }

    #[test]
    fn 남의_서비스가_쥐고_있으면_통과시킨다() {
        assert!(matches!(acquire_lock_on(holder(b"HTTP/1.1 200 OK")), SingleInstance::Unavailable));
    }

    #[test]
    fn 응답이_없으면_통과시킨다() {
        let l = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let port = l.local_addr().unwrap().port();
        std::thread::spawn(move || { for s in l.incoming() { std::mem::forget(s); } });
        assert!(matches!(acquire_lock_on(port), SingleInstance::Unavailable));
    }

    #[test]
    fn 채널마다_잠금이_다르다() {
        assert_ne!(LOCK_PORT_RELEASE, LOCK_PORT_NIGHTLY);
        assert!(LOCK_PORT_RELEASE < PORT_MIN && LOCK_PORT_NIGHTLY < PORT_MIN);
    }
}
