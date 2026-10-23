# electron/ — 데스크탑 셸(Steam 빌드·윈도우 설치본·리눅스 AppImage)

Steam(리눅스·스팀덱)에 올리는 빌드, 윈도우 설치본(.exe), 리눅스 AppImage(x86_64·aarch64)의 셸 — **유일한 데스크탑 셸**이다.
종전 Tauri(WebKitGTK) 셸은 10-12 에 걷어냈다(사용자 결정). 웹 번들(`dist/`)과 프록시(`tools/rbq-web-proxy.js`, Electron 의 Node 로 돈다)를
싣고, 프론트와는 `src/lib/desktopBridge.ts` 의 이벤트로 주고받는다. WebRTC·카메라·마이크·터치는 브라우저 웹 빌드와 같은 경로다.

```bash
bash scripts/steam-package.bash            # → build-out/steam/ (SteamPipe depot 으로 올릴 폴더, 실행 파일 rbq.sh)
RBQ_CHANNEL=nightly bash scripts/win-package.bash   # 윈도우(Git-Bash) → build-out/rbq-controller-x64-setup.exe
RBQ_CHANNEL=nightly bash scripts/linux-package.bash # 리눅스(호스트 arch) → build-out/rbq-controller-<x86_64|aarch64>.AppImage
bash dev.bash                            # 개발 실행 — 웹 번들 갱신 후 electron/ 에서 npx electron . (gui/dist·gui/tools 를 그대로 쓴다)
RBQ_URL_QUERY=demo=1 ...                 # 로봇 없이 데모 모드로 띄운다(스크린샷·심사 확인)
```

## 왜 Tauri 가 아니라 Electron 인가 (09-17 측정 — 걷어낸 이유)

Steam 의 리눅스 게임은 **전부** Steam Linux Runtime 컨테이너(pressure-vessel)에서 돈다 — 호스트 직실행 경로는 없다.
그 컨테이너에서 Tauri AppImage(펼친 폴더)는 화면·3D 는 뜨지만 **HTTPS 가 전부 실패**하고(동봉 GIO TLS 모듈이 런타임
gnutls 와 버전 불일치) **마이크 장치가 0 개**다(GStreamer 플러그인 스캐너 없음). Electron(Chromium)은 같은 컨테이너에서
HTTPS·마이크·WebRTC·WebGL·Gamepad API 가 전부 된다.

## 윈도우 설치본 (10-12)

옛 Tauri 윈도우판은 로봇에 못 붙었다 — 프록시가 wrtc 로 WebRTC 를 대행해야 했는데, NSIS 번들에 wrtc 가 없고 넣으면
pkg 프록시가 0xC0000005 로 죽었다. Electron 은 브라우저가 WebRTC 를 직접 해서 그 문제가 없다.

- **전체화면으로 시작**(`fullscreen`, 10-13 사용자 요청) — 덱과 같다. 창 모드는 F11(`installFullscreenHotkey` → `toggle-fullscreen`).
- **설치기**: electron-builder NSIS(`package.json` 의 `build`), 사용자 단위 설치(관리자 권한 없음). 나이틀리는 appId·이름·설치 폴더가
  달라(`rbq-nightly`) 정식과 같이 깔린다. 채널은 `RBQ_CHANNEL`(또는 `RBQ_NIGHTLY=1`)로 준다 — 없으면 업데이트 UI 가 숨는다.
- **자가 업데이트**: 셸이 실행 파일 옆 `Uninstall *.exe`(설치기가 둔다)를 보고 `RBQ_WIN_INSTALLER` 를 프록시에 넘긴다. 프록시는
  `/app-update?check` 에 `X-App-Mode: exe` 로 답하고, 업데이트는 S3 의 `<태그>/`(나이틀리 `nightly/`) `rbq-controller-x64-setup.exe` 를
  받아 `/S --updated --force-run` 으로 돌린다 — 설치기가 앱을 닫고 덮어쓴 뒤 다시 띄운다. **자산 파일명은 채널과 무관하게 하나**다.
  개발 실행(`electron .`)엔 제거 프로그램이 없어 501 이다.
- **WiFi**: `wifi-netsh.js`(옛 Tauri `netsh.rs` 이식). 시스템 WiFi 창은 `ms-settings:network-wifi`, 바깥 링크는 `explorer.exe <url>`
  (`cmd /c start` 는 URL 의 `&` 를 명령 구분자로 먹는다).
- 실측(10-12, 빌드PC .174, SSH 세션): 빌드·조용한 설치·설치 폴더·`X-App-Mode: exe`·실행 중 덮어쓰기(앱 닫힘) ✅.
  SSH 세션엔 데스크톱이 없어 **재실행(`--force-run`)·바깥 링크·실제 WiFi(그 PC 는 무선 없음)는 미확인**.

## 리눅스 AppImage (10-12)

- **패키징**: electron-builder AppImage, 호스트 아키텍처로만 만든다(arm64 는 arm64 머신에서). `build.bash --linux` 도 이걸 부른다. 파일명은 옛 Tauri 판과 같은
  `rbq-controller-<uname -m>.AppImage` — 깔린 앱의 자가 업데이트·S3 키·Add to Steam 바로가기가 그 이름을 쥐고 있다.
- **실행 파일은 셸 래퍼**(`appimage-wrap.js`, afterPack) — AppRun 이 부르는 `rbq` 가 `LD_PRELOAD`·`GTK_IM_MODULE`·`QT_IM_MODULE` 을 지우고 `rbq.bin` 을 exec 한다.
  Steam 은 비Steam 바로가기에 오버레이(`LD_PRELOAD=gameoverlayrenderer.so`)를 넣는다. Electron 에서는 브라우저 프로세스가 GPU 프로세스를 못 띄우고
  멈추거나(창·프록시 없음), zygote·GPU 프로세스가 죽어 `GPU process isn't usable. Goodbye.` 로 끝난다 — 덱 게이밍 모드에서 옛 Tauri 나이틀리가
  Electron 으로 자가 업데이트된 뒤 실행 불가(10-13 실측: 정식 나이틀리에 LD_PRELOAD 만 지워 Steam 실행 2/2 정상).
  ⚠`--no-sandbox` 만으로는 안 된다 — ssh 에서 LD_PRELOAD 만 흉내 내면 뜨지만, Steam 이 실제로 띄우면(Steam IPC 가 붙어 오버레이가 켜짐) 멈춘다.
  ssh 재현은 Steam 환경 변수를 통째로 넣어도 뜨므로, 확인은 반드시 `steam steam://rungameid/<id>` 로 한다.
  ⚠단일 인스턴스 잠금 때문에, 이렇게 멈춘 인스턴스가 하나 남으면 그 뒤 실행은 조용히 끝난다("실행 자체가 안 된다").
- **런타임은 정적 런타임**(`build.toolsets.appimage`) — 옛 FUSE2 런타임은 `libz.so`(zlib 개발 패키지)를 요구해 보통 데스크톱에서
  바로 죽었다(10-12 ci/cd, bookworm arm64). 정적 런타임도 FUSE(fusermount3)는 쓴다 — 없으면 `--appimage-extract-and-run`.
- **앱 이름**: `rbq-desktop` / `rbq-desktop-nightly` → 설정 폴더 `~/.config/<이름>`·한 번만 뜨기 잠금이 채널마다 갈린다(같이 뜬다).
  `rbq` 는 쓰지 않는다 — `~/.config/rbq` 는 프록시 로봇 목록(/kv) 폴더다.
- **자가 업데이트**: 옛 Tauri 판과 같은 경로(`$APPIMAGE` → 프록시 `/app-update`, 설치 트리 `~/Documents/RBQ[-nightly]/latest/`). AppImage 런타임이
  넣은 `$APPIMAGE` 를 셸이 프록시 env 로 그대로 넘긴다. 받을 파일명은 **프록시가 자기 arch 로 정한다**(`CTRL_ASSET`) — 앱은 이름을 안 보낸다.
- 실측(10-12, 이 PC Ubuntu 22.04 x86_64): 빌드(정적 런타임)·실행·데모 제어 화면·프록시 env `APPIMAGE`·`/app-update?check` 204·채널별 설정 폴더 ✅.
  aarch64(ci/cd 맥 colima VM, bookworm): 빌드·libz.so 없는 환경에서 실행·/pin·/hub ✅(로봇 연결은 VM 이 사내망에 못 나가 미확인).
  **미확인**: Ubuntu 24.04(AppArmor 가 비특권 userns 를 막으면 Chromium 샌드박스가 안 뜰 수 있다 — 그때 `--no-sandbox`),
  실제 설치 트리로의 업데이트·전환.

## 런타임은 3.0(sniper)

Steamworks 런치 옵션에서 **Steam Linux Runtime 3.0 (sniper)** 을 고른다. sniper 에는 GTK3·atk-bridge·atspi 가 들어 있어
Electron 이 라이브러리 동봉 없이 뜬다. 4.0 은 이 셋이 없어 `libatk-bridge-2.0.so.0` 로 즉사한다(동봉하면 뜬다).

## Steam 컨테이너에서 셸이 따로 하는 것

| 항목 | Steam 빌드 | 이유 |
|---|---|---|
| WiFi | NetworkManager **D-Bus**(`wifi.js`) — 일반 리눅스 AppImage 도 같다 | 컨테이너에 nmcli 가 없다. 시스템 버스는 열린다 |
| SteamOS 판별 | `/run/host/os-release` 먼저 | 컨테이너의 `/etc/os-release` 는 런타임(Debian) 것이다 — 프록시도 같은 규칙 |
| 바깥 링크·Steam 설정·키보드 | `steam-runtime-urlopen`(컨테이너 밖이면 `xdg-open`·`steam`) | 컨테이너에 호스트 `steam`·`xdg-open` 이 없다. 런타임 도구가 steam:// 은 Steam 파이프로, https 는 포털→Steam 으로 연다 |
| gamescope 터치 모드·KDE WiFi 창 | `steam-runtime-launch-client --alongside-steam --host` | 호스트 명령. Valve 는 일상 기능엔 비권장 — URL 로 못 하는 두 곳에만 |
| 앱 자가 업데이트 | 없음(501 → UI 숨김) | Steam 이 갱신한다. 앱 밖 자가 업데이트는 Steam 정책 위반 |
| 로봇 패키지 받기 | `RBQ_DESKTOP_SHELL` 게이트로 허용 | 로봇 펌웨어는 Steam 이 대신 못 받는다 |
| 중복 실행 | `requestSingleInstanceLock` | Electron 내장 |

⚠프록시에 @roamhq/wrtc 를 다시 싣지 말 것 — Electron 의 Node 에선 `External buffers are not allowed` 로 프로세스가 죽는다(V8 메모리 케이지, 09-17 실측).

## 스팀덱 게이밍 모드 실측 (2026-09-17, SLR 3.0 sniper 공식 설치본 + 비Steam 바로가기 호환성 도구 강제)

| 항목 | 결과 |
|---|---|
| 컨테이너 안 실행 | ✅ `/proc/<pid>/root/etc/os-release` = steamrt sniper |
| SteamOS·게이밍 판별(`/platform`) · 줌 1.25(1024×640) · 덱 배터리 | ✅ |
| Gamepad API | ✅ `Microsoft X-Box 360 pad 0 (STANDARD GAMEPAD Vendor: 28de Product: 11ff)` — 버튼 한 번 누른 뒤 |
| 마이크 getUserMedia · HTTPS | ✅ 내장 마이크 live · 200 |
| WiFi 스캔·현재망(D-Bus) | ✅ |
| 화면 키보드 `steam://open/keyboard`(urlopen) · 바깥 링크(`/open-url`) | ✅ 둘 다 뜸 |
| 실로봇 연결(브라우저 WebRTC 직결) | ✅ connected, 데이터채널 log·motion-state·command open(motion-state 수신 4.5k), 카메라 `<video>` 1280×720 재생, 사용자 확인 "잘되는듯"(워키토키 포함) |
| 핀치 줌 | ⚠ 기동 시 `/touch-native` 한 번으론 안 먹었다 — Steam 이 게임 창을 잡은 뒤 앱별 터치 설정으로 되돌린다. `deckTouch.ts` 가 **마우스로 들어온 pointerdown 을 신호로 다시 넣게** 고친 뒤 ✅ (첫 탭 한 번은 마우스일 수 있다) |

## 셸이 지키는 것 (10-07)

- **화면 키보드는 앱 자판(`src/lib/osk`) 하나만** — Steam 은 게임에 `GTK_IM_MODULE=Steam` 을 붙여 Chromium 입력 칸 포커스에 Steam 키보드가 저절로 뜬다. 그러면 자판이 두 개 겹치고, Steam 키보드는 뜰 때 포커스를 가져가 입력 칸으로 안 돌아와 창을 닫았다 열어야 했다(10-07 덱 실측). `rbq.sh` 가 `GTK_IM_MODULE`·`QT_IM_MODULE` 을 지운다. Electron 에서 앱 자판을 끄고 Steam 키보드만 쓰는 쪽은 그 포커스 문제로 버렸다.

- **프록시는 앱을 따라 죽는다** — `--exit-with-parent`. 없으면 크래시·강제 종료 뒤 고아 프록시가 포트·로봇 연결을 쥐고 남고, Steam 은 자식이 살아 있어 게임이 안 끝난 것으로 본다.
- **둘째 실행은 아무것도 띄우지 않는다** — `whenReady` 를 단일 인스턴스 잠금 결과로 막는다(`app.quit()` 은 비동기라 그사이 프록시·창을 한 번 만들었다).
- **창 안에는 우리 프록시 origin 만** — `will-navigate`·`setWindowOpenHandler` 로 막고, http(s) 는 `/open-url` 로 호스트 브라우저에 넘긴다. preload 의 `rbqDesktop`(프록시 재시작·WiFi·종료)이 바깥 페이지에 붙지 않게 하려는 것이다.
- **첫 로드 실패·렌더러 사망은 최대 3회 다시 띄운다** — 흰 화면 방지.
- **배포 채널 = release** — `steam-package.bash` 가 `RBQ_CHANNEL` 을 준다. 없으면 펌웨어 화면이 "배포 채널 없음 · 정식 빌드를 설치하세요"만 띄우고 로봇 패키지 받기가 막힌다. 앱 자가 업데이트는 프록시가 `APPIMAGE`(AppImage)·`RBQ_WIN_INSTALLER`(윈도우 설치본)로 게이트해 Steam 빌드에선 꺼진다(Steam 이 갱신).
- **호스트 프로그램에 셸 환경을 넘기지 않는다** — `hostSpawn` 이 `ELECTRON_RUN_AS_NODE`·토큰을 지운다.
- 받아 둔 로봇 패키지 목록(`/app-status`)은 AppImage 설치 트리 전용이라, Steam·윈도우 빌드에선 재시작하면 같은 버전을 다시 받는다(동작은 한다).
  같은 이유로 [자동 업데이트](`lib/autoUpdate.ts`)는 그 빌드들에서 로봇 패키지를 묻지 않는다 — 받아도 못 알아봐 켤 때마다 같은 300MB 를 다시 물었다(10-12).

## 덱 실측 (10-07, 34c94e3ce + 채널·캐시·저장 수정 전 빌드, Steam 바로가기로 실행 = sniper 3.0 컨테이너, 실로봇 DOGU)

| 항목 | 결과 |
|---|---|
| 컨테이너 실행 | ✅ `ID=steamrt VERSION_CODENAME=sniper BUILD_ID=3.0.20260805` |
| 로봇 연결·제어권 | ✅ 30 KB/s 수신, 소유자 = 덱 |
| 영상 피어 | ✅ DTLS connected. 로봇 카메라가 꺼져 있어(NoView) 프레임은 미확인 |
| 펌웨어 온라인 업데이트 | ✅ 목록 4건 → v1.2609.0 받기 327 MB → `Documents/RBQ/v1.2609.0/`, 업데이트 파일로 잡힘(전송은 안 함) |
| 바깥 페이지 차단 | ✅ `file://` 이동 막힘, `window.open` 2건 null·새 창 0, http 링크는 Steam 브라우저로 열림 |
| 렌더러 크래시 복구 | ✅ `Page.crash` → `[renderer] gone: crashed` → 2초 뒤 재로딩, 로봇 재연결·제어권 복귀 |
| 둘째 실행 | ✅ 프록시를 하나도 안 띄우고 rc=0 종료 |
| 강제 종료(kill -9) | ✅ 프록시가 2초 뒤 「부모 종료」로 따라 죽음, 포트·프로세스 0 |
| WiFi 연결(저장 프로필) | ✅ 로봇 AP → 사무실 망 → 로봇 AP 각 1초. Steam 으로 띄운 앱은 polkit 통과(ssh 세션에서는 거부됐다) |
| WiFi 비밀번호로 새 연결 | ✅ 프로필 삭제·재생성(UUID 바뀜) 후 연결, 로봇 응답 |
| 절전→복귀 | ✅ `rtcwake -m mem -s 40`(41.8초) 뒤 WiFi·로봇·제어권 자동 복귀 |
| 소리 출력(앱→덱 스피커) | ✅ 시험음 gain 0.05 → 스피커 sink monitor peak 0.050. 로봇 소리 수신은 로봇이 오디오를 안 보내 미확인 |
| 저장 창 | ⚠ 대화상자로 뜨고 취소 동작. 1231×902 라 덱 화면 아래가 잘리고 제목이 blob 주소 → 덱은 창 없이 다운로드 폴더로 바로 저장하게 고침(미실측) |

실측 함정: 같은 PC 에서 채널 없이 빌드한 적이 있으면 Metro 캐시에 옛 app.config(extra)가 굳어 `RBQ_CHANNEL` 을 줘도 번들에 채널이 안 들어간다 — `steam-package.bash` 는 `--clear` 로 굽는다. 확인은 번들에서 `extra\":{\"channel\":\"release\"`.
Steam 밖(ssh)에서 띄우면 GPU 프로세스가 죽고(`--in-process-gpu --disable-gpu` 필요) WiFi 변경도 polkit 에 막힌다 — 실측은 Steam 바로가기로 띄운 뒤 `--remote-debugging-port` 로 붙는다.

## 덱 실측 2차 (10-07 오후, 원격 영상·원격 전환 수정 빌드)

| 항목 | 결과 |
|---|---|
| 저장(덱) | ✅ 창 없이 `~/Downloads` 에 바로, 같은 이름은 `(1)` |
| 영상(LAN) | ✅ 전방 1280×720 H.264 15fps, 760 KB/s |
| 화면 키보드 | ✅ 입력칸 포커스 → 앱 자체 한글 키보드(`osk/install.ts`, focusin) |
| 원격(랑데부) 영상 | ✅ 수정 후 640×360 15fps 339 KB/s(원격은 로봇이 해상도를 낮춘다). 수정 전엔 영상 pc=null — `webrtcClient.web.ts` 가 늘 `/streamer/api/webrtc/offer`(IP:8081)로만 보냈다 |
| 로봇 WiFi 안 보일 때 원격 전환 | ✅ 수정 후 「로봇 WiFi를 찾을 수 없습니다」 없이 원격으로 붙음(프로필 ssid 를 없는 이름으로 바꿔 재현) |
| 데스크톱 모드 WiFi 창 | ✅ `/platform` = `{steamos:true,gaming:false}`, `/wifi-settings` 204 → `steam-runtime-launch-client --host kcmshell6 kcm_networkmanagement`, 창 837×693(화면 안) |
| 로봇 소리 수신 | ➖ DOGU 는 마이크 없음(`ptt_listen` 응답 `mic_present:false`) — 오디오 m-line 은 sendrecv 로 협상됨 |
| 덱 3D | ➖ 다운스케일 상태 90fps(화면 상한). 해제 시 수치는 미측정 |

원격 시험 방법: 덱이 로봇 AP 에 붙어 있으면 앱이 로봇 WiFi 로 옮겨 직결을 고르므로, 덱에서 `sudo iptables -I OUTPUT -d 192.168.0.10 -j REJECT` 로 직결만 막고 접근 코드로 로그인해 고른다(끝나면 `-D`). 데스크톱 모드에서 ssh 로 띄운 앱은 ssh 가 끝나면 죽는다 — `steam -ifrunning steam://rungameid/<바로가기 gameid>` 로 Steam 이 띄우게 한다.

## 아직 확인 안 한 것 (실기)

- 로봇 소리 수신(마이크 있는 로봇에서)
- 덱 3D 다운스케일(`RobotModel3D` SteamOS 게이트) 해제 시 프레임 — 옛 WebKitGTK 35fps 근거라 Chromium 에선 불필요할 수 있다
- 게임패드·터치만으로 전 화면 조작(조작 시험 — 사용자 담당)
