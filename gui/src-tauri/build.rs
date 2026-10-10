use std::path::Path;

fn check_port_range() {
    let cap = Path::new("capabilities/remote.json");
    let src = Path::new("src/lib.rs");
    println!("cargo:rerun-if-changed=capabilities/remote.json");
    println!("cargo:rerun-if-changed=src/lib.rs");
    let (Ok(cap_txt), Ok(src_txt)) = (std::fs::read_to_string(cap), std::fs::read_to_string(src)) else {
        println!("cargo:warning=포트 범위 검사를 건너뛴다: capabilities/remote.json 또는 src/lib.rs 를 읽을 수 없다");
        return;
    };
    let allowed: Vec<u16> = cap_txt
        .split("http://localhost:")
        .skip(1)
        .filter_map(|s| s.split(|c: char| !c.is_ascii_digit()).next())
        .filter_map(|s| s.parse::<u16>().ok())
        .collect();
    let konst = |name: &str| -> Option<u16> {
        let anchor = format!("const {name}");
        src_txt.split(&anchor).skip(1).find_map(|tail| {
            let (decl, _) = tail.split_once(';')?;
            let digits: String = decl.rsplit('=').next()?.trim().to_string();
            digits.parse::<u16>().ok()
        })
    };
    let (Some(min), Some(declared)) = (konst("PORT_MIN"), konst("PORT_MAX")) else {
        println!("cargo:warning=포트 범위 검사를 건너뛴다: src/lib.rs 에서 PORT_MIN/PORT_MAX 를 못 읽었다");
        return;
    };
    if min > declared {
        println!("cargo:warning=포트 범위 검사를 건너뛴다: PORT_MIN({min}) > PORT_MAX({declared})");
        return;
    }
    let missing: Vec<u16> = (min..=declared).filter(|p| !allowed.contains(p)).collect();
    assert!(
        missing.is_empty(),
        "capabilities/remote.json 에 없는 포트가 있다: {missing:?} (PORT_MAX={declared}) — \
         그 포트로 뜬 앱은 이벤트 IPC 를 못 써 로봇 IP 를 바꿀 수 없다"
    );
}

fn main() {
    check_port_range();
    tauri_build::build()
}
