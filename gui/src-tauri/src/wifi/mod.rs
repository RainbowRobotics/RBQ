use serde::Serialize;

pub mod netsh;
pub mod nmcli;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct WifiNetwork {
    pub ssid: String,
    pub signal: u8,
    pub secured: bool,
    pub saved: bool,
    pub active: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ip: Option<String>,
}

#[derive(Debug)]
pub enum WifiError {
    BackendUnavailable,
    CommandFailed(String),
    Unsupported,
}

impl WifiError {
    pub fn message(&self) -> String {
        match self {
            WifiError::BackendUnavailable => "이 시스템에서 WiFi 제어를 사용할 수 없습니다".into(),
            WifiError::CommandFailed(s) => s.clone(),
            WifiError::Unsupported => "이 운영체제는 WiFi 제어를 지원하지 않습니다".into(),
        }
    }
}

pub trait WifiBackend {
    fn scan(&self) -> Result<Vec<WifiNetwork>, WifiError>;
    fn connect(&self, ssid: &str, password: Option<&str>) -> Result<(), WifiError>;
    fn current(&self) -> Result<Option<WifiNetwork>, WifiError>;
}

pub fn backend() -> Box<dyn WifiBackend> {
    #[cfg(target_os = "linux")]
    {
        Box::new(nmcli::NmcliBackend)
    }
    #[cfg(target_os = "windows")]
    {
        Box::new(netsh::NetshBackend)
    }
    #[cfg(not(any(target_os = "linux", target_os = "windows")))]
    {
        Box::new(UnsupportedBackend)
    }
}

pub fn result_ok(req_id: &str, data: serde_json::Value) -> serde_json::Value {
    serde_json::json!({ "reqId": req_id, "ok": true, "data": data })
}

pub fn result_err(req_id: &str, message: String) -> serde_json::Value {
    serde_json::json!({ "reqId": req_id, "ok": false, "error": message })
}

pub fn req_id_of(payload: &str) -> String {
    serde_json::from_str::<serde_json::Value>(payload)
        .ok()
        .and_then(|v| v.get("reqId").and_then(|x| x.as_str()).map(String::from))
        .unwrap_or_default()
}

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
pub struct UnsupportedBackend;
#[cfg(not(any(target_os = "linux", target_os = "windows")))]
impl WifiBackend for UnsupportedBackend {
    fn scan(&self) -> Result<Vec<WifiNetwork>, WifiError> { Err(WifiError::Unsupported) }
    fn connect(&self, _: &str, _: Option<&str>) -> Result<(), WifiError> { Err(WifiError::Unsupported) }
    fn current(&self) -> Result<Option<WifiNetwork>, WifiError> { Err(WifiError::Unsupported) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn result_ok_wraps_reqid_and_data() {
        let v = result_ok("r1", serde_json::json!([{"ssid":"A"}]));
        assert_eq!(v["reqId"], "r1");
        assert_eq!(v["ok"], true);
        assert_eq!(v["data"][0]["ssid"], "A");
    }

    #[test]
    fn result_err_wraps_reqid_and_message() {
        let v = result_err("r2", "실패함".into());
        assert_eq!(v["reqId"], "r2");
        assert_eq!(v["ok"], false);
        assert_eq!(v["error"], "실패함");
    }

    #[test]
    fn req_id_of_extracts_or_empty() {
        assert_eq!(req_id_of(r#"{"reqId":"abc","ssid":"X"}"#), "abc");
        assert_eq!(req_id_of("not json"), "");
        assert_eq!(req_id_of(r#"{"ssid":"X"}"#), "");
    }
}
