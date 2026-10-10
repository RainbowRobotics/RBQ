use super::WifiNetwork;
#[cfg(target_os = "windows")]
use super::{WifiBackend, WifiError};

pub fn parse_netsh_networks(out: &str) -> Vec<WifiNetwork> {
    let mut nets: Vec<WifiNetwork> = Vec::new();
    let mut cur: Option<WifiNetwork> = None;
    let mut signal_set = false;
    for line in out.lines() {
        let t = line.trim();
        let Some(idx) = t.find(':') else { continue };
        let label = t[..idx].trim();
        let val = t[idx + 1..].trim();
        if label.starts_with("SSID ") && !label.starts_with("BSSID") {
            if let Some(n) = cur.take() {
                if !n.ssid.is_empty() { nets.push(n); }
            }
            cur = Some(WifiNetwork { ssid: val.to_string(), signal: 0, secured: true, saved: false, active: false, ip: None });
            signal_set = false;
            continue;
        }
        let Some(n) = cur.as_mut() else { continue };
        if label.contains("Authentication") || label.contains("인증") {
            let open = val.eq_ignore_ascii_case("Open") || val.contains("열려");
            n.secured = !open;
        }
        if !signal_set {
            if let Some(pct) = val.strip_suffix('%') {
                if let Ok(v) = pct.trim().parse::<u8>() {
                    n.signal = v;
                    signal_set = true;
                }
            }
        }
    }
    if let Some(n) = cur.take() {
        if !n.ssid.is_empty() { nets.push(n); }
    }
    nets
}

pub fn parse_netsh_interface(out: &str) -> Option<WifiNetwork> {
    let mut connected = false;
    let mut ssid = String::new();
    let mut signal = 0u8;
    for line in out.lines() {
        let t = line.trim();
        let Some(idx) = t.find(':') else { continue };
        let label = t[..idx].trim();
        let val = t[idx + 1..].trim();
        if label.eq_ignore_ascii_case("State") || label.contains("상태") {
            connected = val.eq_ignore_ascii_case("connected") || val.contains("연결");
        } else if label.eq_ignore_ascii_case("SSID") && !label.starts_with("BSSID") {
            ssid = val.to_string();
        } else if (label.eq_ignore_ascii_case("Signal") || label.contains("신호")) && signal == 0 {
            if let Some(pct) = val.strip_suffix('%') {
                signal = pct.trim().parse::<u8>().unwrap_or(0);
            }
        }
    }
    if connected && !ssid.is_empty() {
        Some(WifiNetwork { ssid, signal, secured: true, saved: false, active: true, ip: None })
    } else {
        None
    }
}

pub fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

pub fn profile_xml(ssid: &str, key: &str) -> String {
    let s = xml_escape(ssid);
    let k = xml_escape(key);
    format!(
        r#"<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>{s}</name>
  <SSIDConfig><SSID><name>{s}</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>
    <authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption>
    <sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>{k}</keyMaterial></sharedKey>
  </security></MSM>
</WLANProfile>"#
    )
}

pub fn profile_xml_open(ssid: &str) -> String {
    let s = xml_escape(ssid);
    format!(
        r#"<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>{s}</name>
  <SSIDConfig><SSID><name>{s}</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>
    <authEncryption><authentication>open</authentication><encryption>none</encryption><useOneX>false</useOneX></authEncryption>
  </security></MSM>
</WLANProfile>"#
    )
}

#[cfg(target_os = "windows")]
pub struct NetshBackend;

#[cfg(target_os = "windows")]
impl WifiBackend for NetshBackend {
    fn scan(&self) -> Result<Vec<WifiNetwork>, WifiError> {
        Ok(parse_netsh_networks(&run(&["wlan", "show", "networks", "mode=bssid"])?))
    }
    fn connect(&self, ssid: &str, password: Option<&str>) -> Result<(), WifiError> {
        let xml = match password {
            Some(pw) => profile_xml(ssid, pw),
            None => profile_xml_open(ssid),
        };
        let mut path = std::env::temp_dir();
        path.push(format!("rbq-wifi-profile-{}.xml", std::process::id()));
        std::fs::write(&path, xml)
            .map_err(|e| WifiError::CommandFailed(format!("프로필 저장 실패: {e}")))?;
        let filename = format!("filename={}", path.to_string_lossy());
        let add = run(&["wlan", "add", "profile", &filename]);
        let _ = std::fs::remove_file(&path);
        add?;
        run(&["wlan", "connect", &format!("name={ssid}")]).map(|_| ())
    }
    fn current(&self) -> Result<Option<WifiNetwork>, WifiError> {
        Ok(parse_netsh_interface(&run(&["wlan", "show", "interfaces"])?))
    }
}

#[cfg(target_os = "windows")]
fn run(args: &[&str]) -> Result<String, WifiError> {
    use std::os::windows::process::CommandExt;
    use std::process::Command;
    const CREATE_NO_WINDOW: u32 = 0x0800_0000;
    let out = Command::new("netsh")
        .args(args)
        .creation_flags(CREATE_NO_WINDOW)
        .output()
        .map_err(|_| WifiError::BackendUnavailable)?;
    if !out.status.success() {
        let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
        return Err(WifiError::CommandFailed(if err.is_empty() { "netsh 명령 실패".into() } else { err }));
    }
    Ok(String::from_utf8_lossy(&out.stdout).to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHOW_NETWORKS: &str = "Interface name : Wi-Fi\r\nThere are 2 networks currently visible.\r\n\r\nSSID 1 : RBQ_EXAMPLE\r\n    Network type            : Infrastructure\r\n    Authentication          : WPA2-Personal\r\n    Encryption              : CCMP\r\n    BSSID 1                 : 00:11:22:33:44:55\r\n         Signal             : 72%\r\n         Radio type         : 802.11n\r\n\r\nSSID 2 : guest\r\n    Network type            : Infrastructure\r\n    Authentication          : Open\r\n    Encryption              : None\r\n    BSSID 1                 : 66:77:88:99:aa:bb\r\n         Signal             : 40%\r\n";

    #[test]
    fn parses_networks_secured_open_and_signal() {
        let nets = parse_netsh_networks(SHOW_NETWORKS);
        assert_eq!(nets.len(), 2);
        assert_eq!(nets[0].ssid, "RBQ_EXAMPLE");
        assert_eq!(nets[0].secured, true);
        assert_eq!(nets[0].signal, 72);
        assert_eq!(nets[1].ssid, "guest");
        assert_eq!(nets[1].secured, false);
        assert_eq!(nets[1].signal, 40);
    }

    const SHOW_INTERFACES: &str = "\r\nThere is 1 interface on the system:\r\n\r\n    Name                   : Wi-Fi\r\n    State                  : connected\r\n    SSID                   : RBQ_EXAMPLE\r\n    Signal                 : 88%\r\n";

    #[test]
    fn parses_interface_current() {
        let cur = parse_netsh_interface(SHOW_INTERFACES).unwrap();
        assert_eq!(cur.ssid, "RBQ_EXAMPLE");
        assert_eq!(cur.signal, 88);
        assert_eq!(cur.active, true);
    }

    #[test]
    fn interface_none_when_disconnected() {
        let out = "    Name : Wi-Fi\r\n    State : disconnected\r\n";
        assert!(parse_netsh_interface(out).is_none());
    }

    #[test]
    fn xml_escapes_special_chars() {
        assert_eq!(xml_escape("a&b<c>\"d'"), "a&amp;b&lt;c&gt;&quot;d&apos;");
    }

    #[test]
    fn profile_xml_embeds_escaped_ssid_and_key() {
        let xml = profile_xml("A&B", "p<w>");
        assert!(xml.contains("<name>A&amp;B</name>"));
        assert!(xml.contains("<keyMaterial>p&lt;w&gt;</keyMaterial>"));
        assert!(xml.contains("WPA2PSK"));
    }

    #[test]
    fn profile_xml_open_has_open_auth_and_no_key() {
        let xml = profile_xml_open("guest");
        assert!(xml.contains("<authentication>open</authentication>"));
        assert!(xml.contains("<encryption>none</encryption>"));
        assert!(!xml.contains("keyMaterial"));
    }

    #[test]
    fn profile_xml_open_escapes_ssid() {
        let xml = profile_xml_open("A&B");
        assert!(xml.contains("<name>A&amp;B</name>"));
    }
}

