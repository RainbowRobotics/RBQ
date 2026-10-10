use super::{WifiBackend, WifiError, WifiNetwork};

pub fn parse_nmcli_list(out: &str) -> Vec<WifiNetwork> {
    out.lines()
        .filter(|l| !l.trim().is_empty())
        .filter_map(parse_nmcli_line)
        .collect()
}

fn parse_nmcli_line(line: &str) -> Option<WifiNetwork> {
    let f = split_terse(line);
    if f.len() < 4 { return None; }
    let ssid = f[0].clone();
    if ssid.is_empty() { return None; }
    Some(WifiNetwork {
        ssid,
        signal: f[1].parse::<u8>().unwrap_or(0),
        secured: !f[2].is_empty(),
        saved: false,
        active: f[3] == "*",
        ip: None,
    })
}

fn split_terse(line: &str) -> Vec<String> {
    let mut fields = Vec::new();
    let mut cur = String::new();
    let mut chars = line.chars();
    while let Some(ch) = chars.next() {
        match ch {
            '\\' => { if let Some(next) = chars.next() { cur.push(next); } }
            ':' => fields.push(std::mem::take(&mut cur)),
            _ => cur.push(ch),
        }
    }
    fields.push(cur);
    fields
}

#[cfg(target_os = "linux")]
pub struct NmcliBackend;

#[cfg(target_os = "linux")]
impl WifiBackend for NmcliBackend {
    fn scan(&self) -> Result<Vec<WifiNetwork>, WifiError> {
        let out = run(&["-t", "-f", "SSID,SIGNAL,SECURITY,IN-USE", "device", "wifi", "list", "--rescan", "yes"])?;
        let mut nets = parse_nmcli_list(&out);
        if let Ok(list) = run(&["-t", "-f", "NAME,UUID,TYPE", "connection", "show"]) {
            mark_saved(&mut nets, &list);
        }
        Ok(nets)
    }
    fn connect(&self, ssid: &str, password: Option<&str>) -> Result<(), WifiError> {
        if password.is_some() {
            if let Ok(list) = run(&["-t", "-f", "NAME,UUID,TYPE", "connection", "show"]) {
                for uuid in profile_uuids_for_ssid(&list, ssid) {
                    let _ = run(&["connection", "delete", "uuid", &uuid]);
                }
            }
        }
        let mut args = vec!["-w", "25", "device", "wifi", "connect", ssid];
        if let Some(pw) = password {
            args.push("password");
            args.push(pw);
        }
        run(&args).map(|_| ())
    }
    fn current(&self) -> Result<Option<WifiNetwork>, WifiError> {
        let out = run(&["-t", "-f", "NAME,TYPE,DEVICE", "connection", "show", "--active"])?;
        let wifi = out.lines().map(split_terse).find(|f| f.get(1).map(|s| s.contains("wireless")).unwrap_or(false));
        let Some(f) = wifi else { return Ok(None); };
        let device = f.get(2).cloned().unwrap_or_default();
        let ip = if device.is_empty() { None } else { current_ip(&device) };

        if let Ok(list_out) = run(&["-t", "-f", "IN-USE,SSID,SIGNAL", "device", "wifi", "list", "--rescan", "no"]) {
            if let Some((ssid, signal)) = list_out.lines().find_map(parse_inuse_row) {
                return Ok(Some(WifiNetwork { ssid, signal, secured: true, saved: true, active: true, ip }));
            }
        }
        Ok(Some(WifiNetwork { ssid: f[0].clone(), signal: 0, secured: true, saved: true, active: true, ip }))
    }
}

fn parse_inuse_row(line: &str) -> Option<(String, u8)> {
    let f = split_terse(line);
    if f.len() < 3 || f[0] != "*" { return None; }
    let ssid = f[1].clone();
    if ssid.is_empty() { return None; }
    Some((ssid, f[2].parse::<u8>().unwrap_or(0)))
}

pub fn mark_saved(nets: &mut [WifiNetwork], list_out: &str) {
    for n in nets.iter_mut() {
        n.saved = !profile_uuids_for_ssid(list_out, &n.ssid).is_empty();
    }
}

pub fn profile_uuids_for_ssid(list_out: &str, ssid: &str) -> Vec<String> {
    list_out
        .lines()
        .filter(|l| !l.trim().is_empty())
        .filter_map(|l| {
            let f = split_terse(l);
            if f.len() < 3 || !f[2].contains("wireless") || f[1].is_empty() {
                return None;
            }
            let name = &f[0];
            let dup = name
                .strip_prefix(ssid)
                .and_then(|rest| rest.strip_prefix(' '))
                .map(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()))
                .unwrap_or(false);
            if name == ssid || dup { Some(f[1].clone()) } else { None }
        })
        .collect()
}

#[cfg(target_os = "linux")]
fn current_ip(device: &str) -> Option<String> {
    let out = run(&["-t", "-f", "IP4.ADDRESS", "device", "show", device]).ok()?;
    out.lines().find_map(|l| {
        let v = l.split_once(':')?.1;
        Some(v.split('/').next()?.to_string())
    })
}

#[cfg(target_os = "linux")]
fn run(args: &[&str]) -> Result<String, WifiError> {
    use std::process::Command;
    let out = Command::new("nmcli")
        .env_remove("LD_LIBRARY_PATH")
        .env_remove("LD_PRELOAD")
        .env("LC_ALL", "C")
        .args(args)
        .output()
        .map_err(|_| WifiError::BackendUnavailable)?;
    if !out.status.success() {
        let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
        return Err(WifiError::CommandFailed(if err.is_empty() { "nmcli 명령 실패".into() } else { err }));
    }
    Ok(String::from_utf8_lossy(&out.stdout).to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wifi::WifiNetwork;

    #[test]
    fn parses_terse_list_active_and_open() {
        let out = "RBQ_EXAMPLE:72:WPA2:*\noffice_5g:55:WPA1 WPA2:\nguest:40::\n";
        let nets = parse_nmcli_list(out);
        assert_eq!(nets.len(), 3);
        assert_eq!(nets[0], WifiNetwork { ssid: "RBQ_EXAMPLE".into(), signal: 72, secured: true, saved: false, active: true, ip: None });
        assert_eq!(nets[1].secured, true);
        assert_eq!(nets[1].active, false);
        assert_eq!(nets[2], WifiNetwork { ssid: "guest".into(), signal: 40, secured: false, saved: false, active: false, ip: None });
    }

    #[test]
    fn unescapes_colon_in_ssid() {
        let nets = parse_nmcli_list("my\\:net:60:WPA2:\n");
        assert_eq!(nets.len(), 1);
        assert_eq!(nets[0].ssid, "my:net");
        assert_eq!(nets[0].signal, 60);
    }

    #[test]
    fn skips_hidden_empty_ssid() {
        let nets = parse_nmcli_list(":50:WPA2:\nvisible:44::\n");
        assert_eq!(nets.len(), 1);
        assert_eq!(nets[0].ssid, "visible");
    }

    #[test]
    fn parses_inuse_row_with_escaped_ssid() {
        let out = "*:my\\:net:66\n:office_5g:80\n";
        let hit = out.lines().find_map(parse_inuse_row);
        assert_eq!(hit, Some(("my:net".to_string(), 66)));
    }

    #[test]
    fn picks_stale_profiles_for_ssid_including_numbered_dupes() {
        let out = "RBQ_EXAMPLE_C-1:dc30:802-11-wireless\nRBQ_EXAMPLE_C-1 1:aa11:802-11-wireless\nRBQ_EXAMPLE_C-1_5G:bb22:802-11-wireless\nWired connection 1:cc33:802-3-ethernet\n";
        assert_eq!(profile_uuids_for_ssid(out, "RBQ_EXAMPLE_C-1"), vec!["dc30", "aa11"]);
    }

    #[test]
    fn stale_profiles_handle_escaped_colon_in_ssid() {
        assert_eq!(profile_uuids_for_ssid("my\\:net:ee55:802-11-wireless\n", "my:net"), vec!["ee55"]);
    }

    #[test]
    fn marks_saved_from_profile_list() {
        let mut nets = parse_nmcli_list("RBQ_EXAMPLE:72:WPA2:\noffice:55:WPA2:\n");
        mark_saved(&mut nets, "RBQ_EXAMPLE 1:aa11:802-11-wireless\nWired connection 1:cc33:802-3-ethernet\n");
        assert!(nets[0].saved);
        assert!(!nets[1].saved);
    }

    #[test]
    fn no_inuse_row_returns_none() {
        let out = ":office_5g:80\n:guest:40\n";
        let hit = out.lines().find_map(parse_inuse_row);
        assert_eq!(hit, None);
    }
}
