import Foundation
import Network

signal(SIGPIPE, SIG_IGN)
var fails = 0
func check(_ name: String, _ ok: Bool) { print(ok ? "PASS \(name)" : "FAIL \(name)"); if !ok { fails += 1 } }
func pause(_ s: Double) { Thread.sleep(forTimeInterval: s) }

func probe(_ port: Int) -> [String: Any] {
  let sem = DispatchSemaphore(value: 0)
  var out: [String: Any] = [:]
  let t0 = Date()
  RobotTlsCore.probe(host: "127.0.0.1", port: port) { out = $0; sem.signal() }
  sem.wait()
  print(String(format: "probe :%d -> %@ (%.2fs)", port, out.description, Date().timeIntervalSince(t0)))
  return out
}

let relays = RobotRelays()
func relay(_ fp: String) -> UInt16? {
  let sem = DispatchSemaphore(value: 0)
  var local: UInt16?
  robotTlsQueue.async { relays.relay(host: "127.0.0.1", port: 8080, fp: fp) { if case .success(let p) = $0 { local = p }; sem.signal() } }
  sem.wait()
  return local
}

func get(_ port: UInt16, _ path: String = "/api/robot/serial_number") -> String? {
  let sem = DispatchSemaphore(value: 0)
  var body: String?
  let cfg = URLSessionConfiguration.ephemeral
  cfg.timeoutIntervalForRequest = 5
  URLSession(configuration: cfg).dataTask(with: URL(string: "http://127.0.0.1:\(port)\(path)")!) { d, _, err in
    if let d, err == nil { body = String(data: d, encoding: .utf8) } else { print("  GET error: \(err?.localizedDescription ?? "?")") }
    sem.signal()
  }.resume()
  sem.wait()
  return body
}

func addr(_ port: UInt16) -> sockaddr_in {
  var a = sockaddr_in()
  a.sin_family = sa_family_t(AF_INET)
  a.sin_port = port.bigEndian
  a.sin_addr.s_addr = inet_addr("127.0.0.1")
  return a
}

func cancelAfter(_ port: UInt16, _ want: Int) {
  let s = socket(AF_INET, SOCK_STREAM, 0)
  var a = addr(port)
  let ok = withUnsafePointer(to: &a) { $0.withMemoryRebound(to: sockaddr.self, capacity: 1) { connect(s, $0, socklen_t(MemoryLayout<sockaddr_in>.size)) } } == 0
  if ok && want > 0 {
    let req = "GET /big HTTP/1.1\r\nHost: x\r\n\r\n"
    _ = req.withCString { write(s, $0, strlen($0)) }
    var buf = [UInt8](repeating: 0, count: 16384)
    var got = 0
    while got < want { let n = read(s, &buf, buf.count); if n <= 0 { break }; got += n }
  }
  close(s)
}

func occupy(_ port: UInt16) -> Int32 {
  let s = socket(AF_INET, SOCK_STREAM, 0)
  var a = addr(port)
  for _ in 0..<200 {
    if withUnsafePointer(to: &a, { $0.withMemoryRebound(to: sockaddr.self, capacity: 1) { bind(s, $0, socklen_t(MemoryLayout<sockaddr_in>.size)) } }) == 0 {
      listen(s, 1)
      return s
    }
    pause(0.01)
  }
  close(s)
  return -1
}

let mode = CommandLine.arguments[1]
let want = CommandLine.arguments[2]

if mode == "swap" {
  let r = probe(8080)
  check("probe_then_pinned_serial", r["kind"] as? String == "tls" && r["fp"] as? String == want && r["serial"] is NSNull)
  exit(fails == 0 ? 0 : 1)
}

let tls = probe(8080)
check("tls_fp", tls["kind"] as? String == "tls" && tls["fp"] as? String == want)
check("tls_serial_over_pinned_conn", tls["serial"] as? String == "RBQ-T1")
let other = probe(18084)
check("tls_other_port_no_serial", other["kind"] as? String == "tls" && other["fp"] as? String == want && other["serial"] is NSNull)
check("plain_http", probe(18081)["kind"] as? String == "plain")
check("old_robot_400_is_plain", probe(18082)["kind"] as? String == "plain")
check("rsa_is_down", probe(18083)["kind"] as? String == "down")
check("refused_is_down", probe(18099)["kind"] as? String == "down")
check("stall_is_down", probe(18085)["kind"] as? String == "down")
check("marked_plain_is_down", probe(18087)["kind"] as? String == "down")

guard let p1 = relay(want) else { check("relay_ready", false); exit(1) }
let lsof = Process()
lsof.executableURL = URL(fileURLWithPath: "/usr/sbin/lsof")
lsof.arguments = ["-nP", "-iTCP:\(p1)", "-sTCP:LISTEN"]
try? lsof.run()
lsof.waitUntilExit()
check("relay_pinned_get", get(p1)?.contains("RBQ-T1") == true)
check("relay_same_fp_same_port", relay(want) == p1)

for i in 0..<20 { cancelAfter(p1, i % 2 == 0 ? 65536 : 0) }
pause(0.5)
check("relay_survives_client_cancel", get(p1)?.contains("RBQ-T1") == true)

func relay(_ port: Int, _ fp: String) -> UInt16? {
  let sem = DispatchSemaphore(value: 0)
  var local: UInt16?
  robotTlsQueue.async { relays.relay(host: "127.0.0.1", port: port, fp: fp) { if case .success(let p) = $0 { local = p }; sem.signal() } }
  sem.wait()
  return local
}
func lose(_ port: Int) -> RobotRelay? {
  robotTlsQueue.sync {
    let r = relays.current(host: "127.0.0.1", port: port)
    r?.lost(NWError.posix(.ECONNABORTED))
    return r
  }
}

let q1 = relay(18086, want)!
_ = lose(18086)
pause(2)
check("relay_reopens_same_port", relay(18086, want) == q1 && get(q1)?.contains("RBQ-T1") == true)

let k1 = relay(18084, want)!
let lostRelay = lose(18084)
let blocker = occupy(k1)
check("port_taken_by_other", blocker >= 0)
pause(Double(RobotRelay.reopenTries) + 1.5)
check("relay_gives_up_taken_port", robotTlsQueue.sync { lostRelay?.ready == false })
let k2 = relay(18084, want)
check("relay_new_port_after_give_up", k2 != nil && k2 != k1 && get(k2!)?.contains("RBQ-T1") == true)
close(blocker)

_ = lose(8080)
pause(Double(RobotRelay.reopenTries) + 1.5)
let p2 = relay(want)
print("  used port \(p1) after loss -> \(p2.map(String.init) ?? "nil")")
check("relay_working_port_after_loss", p2 != nil && get(p2!)?.contains("RBQ-T1") == true)

let bad = RobotRelay(host: "127.0.0.1", port: 8080, fp: String(repeating: "0", count: 64))
let semBad = DispatchSemaphore(value: 0)
var pBad: UInt16?
robotTlsQueue.async { bad.start { if case .success(let p) = $0 { pBad = p }; semBad.signal() } }
semBad.wait()
check("relay_wrong_fp_fails", pBad != nil && get(pBad!) == nil)
robotTlsQueue.sync { bad.close(); relays.closeAll() }
pause(0.3)
check("closed_relay_refuses", get(p2 ?? p1) == nil)
print(fails == 0 ? "main: all pass" : "main: \(fails) failed")
exit(fails == 0 ? 0 : 1)
