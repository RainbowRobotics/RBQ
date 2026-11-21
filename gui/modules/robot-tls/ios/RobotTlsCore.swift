import CryptoKit
import Foundation
import Network
import Security

let robotTlsQueue = DispatchQueue(label: "expo.modules.robottls")

enum RobotTlsCore {
  static let probeTimeout: DispatchTimeInterval = .milliseconds(1500)
  static let relayConnectSeconds = 5
  static let readCap = 64 * 1024

  static let p256SpkiHeader = Data([
    0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02, 0x01,
    0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03, 0x42, 0x00,
  ])

  static func fingerprint(_ trust: sec_trust_t) -> String? {
    let t = sec_trust_copy_ref(trust).takeRetainedValue()
    guard let chain = SecTrustCopyCertificateChain(t) as? [SecCertificate],
          let leaf = chain.first,
          let key = SecCertificateCopyKey(leaf),
          let attrs = SecKeyCopyAttributes(key) as? [String: Any],
          (attrs[kSecAttrKeyType as String] as? String) == (kSecAttrKeyTypeECSECPrimeRandom as String),
          let raw = SecKeyCopyExternalRepresentation(key, nil) as Data?,
          raw.count == 65, raw.first == 0x04
    else { return nil }
    return SHA256.hash(data: p256SpkiHeader + raw).map { String(format: "%02x", $0) }.joined()
  }

  static func tlsParameters(want: String?, connectSeconds: Int? = nil, seen: @escaping (String?) -> Void = { _ in }) -> NWParameters {
    let tls = NWProtocolTLS.Options()
    let sec = tls.securityProtocolOptions
    sec_protocol_options_set_min_tls_protocol_version(sec, .TLSv12)
    sec_protocol_options_set_verify_block(sec, { _, trust, complete in
      let fp = fingerprint(trust)
      seen(fp)
      complete(want != nil && fp == want)
    }, robotTlsQueue)
    let tcp = NWProtocolTCP.Options()
    if let s = connectSeconds { tcp.connectionTimeout = s }
    return NWParameters(tls: tls, tcp: tcp)
  }

  static func endpointPort(_ port: Int) -> NWEndpoint.Port? {
    UInt16(exactly: port).flatMap { NWEndpoint.Port(rawValue: $0) }
  }

  static func request(_ host: String, _ port: Int, _ path: String) -> Data {
    Data("GET \(path) HTTP/1.1\r\nHost: \(host):\(port)\r\nConnection: close\r\n\r\n".utf8)
  }

  static func parseSerial(_ d: Data) -> String? {
    guard let sep = d.range(of: Data("\r\n\r\n".utf8)),
          let head = String(data: d[..<sep.lowerBound], encoding: .utf8)?.components(separatedBy: "\r\n").first,
          head.range(of: #"^HTTP/1\.[01] 200(\s|$)"#, options: .regularExpression) != nil,
          let obj = try? JSONSerialization.jsonObject(with: d[sep.upperBound...]) as? [String: Any],
          let sn = obj["serial_number"] as? String
    else { return nil }
    return sn.trimmingCharacters(in: .whitespacesAndNewlines)
  }

  static func plainKind(_ d: Data) -> String {
    guard d.starts(with: Data("HTTP/1.".utf8)), let sep = d.range(of: Data("\r\n\r\n".utf8)),
          let head = String(data: d[..<sep.lowerBound], encoding: .isoLatin1)
    else { return "down" }
    return head.range(of: "\r\nx-rbq-tls\\s*:", options: [.regularExpression, .caseInsensitive]) != nil ? "down" : "plain"
  }

  static func unreachable(_ e: NWError) -> Bool {
    guard case .posix(let c) = e else { return false }
    return [.ECONNREFUSED, .EHOSTUNREACH, .ENETUNREACH, .EHOSTDOWN, .ENETDOWN].contains(c)
  }

  static func probe(host: String, port: Int, done: @escaping ([String: Any]) -> Void) {
    robotTlsQueue.async { ProbeRun(host: host, port: port, done: done).start() }
  }
}

private final class ProbeRun {
  private enum Phase { case tls, serial, plain, done }

  private let host: String
  private let port: Int
  private let done: ([String: Any]) -> Void
  private var phase = Phase.tls
  private var conn: NWConnection?
  private var fp: String?
  private var sawCert = false
  private var buf = Data()

  init(host: String, port: Int, done: @escaping ([String: Any]) -> Void) {
    self.host = host
    self.port = port
    self.done = done
  }

  func start() {
    guard let p = RobotTlsCore.endpointPort(port) else { return finish(["kind": "down"]) }
    let c = NWConnection(host: NWEndpoint.Host(host), port: p, using: RobotTlsCore.tlsParameters(want: nil) { [self] fp in
      self.fp = fp
      self.sawCert = true
    })
    open(c, .tls)
    c.stateUpdateHandler = { [self] st in
      guard phase == .tls else { return }
      switch st {
      case .ready: certSeen(p)
      case .waiting(let e), .failed(let e):
        if sawCert { certSeen(p) } else if RobotTlsCore.unreachable(e) { finish(["kind": "down"]) } else { plain(p) }
      default: break
      }
    }
    c.start(queue: robotTlsQueue)
    after(.tls) { [self] in sawCert ? certSeen(p) : plain(p) }
  }

  private func certSeen(_ p: NWEndpoint.Port) {
    guard let fp else { return finish(["kind": "down"]) }
    guard port == 8080 else { return finish(["kind": "tls", "fp": fp, "serial": NSNull()]) }
    let c = NWConnection(host: NWEndpoint.Host(host), port: p, using: RobotTlsCore.tlsParameters(want: fp))
    open(c, .serial)
    let answer = { [self] in finish(["kind": "tls", "fp": fp, "serial": RobotTlsCore.parseSerial(buf) ?? NSNull()]) }
    c.stateUpdateHandler = { [self] st in
      guard phase == .serial else { return }
      switch st {
      case .ready:
        c.send(content: RobotTlsCore.request(host, port, "/api/robot/serial_number"), completion: .contentProcessed { _ in })
        read(c, .serial, enough: { _ in false }, then: answer)
      case .waiting, .failed: answer()
      default: break
      }
    }
    c.start(queue: robotTlsQueue)
    after(.serial, answer)
  }

  private func plain(_ p: NWEndpoint.Port) {
    let c = NWConnection(host: NWEndpoint.Host(host), port: p, using: .tcp)
    open(c, .plain)
    let answer = { [self] in finish(["kind": RobotTlsCore.plainKind(buf)]) }
    c.stateUpdateHandler = { [self] st in
      guard phase == .plain else { return }
      switch st {
      case .ready:
        c.send(content: RobotTlsCore.request(host, port, "/api/version"), completion: .contentProcessed { _ in })
        read(c, .plain, enough: { $0.range(of: Data("\r\n\r\n".utf8)) != nil }, then: answer)
      case .waiting, .failed: finish(["kind": "down"])
      default: break
      }
    }
    c.start(queue: robotTlsQueue)
    after(.plain, answer)
  }

  private func open(_ c: NWConnection, _ next: Phase) {
    conn?.stateUpdateHandler = nil
    conn?.cancel()
    conn = c
    phase = next
    buf = Data()
  }

  private func read(_ c: NWConnection, _ at: Phase, enough: @escaping (Data) -> Bool, then: @escaping () -> Void) {
    c.receive(minimumIncompleteLength: 1, maximumLength: RobotTlsCore.readCap) { [self] data, _, isComplete, error in
      guard phase == at else { return }
      if let data { buf.append(data) }
      if isComplete || error != nil || buf.count >= RobotTlsCore.readCap || enough(buf) { then() } else { read(c, at, enough: enough, then: then) }
    }
  }

  private func after(_ at: Phase, _ f: @escaping () -> Void) {
    robotTlsQueue.asyncAfter(deadline: .now() + RobotTlsCore.probeTimeout) { [self] in if phase == at { f() } }
  }

  private func finish(_ r: [String: Any]) {
    guard phase != .done else { return }
    phase = .done
    conn?.stateUpdateHandler = nil
    conn?.cancel()
    conn = nil
    done(r)
  }
}

final class RobotRelay {
  static let reopenTries = 5

  let host: String
  let port: Int
  let fp: String
  private(set) var local: UInt16 = 0
  private(set) var ready = false
  private var listener: NWListener?
  private var pending: ((Result<UInt16, Error>) -> Void)?
  private var conns: [ObjectIdentifier: NWConnection] = [:]
  private var closed = false
  private var tries = 0

  init(host: String, port: Int, fp: String) {
    self.host = host
    self.port = port
    self.fp = fp
  }

  func start(_ result: @escaping (Result<UInt16, Error>) -> Void) {
    pending = result
    listen()
  }

  private func listen() {
    guard !closed else { return }
    let params = NWParameters.tcp
    params.requiredInterfaceType = .loopback
    params.requiredLocalEndpoint = .hostPort(host: .ipv4(.loopback), port: NWEndpoint.Port(rawValue: local) ?? .any)
    let l: NWListener
    do { l = try NWListener(using: params) } catch { return lost(error) }
    listener = l
    l.newConnectionHandler = { [weak self] c in self?.serve(c) }
    l.stateUpdateHandler = { [weak self, weak l] st in
      guard let self, let l, self.listener === l else { return }
      switch st {
      case .ready:
        self.local = l.port?.rawValue ?? self.local
        self.ready = true
        self.tries = 0
        self.pending?(.success(self.local))
        self.pending = nil
      case .waiting(let e), .failed(let e): self.lost(e)
      case .cancelled: self.lost(NWError.posix(.ECONNABORTED))
      default: break
      }
    }
    l.start(queue: robotTlsQueue)
  }

  func lost(_ e: Error) {
    ready = false
    if let l = listener {
      listener = nil
      l.stateUpdateHandler = nil
      l.cancel()
    }
    if let p = pending {
      pending = nil
      close()
      return p(.failure(e))
    }
    tries += 1
    guard tries <= RobotRelay.reopenTries else { return close() }
    robotTlsQueue.asyncAfter(deadline: .now() + 1) { [weak self] in self?.listen() }
  }

  private func serve(_ down: NWConnection) {
    guard !closed, let p = RobotTlsCore.endpointPort(port) else { return down.cancel() }
    let up = NWConnection(host: NWEndpoint.Host(host), port: p,
                          using: RobotTlsCore.tlsParameters(want: fp, connectSeconds: RobotTlsCore.relayConnectSeconds))
    let ids = [ObjectIdentifier(down), ObjectIdentifier(up)]
    conns[ids[0]] = down
    conns[ids[1]] = up
    var shut = false
    var upReady = false
    let close: () -> Void = { [weak self] in
      if shut { return }
      shut = true
      for c in [down, up] {
        c.stateUpdateHandler = nil
        c.cancel()
      }
      ids.forEach { self?.conns[$0] = nil }
    }
    up.stateUpdateHandler = { st in
      switch st {
      case .ready:
        upReady = true
        RobotRelay.pump(down, up, close)
        RobotRelay.pump(up, down, close)
      case .waiting, .failed, .cancelled: close()
      default: break
      }
    }
    down.stateUpdateHandler = { st in
      switch st {
      case .waiting, .failed, .cancelled: close()
      default: break
      }
    }
    down.start(queue: robotTlsQueue)
    up.start(queue: robotTlsQueue)
    robotTlsQueue.asyncAfter(deadline: .now() + .seconds(RobotTlsCore.relayConnectSeconds)) { if !upReady { close() } }
  }

  private static func pump(_ from: NWConnection, _ to: NWConnection, _ close: @escaping () -> Void) {
    from.receive(minimumIncompleteLength: 1, maximumLength: 65536) { data, _, isComplete, error in
      let more = !isComplete && error == nil
      guard let data, !data.isEmpty else { return more ? pump(from, to, close) : close() }
      to.send(content: data, completion: .contentProcessed { err in
        if err == nil && more { pump(from, to, close) } else { close() }
      })
    }
  }

  func close() {
    closed = true
    ready = false
    if let l = listener {
      listener = nil
      l.stateUpdateHandler = nil
      l.cancel()
    }
    conns.values.forEach { $0.cancel() }
    conns.removeAll()
    if let p = pending {
      pending = nil
      p(.failure(NWError.posix(.ECANCELED)))
    }
  }
}

final class RobotRelays {
  private var map: [String: RobotRelay] = [:]

  func relay(host: String, port: Int, fp: String, done: @escaping (Result<UInt16, Error>) -> Void) {
    let key = "\(host):\(port)"
    if let old = map[key] {
      if old.fp == fp && old.ready { return done(.success(old.local)) }
      old.close()
    }
    let r = RobotRelay(host: host, port: port, fp: fp)
    map[key] = r
    r.start { [weak self] result in
      if case .failure = result, self?.map[key] === r { self?.map[key] = nil }
      done(result)
    }
  }

  func close(host: String, port: Int) {
    map.removeValue(forKey: "\(host):\(port)")?.close()
  }

  func current(host: String, port: Int) -> RobotRelay? { map["\(host):\(port)"] }

  func closeAll() {
    map.values.forEach { $0.close() }
    map.removeAll()
  }
}
