import ExpoModulesCore

public class RobotTlsModule: Module {
  private let relays = RobotRelays()

  public func definition() -> ModuleDefinition {
    Name("RobotTls")

    AsyncFunction("probe") { (host: String, port: Int, promise: Promise) in
      RobotTlsCore.probe(host: host, port: port) { promise.resolve($0) }
    }

    AsyncFunction("relay") { (host: String, port: Int, fp: String, promise: Promise) in
      robotTlsQueue.async {
        self.relays.relay(host: host, port: port, fp: fp) { result in
          switch result {
          case .success(let local): promise.resolve(Int(local))
          case .failure(let e): promise.reject("ERR_ROBOT_TLS_RELAY", "\(e)")
          }
        }
      }
    }

    Function("close") { (host: String, port: Int) in
      robotTlsQueue.async { self.relays.close(host: host, port: port) }
    }

    OnDestroy {
      robotTlsQueue.async { self.relays.closeAll() }
    }
  }
}
