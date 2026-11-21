import ExpoModulesCore
import GameController

public class GamepadInputModule: Module {


  private enum Axis {
    static let x = 0, y = 1
    static let z = 11, rz = 14
    static let hatX = 15, hatY = 16
    static let lTrigger = 17, rTrigger = 18
  }

  private enum Key {
    static let a = 96, b = 97, x = 99, y = 100
    static let l1 = 102, r1 = 103
    static let thumbL = 106, thumbR = 107
    static let start = 108, select = 109, mode = 110
  }

  private static let keyLabels: [Int: String] = [
    Key.a: "KEYCODE_BUTTON_A", Key.b: "KEYCODE_BUTTON_B",
    Key.x: "KEYCODE_BUTTON_X", Key.y: "KEYCODE_BUTTON_Y",
    Key.l1: "KEYCODE_BUTTON_L1", Key.r1: "KEYCODE_BUTTON_R1",
    Key.thumbL: "KEYCODE_BUTTON_THUMBL", Key.thumbR: "KEYCODE_BUTTON_THUMBR",
    Key.start: "KEYCODE_BUTTON_START", Key.select: "KEYCODE_BUTTON_SELECT",
    Key.mode: "KEYCODE_BUTTON_MODE",
  ]

  private static let androidSources = 0x0100_0411


  private var ids: [ObjectIdentifier: Int] = [:]
  private var nextId = 1
  private var lastAxes: [Int: [Int: Float]] = [:]
  private var observers: [NSObjectProtocol] = []

  public func definition() -> ModuleDefinition {
    Name("GamepadInput")

    Events("onGamepadDevices", "onGamepadAxes", "onGamepadButton")

    Function("getDevices") { [weak self] () -> [[String: Any?]] in
      self?.listGamepads() ?? []
    }

    OnCreate {
      self.startObserving()
    }

    OnDestroy {
      self.stopObserving()
    }
  }


  private func startObserving() {
    let center = NotificationCenter.default
    observers.append(center.addObserver(forName: .GCControllerDidConnect, object: nil,
                                        queue: .main) { [weak self] note in
      guard let self else { return }
      if let c = note.object as? GCController { self.attach(c) }
      self.emitDevices()
    })
    observers.append(center.addObserver(forName: .GCControllerDidDisconnect, object: nil,
                                        queue: .main) { [weak self] note in
      guard let self else { return }
      if let c = note.object as? GCController {
        let key = ObjectIdentifier(c)
        if let id = self.ids[key] { self.lastAxes[id] = nil }
        self.ids[key] = nil
      }
      self.emitDevices()
    })
    GCController.controllers().forEach { attach($0) }
  }

  private func stopObserving() {
    observers.forEach { NotificationCenter.default.removeObserver($0) }
    observers.removeAll()
    ids.removeAll()
    lastAxes.removeAll()
  }

  private func deviceId(_ c: GCController) -> Int {
    let key = ObjectIdentifier(c)
    if let id = ids[key] { return id }
    let id = nextId
    nextId += 1
    ids[key] = id
    return id
  }


  private func attach(_ c: GCController) {
    guard let pad = c.extendedGamepad else { return }
    let id = deviceId(c)

    pad.valueChangedHandler = { [weak self] pad, _ in
      self?.emitAxes(id: id, pad: pad)
    }

    bind(pad.buttonA, id, Key.a)
    bind(pad.buttonB, id, Key.b)
    bind(pad.buttonX, id, Key.x)
    bind(pad.buttonY, id, Key.y)
    bind(pad.leftShoulder, id, Key.l1)
    bind(pad.rightShoulder, id, Key.r1)
    bind(pad.buttonMenu, id, Key.start)
    if let options = pad.buttonOptions { bind(options, id, Key.select) }
    if let home = pad.buttonHome { bind(home, id, Key.mode) }
    if let l3 = pad.leftThumbstickButton { bind(l3, id, Key.thumbL) }
    if let r3 = pad.rightThumbstickButton { bind(r3, id, Key.thumbR) }
  }

  private func bind(_ button: GCControllerButtonInput, _ id: Int, _ keyCode: Int) {
    button.pressedChangedHandler = { [weak self] _, _, pressed in
      self?.sendEvent("onGamepadButton", [
        "deviceId": id,
        "keyCode": keyCode,
        "label": Self.keyLabels[keyCode] ?? "KEYCODE_\(keyCode)",
        "down": pressed,
        "repeat": 0,
      ])
    }
  }

  private func emitAxes(id: Int, pad: GCExtendedGamepad) {
    var frame: [Int: Float] = [
      Axis.x: pad.leftThumbstick.xAxis.value,
      Axis.y: -pad.leftThumbstick.yAxis.value,
      Axis.z: pad.rightThumbstick.xAxis.value,
      Axis.rz: -pad.rightThumbstick.yAxis.value,
      Axis.hatX: pad.dpad.xAxis.value,
      Axis.hatY: -pad.dpad.yAxis.value,
      Axis.lTrigger: pad.leftTrigger.value,
      Axis.rTrigger: pad.rightTrigger.value,
    ]

    if let prev = lastAxes[id], prev == frame { return }
    lastAxes[id] = frame

    var axes: [String: Double] = [:]
    for (code, value) in frame { axes[String(code)] = Double(value) }
    sendEvent("onGamepadAxes", ["deviceId": id, "axes": axes])
  }


  private func axisRange(_ code: Int, _ label: String, min: Double = -1) -> [String: Any] {
    ["axis": code, "label": label, "min": min, "max": 1.0, "flat": 0.0]
  }

  private func deviceInfo(_ c: GCController) -> [String: Any?]? {
    guard let pad = c.extendedGamepad else { return nil }
    let name = c.vendorName ?? "Game Controller"

    var keys: [Int] = [Key.a, Key.b, Key.x, Key.y, Key.l1, Key.r1, Key.start]
    if pad.buttonOptions != nil { keys.append(Key.select) }
    if pad.buttonHome != nil { keys.append(Key.mode) }
    if pad.leftThumbstickButton != nil { keys.append(Key.thumbL) }
    if pad.rightThumbstickButton != nil { keys.append(Key.thumbR) }

    let axes: [[String: Any]] = [
      axisRange(Axis.x, "AXIS_X"), axisRange(Axis.y, "AXIS_Y"),
      axisRange(Axis.z, "AXIS_Z"), axisRange(Axis.rz, "AXIS_RZ"),
      axisRange(Axis.hatX, "AXIS_HAT_X"), axisRange(Axis.hatY, "AXIS_HAT_Y"),
      axisRange(Axis.lTrigger, "AXIS_LTRIGGER", min: 0),
      axisRange(Axis.rTrigger, "AXIS_RTRIGGER", min: 0),
    ]

    let descriptor = "ios:\(name):\(c.productCategory)"

    return [
      "id": deviceId(c),
      "name": name,
      "vendorId": 0,
      "productId": 0,
      "descriptor": descriptor,
      "sources": Self.androidSources,
      "keys": keys,
      "axes": axes,
    ]
  }

  private func listGamepads() -> [[String: Any?]] {
    GCController.controllers().compactMap { deviceInfo($0) }
  }

  private func emitDevices() {
    sendEvent("onGamepadDevices", ["devices": listGamepads()])
  }
}
