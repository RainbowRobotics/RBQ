package expo.modules.gamepadinput

import android.content.Context
import android.hardware.input.InputManager
import android.os.Handler
import android.os.Looper
import android.view.InputDevice
import android.view.KeyEvent
import android.view.MotionEvent
import android.view.Window
import expo.modules.kotlin.modules.Module
import expo.modules.kotlin.modules.ModuleDefinition

class GamepadInputModule : Module() {
  private var deviceListener: InputManager.InputDeviceListener? = null

  override fun definition() = ModuleDefinition {
    Name("GamepadInput")

    Events("onGamepadDevices", "onGamepadAxes", "onGamepadButton")

    Function("getDevices") { listGamepads() }

    OnActivityEntersForeground {
      installWindowCallback()
      registerDeviceListener()
    }

    OnActivityEntersBackground { unregisterDeviceListener() }
    OnDestroy { unregisterDeviceListener() }
  }


  private fun isGamepad(dev: InputDevice?): Boolean {
    if (dev == null || dev.isVirtual) return false
    val src = dev.sources
    return (src and InputDevice.SOURCE_GAMEPAD == InputDevice.SOURCE_GAMEPAD) ||
      (src and InputDevice.SOURCE_JOYSTICK == InputDevice.SOURCE_JOYSTICK)
  }

  private val CANDIDATE_KEYS = intArrayOf(
    KeyEvent.KEYCODE_BUTTON_A, KeyEvent.KEYCODE_BUTTON_B, KeyEvent.KEYCODE_BUTTON_C,
    KeyEvent.KEYCODE_BUTTON_X, KeyEvent.KEYCODE_BUTTON_Y, KeyEvent.KEYCODE_BUTTON_Z,
    KeyEvent.KEYCODE_BUTTON_L1, KeyEvent.KEYCODE_BUTTON_R1,
    KeyEvent.KEYCODE_BUTTON_L2, KeyEvent.KEYCODE_BUTTON_R2,
    KeyEvent.KEYCODE_BUTTON_THUMBL, KeyEvent.KEYCODE_BUTTON_THUMBR,
    KeyEvent.KEYCODE_BUTTON_START, KeyEvent.KEYCODE_BUTTON_SELECT, KeyEvent.KEYCODE_BUTTON_MODE,
    KeyEvent.KEYCODE_DPAD_UP, KeyEvent.KEYCODE_DPAD_DOWN, KeyEvent.KEYCODE_DPAD_LEFT, KeyEvent.KEYCODE_DPAD_RIGHT,
  )

  private fun presentKeys(dev: InputDevice): List<Int> {
    val has = dev.hasKeys(*CANDIDATE_KEYS)
    return CANDIDATE_KEYS.filterIndexed { i, _ -> has.getOrElse(i) { false } }
  }

  private fun deviceInfo(dev: InputDevice): Map<String, Any?> = mapOf(
    "id" to dev.id,
    "name" to dev.name,
    "vendorId" to dev.vendorId,
    "productId" to dev.productId,
    "descriptor" to dev.descriptor,
    "sources" to dev.sources,
    "keys" to presentKeys(dev),
    "axes" to dev.motionRanges
      .filter { it.source and InputDevice.SOURCE_JOYSTICK == InputDevice.SOURCE_JOYSTICK }
      .map {
        mapOf(
          "axis" to it.axis,
          "label" to MotionEvent.axisToString(it.axis),
          "min" to it.min,
          "max" to it.max,
          "flat" to it.flat,
        )
      },
  )

  private fun listGamepads(): List<Map<String, Any?>> =
    InputDevice.getDeviceIds()
      .toList()
      .mapNotNull { InputDevice.getDevice(it) }
      .filter { isGamepad(it) }
      .map { deviceInfo(it) }

  private fun emitDevices() {
    sendEvent("onGamepadDevices", mapOf("devices" to listGamepads()))
  }


  private fun registerDeviceListener() {
    if (deviceListener != null) return
    val im = appContext.reactContext
      ?.getSystemService(Context.INPUT_SERVICE) as? InputManager ?: return
    val listener = object : InputManager.InputDeviceListener {
      override fun onInputDeviceAdded(deviceId: Int) = emitDevices()
      override fun onInputDeviceRemoved(deviceId: Int) = emitDevices()
      override fun onInputDeviceChanged(deviceId: Int) = emitDevices()
    }
    im.registerInputDeviceListener(listener, Handler(Looper.getMainLooper()))
    deviceListener = listener
  }

  private fun unregisterDeviceListener() {
    val listener = deviceListener ?: return
    val im = appContext.reactContext
      ?.getSystemService(Context.INPUT_SERVICE) as? InputManager ?: return
    im.unregisterInputDeviceListener(listener)
    deviceListener = null
  }


  private fun installWindowCallback() {
    val window = appContext.currentActivity?.window ?: return
    val current = window.callback ?: return
    if (current is GamepadWindowCallback) return
    window.callback = GamepadWindowCallback(current, this)
  }

  internal fun handleKey(event: KeyEvent): Boolean {
    if (!isGamepad(event.device)) return false
    when (event.action) {
      KeyEvent.ACTION_DOWN, KeyEvent.ACTION_UP -> sendEvent(
        "onGamepadButton",
        mapOf(
          "deviceId" to event.deviceId,
          "keyCode" to event.keyCode,
          "label" to KeyEvent.keyCodeToString(event.keyCode),
          "down" to (event.action == KeyEvent.ACTION_DOWN),
          "repeat" to event.repeatCount,
        ),
      )
    }
    return true
  }

  internal fun handleMotion(event: MotionEvent): Boolean {
    if (!event.isFromSource(InputDevice.SOURCE_JOYSTICK) || event.action != MotionEvent.ACTION_MOVE) return false
    val dev = event.device ?: return false
    val axes = HashMap<String, Double>()
    for (range in dev.motionRanges) {
      if (range.source and InputDevice.SOURCE_JOYSTICK == InputDevice.SOURCE_JOYSTICK) {
        axes[range.axis.toString()] = event.getAxisValue(range.axis).toDouble()
      }
    }
    sendEvent("onGamepadAxes", mapOf("deviceId" to event.deviceId, "axes" to axes))
    return true
  }
}

private class GamepadWindowCallback(
  private val base: Window.Callback,
  private val module: GamepadInputModule,
) : Window.Callback by base {
  override fun dispatchKeyEvent(event: KeyEvent): Boolean {
    if (module.handleKey(event)) return true
    return base.dispatchKeyEvent(event)
  }

  override fun dispatchGenericMotionEvent(event: MotionEvent): Boolean {
    if (module.handleMotion(event)) return true
    return base.dispatchGenericMotionEvent(event)
  }
}
