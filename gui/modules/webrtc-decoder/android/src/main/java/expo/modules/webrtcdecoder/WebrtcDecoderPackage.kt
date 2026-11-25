package expo.modules.webrtcdecoder

import android.app.Application
import android.content.Context
import com.oney.WebRTCModule.EglUtils
import com.oney.WebRTCModule.WebRTCModuleOptions
import com.oney.WebRTCModule.webrtcutils.H264AndSoftwareVideoDecoderFactory
import com.oney.WebRTCModule.webrtcutils.H264AndSoftwareVideoEncoderFactory
import expo.modules.core.interfaces.ApplicationLifecycleListener
import expo.modules.core.interfaces.Package
import org.webrtc.RbqLowLatencyVideoDecoderFactory

class WebrtcDecoderPackage : Package {
  override fun createApplicationLifecycleListeners(context: Context): List<ApplicationLifecycleListener> =
    listOf(object : ApplicationLifecycleListener {
      override fun onCreate(application: Application) {
        val options = WebRTCModuleOptions.getInstance()
        if (options.videoEncoderFactory != null || options.videoDecoderFactory != null) return
        val egl = EglUtils.getRootEglBaseContext() ?: return
        options.videoEncoderFactory = H264AndSoftwareVideoEncoderFactory(egl)
        options.videoDecoderFactory = RbqLowLatencyVideoDecoderFactory(H264AndSoftwareVideoDecoderFactory(egl))
      }
    })
}
