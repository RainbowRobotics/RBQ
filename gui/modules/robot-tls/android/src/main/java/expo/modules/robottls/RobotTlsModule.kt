package expo.modules.robottls

import expo.modules.kotlin.Promise
import expo.modules.kotlin.modules.Module
import expo.modules.kotlin.modules.ModuleDefinition
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.io.Closeable
import java.io.IOException
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.ServerSocket
import java.net.Socket
import java.net.SocketTimeoutException
import java.security.MessageDigest
import java.security.cert.CertificateException
import java.security.cert.X509Certificate
import java.util.Collections
import java.util.concurrent.ConcurrentHashMap
import javax.net.ssl.SSLContext
import javax.net.ssl.SSLSocket
import javax.net.ssl.X509TrustManager
import kotlin.concurrent.thread

class RobotTlsModule : Module() {
  private val relays = Relays()

  override fun definition() = ModuleDefinition {
    Name("RobotTls")

    AsyncFunction("probe") { host: String, port: Int, promise: Promise ->
      thread(isDaemon = true, name = "robot-tls-probe") {
        promise.resolve(try { probe(host, port) } catch (_: Exception) { mapOf("kind" to "down") })
      }
    }

    AsyncFunction("relay") { host: String, port: Int, fp: String, promise: Promise ->
      try {
        promise.resolve(relays.relay(host, port, fp))
      } catch (e: Exception) {
        promise.reject("ERR_ROBOT_TLS_RELAY", e.message ?: "relay failed", e)
      }
    }

    Function("close") { host: String, port: Int ->
      relays.remove(host, port)?.let(::closeLater)
      Unit
    }

    OnDestroy { relays.removeAll().forEach(::closeLater) }
  }
}

private const val PROBE_MS = 1500
private const val RELAY_CONNECT_MS = 5000
private const val READ_CAP = 64 * 1024

private fun spkiSha256(cert: X509Certificate): String =
  MessageDigest.getInstance("SHA-256").digest(cert.publicKey.encoded).joinToString("") { "%02x".format(it) }

private class Pin(private val want: String?) : X509TrustManager {
  @Volatile var seen: String? = null

  override fun checkServerTrusted(chain: Array<X509Certificate>, authType: String) {
    val leaf = chain.firstOrNull() ?: throw CertificateException("empty chain")
    val fp = spkiSha256(leaf)
    seen = fp
    if (want == null) throw CertificateException("probe")
    if (fp != want) throw CertificateException("robot fingerprint mismatch")
  }

  override fun checkClientTrusted(chain: Array<X509Certificate>, authType: String) =
    throw CertificateException("client certificates not accepted")

  override fun getAcceptedIssuers(): Array<X509Certificate> = emptyArray()
}

private fun Closeable.closeQuietly() = try { close() } catch (_: IOException) {}

private fun connect(host: String, port: Int, timeoutMs: Int): Socket =
  Socket().also {
    try { it.connect(InetSocketAddress(host, port), timeoutMs) } catch (e: IOException) { it.closeQuietly(); throw e }
  }

private fun handshake(raw: Socket, host: String, port: Int, pin: Pin, timeoutMs: Int): SSLSocket {
  val ctx = SSLContext.getInstance("TLS").apply { init(null, arrayOf(pin), null) }
  val s = ctx.socketFactory.createSocket(raw, host, port, true) as SSLSocket
  try {
    s.enabledProtocols = s.supportedProtocols.filter { it == "TLSv1.2" || it == "TLSv1.3" }.toTypedArray()
    s.soTimeout = timeoutMs
    s.startHandshake()
  } catch (e: Exception) {
    s.closeQuietly()
    throw e
  }
  return s
}

private fun exchange(s: Socket, request: String, totalMs: Int, enough: (ByteArrayOutputStream) -> Boolean = { false }): String {
  val deadline = System.nanoTime() + totalMs * 1_000_000L
  val out = ByteArrayOutputStream()
  try {
    s.getOutputStream().apply { write(request.toByteArray(Charsets.US_ASCII)); flush() }
    val input = s.getInputStream()
    val buf = ByteArray(8192)
    while (out.size() < READ_CAP && !enough(out)) {
      val left = ((deadline - System.nanoTime()) / 1_000_000L).toInt()
      if (left <= 0) break
      s.soTimeout = left
      val n = input.read(buf)
      if (n < 0) break
      out.write(buf, 0, n)
    }
  } catch (_: SocketTimeoutException) {
  } catch (_: IOException) {
  }
  return out.toString("UTF-8")
}

private fun get(host: String, port: Int, path: String) =
  "GET $path HTTP/1.1\r\nHost: $host:$port\r\nConnection: close\r\n\r\n"

private fun parseSerial(resp: String): String? {
  if (!Regex("^HTTP/1\\.[01] 200(\\s|$)").containsMatchIn(resp.substringBefore("\r\n"))) return null
  val body = resp.substringAfter("\r\n\r\n", "")
  return try { (JSONObject(body).opt("serial_number") as? String)?.trim() } catch (_: Exception) { null }
}

private fun readSerial(host: String, port: Int, fp: String): String? = try {
  handshake(connect(host, port, PROBE_MS), host, port, Pin(fp), PROBE_MS).use {
    parseSerial(exchange(it, get(host, port, "/api/robot/serial_number"), PROBE_MS))
  }
} catch (_: Exception) {
  null
}

private fun probe(host: String, port: Int): Map<String, Any?> {
  val raw = try { connect(host, port, PROBE_MS) } catch (_: IOException) { return mapOf("kind" to "down") }
  val pin = Pin(null)
  try { handshake(raw, host, port, pin, PROBE_MS).closeQuietly() } catch (_: Exception) {}
  val fp = pin.seen
  if (fp != null) return mapOf("kind" to "tls", "fp" to fp, "serial" to (if (port == 8080) readSerial(host, port, fp) else null))
  val plain = try { connect(host, port, PROBE_MS) } catch (_: IOException) { return mapOf("kind" to "down") }
  val head = plain.use { exchange(it, get(host, port, "/api/version"), PROBE_MS) { b -> b.toString("ISO-8859-1").contains("\r\n\r\n") } }
  return mapOf("kind" to plainKind(head))
}

private fun plainKind(resp: String): String {
  val end = resp.indexOf("\r\n\r\n")
  if (!resp.startsWith("HTTP/1.") || end < 0) return "down"
  return if (Regex("\r\nx-rbq-tls\\s*:", RegexOption.IGNORE_CASE).containsMatchIn(resp.substring(0, end))) "down" else "plain"
}

private class Relay(val host: String, val port: Int, val fp: String) {
  private val server = ServerSocket(0, 50, InetAddress.getByAddress(byteArrayOf(127, 0, 0, 1)))
  val local: Int = server.localPort
  val alive get() = !server.isClosed
  private val open: MutableSet<Socket> = Collections.newSetFromMap(ConcurrentHashMap())

  init {
    thread(isDaemon = true, name = "robot-tls-relay-$local") {
      while (alive) {
        val c = try {
          server.accept()
        } catch (_: IOException) {
          if (!alive) break
          Thread.sleep(100)
          continue
        }
        thread(isDaemon = true, name = "robot-tls-conn-$local") { serve(c) }
      }
    }
  }

  private fun serve(c: Socket) {
    open += c
    val up = try {
      handshake(connect(host, port, RELAY_CONNECT_MS), host, port, Pin(fp), RELAY_CONNECT_MS).also { it.soTimeout = 0 }
    } catch (_: Exception) {
      c.closeQuietly(); open -= c
      return
    }
    open += up
    val done = { c.closeQuietly(); up.closeQuietly(); open -= c; open -= up }
    if (!alive) { done(); return }
    thread(isDaemon = true, name = "robot-tls-up-$local") { pump(c, up); done() }
    pump(up, c)
    done()
  }

  private fun pump(from: Socket, to: Socket) {
    try { from.getInputStream().copyTo(to.getOutputStream()) } catch (_: Exception) {}
  }

  fun close() {
    server.closeQuietly()
    open.toList().forEach { it.closeQuietly() }
    open.clear()
  }
}

private class Relays {
  private val map = HashMap<String, Relay>()

  @Synchronized fun relay(host: String, port: Int, fp: String): Int {
    val key = "$host:$port"
    val old = map[key]
    if (old != null && old.fp == fp && old.alive) return old.local
    map.remove(key)?.let(::closeLater)
    return Relay(host, port, fp).also { map[key] = it }.local
  }

  @Synchronized fun remove(host: String, port: Int): Relay? = map.remove("$host:$port")

  @Synchronized fun removeAll(): List<Relay> = map.values.toList().also { map.clear() }
}

private fun closeLater(r: Relay) {
  thread(isDaemon = true, name = "robot-tls-close") { r.close() }
}
