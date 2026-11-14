
private var fails = 0
private fun check(n: String, ok: Boolean) { println((if (ok) "PASS " else "FAIL ") + n); if (!ok) fails++ }

private fun timed(p: Int): Map<String, Any?> {
  val t = System.nanoTime()
  val r = probe("127.0.0.1", p)
  println("probe :$p -> $r (%.2fs)".format((System.nanoTime() - t) / 1e9))
  return r
}

private fun httpGet(port: Int, path: String = "/api/robot/serial_number"): String? = try {
  val c = java.net.URL("http://127.0.0.1:$port$path").openConnection() as java.net.HttpURLConnection
  c.connectTimeout = 5000; c.readTimeout = 5000
  c.inputStream.bufferedReader().readText()
} catch (e: IOException) { println("  GET error: $e"); null }

private fun cancelAfter(port: Int, want: Int) = try {
  Socket("127.0.0.1", port).use { s ->
    if (want > 0) {
      s.getOutputStream().write("GET /big HTTP/1.1\r\nHost: x\r\n\r\n".toByteArray())
      val buf = ByteArray(16384); var got = 0
      while (got < want) { val n = s.getInputStream().read(buf); if (n < 0) break; got += n }
    }
  }
} catch (_: IOException) {}

fun main(args: Array<String>) {
  val want = args[1]
  var uncaught = 0
  Thread.setDefaultUncaughtExceptionHandler { t, e -> uncaught++; println("UNCAUGHT in ${t.name}: $e") }

  if (args[0] == "swap") {
    val r = timed(8080)
    check("probe_then_pinned_serial", r["kind"] == "tls" && r["fp"] == want && r.containsKey("serial") && r["serial"] == null)
    kotlin.system.exitProcess(if (fails == 0) 0 else 1)
  }

  val tls = timed(8080)
  check("tls_fp", tls["kind"] == "tls" && tls["fp"] == want)
  check("tls_serial_over_pinned_conn", tls["serial"] == "RBQ-T1")
  val other = timed(18084)
  check("tls_other_port_no_serial", other["kind"] == "tls" && other["fp"] == want && other.containsKey("serial") && other["serial"] == null)
  check("plain_http", timed(18081)["kind"] == "plain")
  check("old_robot_400_is_plain", timed(18082)["kind"] == "plain")
  check("rsa_still_tls_on_android", timed(18083)["kind"] == "tls")
  check("refused_is_down", timed(18099)["kind"] == "down")
  check("stall_is_down", timed(18085)["kind"] == "down")
  check("marked_plain_is_down", timed(18087)["kind"] == "down")

  val relays = Relays()
  val p1 = relays.relay("127.0.0.1", 8080, want)
  ProcessBuilder("/usr/sbin/lsof", "-nP", "-iTCP:$p1", "-sTCP:LISTEN").inheritIO().start().waitFor()
  check("relay_pinned_get", httpGet(p1)?.contains("RBQ-T1") == true)
  check("relay_same_fp_same_port", relays.relay("127.0.0.1", 8080, want) == p1)

  repeat(40) { cancelAfter(p1, if (it % 2 == 0) 65536 else 0) }
  Thread.sleep(500)
  check("no_uncaught_on_client_cancel", uncaught == 0)
  check("relay_survives_client_cancel", httpGet(p1)?.contains("RBQ-T1") == true)

  relays.remove("127.0.0.1", 8080)!!.close()
  check("closed_relay_refuses", httpGet(p1) == null)
  val p2 = relays.relay("127.0.0.1", 8080, want)
  check("relay_reopens_after_close", p2 != p1 && httpGet(p2)?.contains("RBQ-T1") == true)

  val pBad = relays.relay("127.0.0.1", 8080, "0".repeat(64))
  check("relay_wrong_fp_fails", pBad != p2 && httpGet(pBad) == null)
  relays.removeAll().forEach { it.close() }
  Thread.sleep(300)
  check("no_uncaught_total", uncaught == 0)
  println(if (fails == 0) "main: all pass" else "main: $fails failed")
  kotlin.system.exitProcess(if (fails == 0) 0 else 1)
}
