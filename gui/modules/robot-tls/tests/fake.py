"""run.sh 의 가짜 로봇. servers = 평문(18081)·옛 로봇(18082)·멈춤(18085)·늦은 TLS(18087), swap A B = 8080 TLS, 연결마다 인증서 A·B 번갈아."""
import socket, ssl, sys, threading, time


def listen(port):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(16)
    return s


def plain(c):   # 무엇이 와도 HTTP 로 답(python http.server 흉내)
    c.recv(4096); c.sendall(b"HTTP/1.0 404 Not Found\r\n\r\n"); c.close()


def old(c):     # 옛 로봇: ClientHello 에도 400 을 HTTP 로 답하고 끊는다
    c.recv(4096); c.sendall(b"HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n"); c.close()


def stall(c):   # 받기만 하고 답 없음
    time.sleep(10); c.close()


def marked(c):  # TLS 를 하는 로봇인데 핸드셰이크가 늦다(혼잡) — ClientHello 엔 답이 없고, 평문엔 X-RBQ-TLS: 1 을 단 답(실기 RestServer, R23)
    if c.recv(1, socket.MSG_PEEK) == b"\x16":
        time.sleep(10); c.close(); return
    c.recv(4096)
    c.sendall(b"HTTP/1.1 401 Unauthorized\r\nContent-Length: 0\r\nConnection: close\r\nX-RBQ-TLS: 1\r\n\r\n"); c.close()


def loop(port, f):
    s = listen(port)
    while True:
        c, _ = s.accept()
        threading.Thread(target=f, args=(c,), daemon=True).start()


if sys.argv[1] == "servers":
    for p, f in ((18081, plain), (18082, old), (18085, stall), (18087, marked)):
        threading.Thread(target=loop, args=(p, f), daemon=True).start()
    time.sleep(600)
elif sys.argv[1] == "swap":
    ctxs = []
    for name in sys.argv[2:4]:
        x = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        x.load_cert_chain(name + ".crt", name + ".pem")
        ctxs.append(x)
    s = listen(8080)
    n = 0
    while True:
        c, _ = s.accept()
        try:
            t = ctxs[n % 2].wrap_socket(c, server_side=True)
            t.recv(4096)
            body = b'{"serial_number":"SWAPPED"}'
            t.sendall(b"HTTP/1.1 200 OK\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s" % (len(body), body))
            t.close()
        except Exception:
            c.close()
        n += 1
