#!/usr/bin/env python3
"""QC 단계를 사람이 읽는 글로 푼다 — 웹 제품 페이지와 PDF 가 같은 글·표를 그린다(09-29).

    python3 qc_steps.py <QC 보고서 CSV>      # 단계마다 판정 · 한 줄 결과 · 기준을 터미널에(확인용)

QC 탭 CSV 의 단계표(검사·판정·측정값·기준)와 세부 값(항목,값)으로 단계마다
  무엇을 · 어떻게 · 왜     글은 여기 코드가 고정한다(qctab.cpp 단계 함수를 읽고 쓴 것)
  기준                     PASS · WARN · FAIL 이 각각 언제인지 숫자로
  측정값 표                행 = 축 · 보드 · 카메라 · 이동, 열 = 값 종류
  건너뛴 까닭              SKIP 이면 사람 말로
를 만든다. 기준 숫자는 그 검사 CSV 의 '기준' 칸에서 읽는다 — 그때 그 문턱으로 판정했으니까
(09-29 전에 돈 검사는 전류 90% · 왕복 15 ms 였다). 못 읽으면 아래 LIM(지금 qctab.cpp QcLimit ·
qcgwload.h QcGwLimit · qcaudiotest.h QcAudioLimit 값)을 쓴다.

18(ONVIF 제어 · home 프리셋)은 09-29 부터 QC 탭 단계다 — 그 전에 돈 CSV 에는 없어서 '기록 없음'으로 붙인다.
19(12시간 공회전)는 QC 탭 단계가 아니지만 늘 붙인다. <검사 폴더>/soak.json(soaktab.cpp writeJson)이 있으면
판정 · 돌린 시간 · 기준을 벗어난 시간을 옮기고, 없으면 보고서 단계(stage — 중간이면 공회전 중, 최종이면 안 함)로 적는다.
19 절의 판정 v2(통신 지연은 WARN — 09-29) · 기준 · 표 · 시간 그래프 · 진짜 이상 · 기준을 벗어난 1초는 soak_fill() 이
(qc_soak.fill_step19 — 옛 기록은 soak.csv 로 다시 센다), 공회전 끝 영상 · 20 공회전 뒤 재검사(전·후 비교)는
soak_section() 자리에서 qc_soak.py 가 채운다. 여기 _s19 의 판정 · 기준은 qc_soak 이 없거나 실패할 때만 남는다.

판정은 CSV 그대로다. 여기서는 옮겨 적고 풀어 쓰기만 한다. QC 탭 글자(선 곳 · OS · 스트로크 …)가
바뀌어도 읽히게 옛 글자와 새 글자를 둘 다 받는다.
"""
from __future__ import annotations

import html
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

# 지금 문턱 — CSV 기준 칸에서 못 읽을 때만 쓴다
LIM = {
    "hold_pass": 0.05, "hold_warn": 0.20,          # 8 흔들림 폭 [deg]
    "os_pass": 0.30, "os_warn": 1.00,              # 9 넘어간 정도 [deg]
    "err_pass": 0.25, "err_warn": 0.50,            # 9 멈춘 위치 오차 [deg]
    "lim_pass": 0.30, "lim_warn": 1.00,            # 10 리밋 오차 [deg]
    "cur_ratio": 0.95,                             # 9·10 이동 중 최대 전류 / Cur MAX — YAW(10-08 전 검사는 PITCH 도)
    "pitch_cur": 2.5,                              # 9·10 PITCH 이동 중 최대 전류 [A] — 넘으면 WARN(10-08 결정 5 A = 옛 2배 숫자, ui 58)
    "pitch_cur_max_min": 3.5,                      # 5 PITCH 설정 Cur MAX 가 이보다 작으면 FAIL(10-26 결정 실제 3.5 A, ui 58)
    "null_lo": 1500, "null_hi": 2500, "null_resid": 0.05,
    "preset_tol": 0.20, "preset_slot": 254,
    "rs485_deg": 5.0,
    "cam_first_s": 15, "cam_frames": 10, "cam_watch_s": 3,
    "cam_dark_mean": 25, "cam_dark_std": 10, "cam_flat_std": 4, "cam_freeze": 0.1, "cam_osd_std": 15,
    "mic_pass": 20, "mic_fail": 10,
    "gw_p99": 100, "gw_lossless": 500,
    "homep_pass": 0.20, "homep_warn": 0.50,            # 18 home 복귀 오차 [deg] — qctab.cpp HOMEP_TOL_*
    "yaw_lim": 168.5, "pitch_max": 86.0, "pitch_min": -20.0,
}
PRESET_AT = {"YAW": 30.0, "PITCH": 20.0}           # qctab.cpp PRESET_A — 11단계가 저장하는 자리
STROKE_DEG = {"YAW": 345.0, "PITCH": 114.0}        # 모터 펌웨어 control.c Cal.stroke_rad — 끝~끝을 이 각으로 본다
AXES = ("YAW", "PITCH")
BOARDS = (("GW", "게이트웨이"), ("YAW", "YAW"), ("PITCH", "PITCH"))
TITLE = {
    1: "통신 응답", 2: "식별 · 버전", 3: "펌웨어 최신화", 4: "드라이버 상태", 5: "설정값 (EEPROM)",
    6: "전류 영점", 7: "홈잉", 8: "정지 유지", 9: "지점 이동", 10: "소프트 리밋", 11: "프리셋 · EEPROM",
    12: "카메라 영상", 13: "스피커 → 마이크", 14: "카메라 경유 구동", 15: "원위치",
    16: "게이트웨이 통신 부하", 17: "진단 카운터", 18: "ONVIF 제어 · home 프리셋", 19: "12시간 공회전",
}
MOTION = {7, 8, 9, 10, 11, 14, 15, 18}             # 축을 움직이는 단계(qctab.cpp Step.motion)

# 5 설정값 — qctab.cpp kCfg 의 범위(QC 가 판정에 쓴 값)와 뜻
CFG = [
    (0, "BNO", "보드 번호 — YAW 1 · PITCH 2", 1, 2),
    (1, "Cur P", "전류 제어 P 게인", 0.005, 5.0),
    (2, "Cur I", "전류 제어 I 게인", 0.0, 50000.0),
    (3, "Vref MAX", "모터에 거는 최대 전압 [V]", 0.0, 12.0),
    (4, "Cur MAX", "모터에 흘리는 최대 전류 [A]", 0.001, 11.8),
    (5, "Home Zero TH", "홈잉 최소 기준 — 끝~끝 거리 [펄스]", 1000.0, 900000.0),
    (6, "Pos P", "위치 제어 P 게인", 0.0, 2000.0),
    (7, "Pos I", "위치 제어 I 게인", 0.0, 1000.0),
    (8, "Pos D", "위치 제어 D 게인", 0.0, 500.0),
    (9, "ADC U", "U 상 전류 영점 — 6단계가 새로 재기 전 값", 1500.0, 2500.0),
    (10, "ADC V", "V 상 전류 영점 — 6단계가 새로 재기 전 값", 1500.0, 2500.0),
    (11, "ADC W", "W 상 전류 영점 — 6단계가 새로 재기 전 값", 1500.0, 2500.0),
    (12, "Pos Offset", "원점 보정 각 [°]", -345.0, 345.0),
    (14, "LAN2CAN Port", "게이트웨이 통신 포트", 1.0, 65535.0),
    (15, "Cur AW", "전류 제어 안티와인드업 게인 [1/s] — 출력이 한계에 걸렸을 때 쌓인 적분을 덜어 내는 세기", 0.0, 20000.0),
    (16, "Vel Max", "움직일 때 최고 속도 [rad/s]", 0.01, 10.0),
    (17, "Accel Max", "가속 · 감속 [rad/s²]", 0.1, 100.0),
    (18, "Follow Err", "따라가기 오차 한계 [rad]", 0.05, 1.0),
    (19, "Home Vel", "홈잉 때 끝으로 다가가는 속도 [rad/s]", 0.30, 5.0),
    (20, "Pos AW", "위치 제어 안티와인드업 게인 [1/s]", 0.0, 1000.0),
    (21, "Stage2 Ratio", "2단 감속비", 0.5, 10.0),
]

# 17 진단 카운터 — 모터 펌웨어 Protocol.c 0xA8 순서. (이름, 뜻, 늘면)  늘면: FAIL · WARN · "" = 기록만
DIAG = [
    ("EEPROM 드롭", "EEPROM 에 쓸 것을 못 쓰고 버린 수", "FAIL"),
    ("EEPROM 타임아웃", "EEPROM 이 제때 응답하지 않은 수", "FAIL"),
    ("EEPROM busy", "EEPROM 이 바빠 미룬 수", ""),
    ("CAN TX 드롭", "보낼 CAN 메시지를 못 보내고 버린 수", "WARN"),
    ("드라이버 폴트", "드라이버 칩이 폴트를 낸 수", "WARN"),
    ("폴트 복구", "폴트에서 스스로 되살아난 수", ""),
    ("STATUS", "폴트를 지울 때 읽은 드라이버 STATUS(255 = 지운 적 없음)", ""),
    ("nFAULT", "nFAULT 핀 지금 상태(1 = 정상)", ""),
    ("복구 I2C", "마지막 폴트 복구 때 드라이버 칩과 주고받은(I2C) 결과(255 = 복구한 적 없음)", ""),
    ("bus-off", "CAN 통신에서 떨어져 나간 수", "WARN"),
    ("CAN 재시작", "CAN 을 다시 켠 수", ""),
    ("재시작 실패", "CAN 을 다시 켜다 실패한 수", ""),
    # 12~17 — 모터 fw 63(261002)부터. 마지막 일의 기록이라 판정에 안 쓴다(soakcore.h diagCumulative · ui 55)
    ("폴트 순간 STATUS", "드라이버가 폴트를 낸 순간의 STATUS — 폴트 원인", ""),
    ("폴트까지 ms", "모터를 켠(무장) 뒤 폴트까지 걸린 ms · 맨 위 8비트는 그때 홈 상태", ""),
    ("무장 순간 적분", "모터를 켠 순간 전류 제어의 적분값[mV]", ""),
    ("무장 직후 최대 전류", "모터를 켠 뒤 5 ms 동안 가장 큰 상전류[mA]", ""),
    ("무장 수", "모터를 켠 수", ""),
    ("홈 중 자동 복구", "홈잉 중 폴트를 스스로 지우려 한 수", ""),
]


def drv_status_text(v) -> str:
    """드라이버 STATUS(0x80) 를 사람 말로 — 데이터시트 DS13630 표 25: b0 VCC_UVLO · b1 THSD · b2 VDS_P · b3 RESET · b7 LOCK.
    soakcore.cpp SoakRule::drvStatusText 와 같은 글. 못 읽은 값(없음 · 음수 · 0xFF 넘음)이면 빈 글."""
    if v is None or v < 0 or v > 0xFF:
        return ""
    if v == 0xFF:
        return "STATUS 를 못 읽음 — STATUS 0xFF"
    c = []
    if v & 0x04:
        c.append("VDS 감시(과전류 · 단락)")
    if v & 0x01:
        c.append("게이트 전원 저전압(VCC UVLO)")
    if v & 0x02:
        c.append("과열 차단(THSD)")
    if v & 0x08:
        c.append("드라이버 리셋(전원 순간 끊김)")
    return (" · ".join(c) if c else "원인 비트 없음") + f" — STATUS 0x{v:02X}"


# ---------------------------------------------------------------- 모양
@dataclass
class Table:
    cols: list[str]
    rows: list[list] = field(default_factory=list)   # 칸 = 글자 또는 (글자, 'bad'|'warn'|'ok'|'muted')
    num: tuple = ()                                  # 오른쪽 맞춤할 열 번호
    caption: str = ""                                # 표 위 작은 제목
    note: str = ""                                   # 표 아래 한 줄
    fold: str = ""                                   # 웹에서 접어 둘 때 여는 글(PDF 는 늘 펼침)


@dataclass
class StepView:
    no: int
    name: str
    verdict: str                  # PASS · WARN · FAIL · SKIP · 중지 · 구현 전 · 대기 · 진행 중 · ''
    line: str                     # 한 줄 결과
    what: str = ""
    how: str = ""
    why: str = ""
    crit: list = field(default_factory=list)      # [(PASS·WARN·FAIL·SKIP·기록만, 글)]
    tables: list = field(default_factory=list)
    skip: str = ""                # 건너뛴 까닭(사람 말)
    notes: list = field(default_factory=list)

    @property
    def vclass(self) -> str:
        if self.verdict in ("PASS", "WARN", "FAIL", "SKIP", "중지"):
            return self.verdict
        return "WAIT" if self.verdict else ""


# ---------------------------------------------------------------- 숫자 · 글자
def esc(s) -> str:
    return html.escape("" if s is None else str(s))


def num(s) -> float | None:
    m = re.search(r"-?[0-9]+(?:\.[0-9]+)?", str(s or "").replace(",", ""))
    return float(m.group(0)) if m else None


def fx(v, nd: int = 2, unit: str = "", sign: bool = False) -> str:
    """숫자 → 글자. 없으면 '—'. 음수는 '−'(빼기 기호)로."""
    if v is None:
        return "—"
    s = f"{v:+.{nd}f}" if sign else f"{v:.{nd}f}"
    s = s.replace("-", "−")
    return s + unit


def comma(v) -> str:
    return "—" if v is None else f"{v:,.0f}"


def hms(sec) -> str:
    try:
        s = int(round(float(sec)))
    except (TypeError, ValueError):
        return "—"
    return f"{s // 3600}:{s % 3600 // 60:02d}:{s % 60:02d}"


def dur(sec) -> str:
    """초 → '3시간 12분 5초'(0 인 자리는 뺀다). 없으면 '—'."""
    try:
        s = int(round(float(sec)))
    except (TypeError, ValueError):
        return "—"
    h, m, x = s // 3600, s % 3600 // 60, s % 60
    return " ".join(f"{n}{u}" for n, u in ((h, "시간"), (m, "분"), (x, "초")) if n) or "0초"


def worse(a: str, b: str) -> str:
    rank = {"FAIL": 3, "WARN": 2, "PASS": 1}
    return a if rank.get(a, 0) >= rank.get(b, 0) else b


def grade(v, p, w) -> str:
    if v is None:
        return ""
    return "PASS" if v <= p else "WARN" if v <= w else "FAIL"


def cell_v(v: str):
    return (v, {"PASS": "ok", "WARN": "warn", "FAIL": "bad"}.get(v, "muted"))


def get(d: dict, *keys, default=""):
    """세부 값 — 이름이 바뀐 키(옛 · 새)를 차례로 찾는다."""
    for k in keys:
        if k in d:
            return d[k]
    return default


# ---------------------------------------------------------------- 문턱 — 그 검사 CSV 의 기준 칸에서
def limits(qc: dict) -> dict:
    L = dict(LIM)
    crit = {step_no(s["name"]): s.get("criterion", "") for s in qc["steps"]}
    d = dict(qc["details"])

    def pick(pat, text, *keys, scale=1.0):
        m = re.search(pat, text or "")
        if m:
            for k, g in zip(keys, m.groups()):
                if g is not None:
                    L[k] = float(g) * scale
        return bool(m)

    # 10-14 부터 5단계가 PITCH 설정 Cur MAX 를 본다. 기준 칸에 그 글이 없으면 그 전 검사 — 그 조건 없이 적는다
    if not pick(r"PITCH Cur MAX\s*≥\s*([0-9.]+)\s*A", crit.get(5, ""), "pitch_cur_max_min"):
        L["pitch_cur_max_min"] = None
    c = crit.get(8, "")
    deg = re.findall(r"([0-9]*\.?[0-9]+)\s*°", c)
    if len(deg) >= 2:
        L["hold_pass"], L["hold_warn"] = float(deg[0]), float(deg[1])
    c = crit.get(9, "")
    pick(r"(?:오버슈트|넘어간 정도)[^·]*?≤\s*([0-9.]+)\s*°[^·]*?≤\s*([0-9.]+)\s*°", c, "os_pass", "os_warn")
    pick(r"(?:정착\s*오차|위치 오차|오차)[^·]*?≤\s*([0-9.]+)\s*°[^·]*?≤\s*([0-9.]+)\s*°", c, "err_pass", "err_warn")
    if not pick(r"Cur MAX\s*[×x*]\s*([0-9.]+)", c, "cur_ratio"):
        pick(r"Cur MAX\s*의\s*([0-9.]+)\s*%", c, "cur_ratio", scale=0.01)
    # 10-08 부터 PITCH 는 Cur MAX 비율이 아니라 고정 문턱(그때 숫자 5 A = 바른 환산 2.5 A, ui 58). 기준 칸에 그 글이 없으면 그 전 검사 — PITCH 도 비율로 본다
    L["pitch_abs"] = pick(r"PITCH\s*<\s*([0-9.]+)\s*A", c, "pitch_cur")
    c = crit.get(10, "")
    pm = re.findall(r"±\s*([0-9.]+)\s*°", c)
    if len(pm) >= 3:
        L["yaw_lim"], L["lim_pass"], L["lim_warn"] = float(pm[0]), float(pm[1]), float(pm[2])
    pick(r"PITCH\s*(-?[0-9.]+)\s*~\s*(-?[0-9.]+)\s*°", c, "pitch_min", "pitch_max")
    c = crit.get(11, "")
    pick(r"슬롯\s*(\d+)", c, "preset_slot")
    pick(r"오차\s*≤\s*([0-9.]+)\s*°", c, "preset_tol")
    c = crit.get(6, "")
    pick(r"(\d+)\s*~\s*(\d+)", c, "null_lo", "null_hi")
    pick(r"≤\s*([0-9.]+)\s*A", c, "null_resid")
    c = crit.get(12, "")
    pick(r"(\d+)\s*초 안에 프레임\s*(\d+)\s*장", c, "cam_first_s", "cam_frames")
    pick(r"(\d+)\s*초 지켜봄", c, "cam_watch_s")
    pick(r"밝기\s*<\s*([0-9.]+)\s*이고\s*σ\s*<\s*([0-9.]+)", c, "cam_dark_mean", "cam_dark_std")
    pick(r"한 가지 색\s*\(\s*σ\s*<\s*([0-9.]+)", c, "cam_flat_std")
    pick(r"차\s*≤\s*([0-9.]+)", c, "cam_freeze")
    pick(r"OSD[^≥]*≥\s*([0-9.]+)", c, "cam_osd_std")
    c = crit.get(13, "") + " " + d.get("스피커→마이크.판정 기준", "")
    if not pick(r"≥\s*([0-9.]+)\s*dB\s*\(\s*WARN\s*≥\s*([0-9.]+)\s*dB", c, "mic_pass", "mic_fail"):
        pick(r"≥\s*([0-9.]+)\s*dB\s*PASS.*?([0-9.]+)\s*~", c, "mic_pass", "mic_fail")
    pick(r"([0-9.]+)\s*°\s*이상", crit.get(14, ""), "rs485_deg")
    pick(r"([0-9.]+)\s*°\s*안\s*\(\s*WARN\s*([0-9.]+)\s*°", crit.get(18, ""), "homep_pass", "homep_warn")
    c = crit.get(16, "") + " " + d.get("게이트웨이부하.판정 기준", "")
    pick(r"p99\s*(?:지연\s*)?≤\s*([0-9.]+)\s*ms", c, "gw_p99")
    pick(r"(?:무손실 상한|손실 없이 버틴 최대 부하)\s*≥\s*([0-9.]+)", c, "gw_lossless")
    return L


def step_no(name: str) -> int:
    m = re.match(r"\s*(\d+)", name or "")
    return int(m.group(1)) if m else 0


# ---------------------------------------------------------------- 이동 한 번(9·10·15)
_MOVE_KEY = re.compile(r"^(\d+) (YAW|PITCH)→(-?[0-9.]+)°$")
_MOVE_TXT = re.compile(
    r"(?P<ax>YAW|PITCH)→(?P<cmd>-?[0-9.]+)°\s*(?:선 곳|멈춘 위치)\s*(?P<pos>-?[0-9.]+)°\s*"
    r"오차\s*(?P<err>[0-9.]+)°\s*(?:OS|넘어감|넘어간 정도)\s*(?P<os>[0-9.]+)°\s*"
    r"(?:도착\s*)?(?P<t>[0-9.]+|—)\s*s\s*(?:I|최대 전류|전류)\s*(?P<i>[0-9.]+)\s*A(?P<rest>.*)")


def parse_move(text: str) -> dict | None:
    """'YAW→-90.0° 선 곳 -90.05° 오차 0.05° OS 0.03° 0.7s I 1.48A' → 값들. qc_metrics 도 쓴다."""
    m = _MOVE_TXT.search(text or "")
    if not m:
        return None
    g = m.groupdict()
    return {"axis": g["ax"], "cmd": float(g["cmd"]), "pos": float(g["pos"]), "err": float(g["err"]),
            "os": float(g["os"]), "t": None if g["t"] == "—" else float(g["t"]), "i": float(g["i"]),
            "jam": "걸림" in (g["rest"] or "")}


def moves(details, step: int) -> list[dict]:
    out = []
    for k, v in details:
        m = _MOVE_KEY.match(k)
        if not m or int(m.group(1)) != step:
            continue
        mv = parse_move(v) or {"axis": m.group(2), "cmd": float(m.group(3)), "fail": v}
        out.append(mv)
    return out


def cur_max(d: dict, ax: str) -> float | None:
    v = num(d.get(f"{ax}.CFG4 Cur MAX"))
    return v if v else None


# ---------------------------------------------------------------- SKIP 까닭
def skip_reason(no: int, raw: str, views: dict) -> tuple[str, str]:
    """(요약표에 쓸 짧은 까닭, 절 머리에 쓸 문장). 절 머리는 '건너뜀 — ' 뒤에 붙으니 문장에 '건너뛰었다'를 또 쓰지 않는다."""
    r = (raw or "").strip()
    # QC 탭 글이 바뀌어도(보고서에 적은 C++ 제안 — '… 꺼짐' · '모듈이 연결되지 않은 판' · '… 실패로 축을 움직이는 검사 건너뜀') 같게 읽는다
    if r == "선택 해제됨" or r.endswith("꺼짐") and ("전류 영점" in r or "카메라·소리" in r):
        if no == 6:
            return ("QC 탭에서 '전류 영점 보정'을 꺼 둠",
                    "QC 탭에서 '전류 영점 보정'을 꺼 두어, 모터 EEPROM 에 있던 영점을 그대로 썼다.")
        return "QC 탭에서 '카메라·소리 검사'를 꺼 둠", "QC 탭에서 '카메라·소리 검사'를 꺼 두었다."
    if r.startswith("구현 중") or "모듈이 연결되지 않은" in r:
        return ("이 검사가 없던 QC 프로그램으로 돌림",
                "이 검사가 아직 들어가지 않은 QC 프로그램으로 돌렸다.")
    if "카메라 설정 없음" in r or ("줌 설정 없음" in r and "열화상 설정 없음" in r):
        return "CONFIG 탭에 카메라 설정 없음", "CONFIG 탭에 카메라 주소와 암호가 없었다."
    if "줌 카메라 주소·암호 없음" in r:
        return "CONFIG 탭에 줌 카메라 주소나 암호 없음", "CONFIG 탭에 줌 카메라 주소나 암호가 비어 있었다."
    if "시험 파이프라인" in r:
        return ("줌 카메라가 시험 영상이라 안 함",
                "CONFIG 탭의 줌 카메라가 실제 카메라가 아니라 시험 영상(파이프라인)으로 잡혀 있어, 카메라에 프리셋을 쓰지 않았다.")
    if "기준 이미지 경로 없음" in r:
        return ("기준 펌웨어 목록 없이 돌림",
                "기준 펌웨어 목록(bin/manifest.json) 없이 QC 탭만 따로 돌려, 견줄 기준이 없었다.")
    m = re.match(r"^(\d+)\s+(.+?)\s*실패로 중단$", r)
    if m:
        return (f"{m.group(1)}단계({m.group(2)}) 실패로 검사가 멈춤",
                f"{m.group(1)}단계({m.group(2)})가 실패해 검사를 거기서 멈췄다.")
    if "앞 단계 실패로 구동 검사 중단" in r or "실패로 축을 움직이는 검사 건너뜀" in r:
        m = re.match(r"^(\d+)\s+(.+?)\s*실패로 축을", r)
        if m:
            who = f"{m.group(1)}단계({m.group(2)})"
        else:
            prev = [v for k, v in sorted(views.items()) if k < no and v.verdict == "FAIL"]
            why = next((v for v in prev if v.no in (4, 5, 6, 7)), None) or next((v for v in prev if v.no in MOTION), None)
            who = f"{why.no}단계({why.name})" if why else "앞 단계"
        return (f"{who} 실패 — 축을 움직이는 검사라 안 함",
                f"{who}가 실패해, 안전을 위해 축을 움직이는 검사는 돌리지 않았다.")
    if r == "중지됨":
        return "사람이 검사를 중지함", "이 단계에 오기 전에 사람이 검사를 중지했다."
    if r.startswith("중지됨 — "):                      # 사람이 아닌 까닭으로 멈춤(배터리 · 원격 중지 · 다른 PTZ …) — 그 까닭 그대로
        why = r[len("중지됨 — "):].strip()
        return f"검사가 멈춤 — {why}", f"이 단계에 오기 전에 검사가 멈췄다. 까닭: {why}."
    # 로봇 앱 정비 탭(qc/03) — 고른 단계만 돈 검사 · 레벨 1 · 프리셋 동의 · 원점이 잡혀 있던 7
    if r == "이번에 고르지 않음":
        return "이번에 고르지 않은 단계", "이번에는 사람이 고른 단계만 다시 돌렸고, 이 단계는 고르지 않았다."
    if r.startswith("레벨 1 점검에서 빼는 단계"):
        return ("프리셋을 바꾸는 단계라 기본 점검에서 뺌",
                "프리셋 자리를 바꾸는 단계라 기본 점검(레벨 1)에서는 돌리지 않았다.")
    if r == "프리셋 변경 동의 안 함":
        return ("프리셋 변경에 동의하지 않음",
                "검사를 시작할 때 프리셋 변경에 동의하지 않아, 프리셋을 바꾸는 이 단계는 돌리지 않았다.")
    if "원점이 잡혀 있어" in r:
        return ("원점이 잡혀 있어 다시 잡지 않음",
                "두 축의 원점이 이미 잡혀 있어서, 고른 단계만 다시 돌릴 때는 홈잉을 다시 하지 않았다.")
    if "이미 쓰이고 있어" in r:
        return ("검사용 프리셋 칸이 이미 쓰이고 있음",
                "검사용 프리셋 칸에 이미 저장된 자리가 있어 덮어쓰지 않았다.")
    if "LAN 연결이 아님" in r:
        return ("LAN 이 아니라 CAN 에 바로 연결해 안 함",
                "PC 를 LAN 이 아니라 CAN 으로 바로 연결해 돌렸다. 게이트웨이 부하 시험은 LAN 으로 연결했을 때만 한다.")
    if "굽는 중" in r:
        return "펌웨어를 굽는 중", "그때 펌웨어를 굽고 있었다."
    return (r or "건너뜀"), (r or "QC 탭이 까닭을 남기지 않았다.")


# ---------------------------------------------------------------- 단계마다
# 1단계 FAIL 까닭 — QC 탭 글(qctab.cpp commTick) → 사람 말
_COMM = [("부트에 멈춤(L2CB 배너) — 앱 없음 또는 앱 기동 실패", "부트로더에 멈춤 — 앱이 없거나 앱이 시작하지 못함"),
         ("부트에 멈춤 — 앱 없음", "부트로더에 멈춤 — 앱 없음"),
         ("부트에 멈춤 — 부트 모드 대기", "부트로더에 멈춤 — 부트 모드로 기다리는 중"),
         ("앱이 부트 조회엔 답함 — CAN_CHECK 만 안 옴", "앱이 부트로더 조회에는 응답함 — 확인 신호(CAN_CHECK)만 안 옴")]


def _comm_plain(label: str, why: str) -> str:
    for a, b in _COMM:
        if why == a:
            return b
    # 모터 보드의 '응답 없음'은 부트로더 조회에도 답이 없었다는 뜻이다(게이트웨이는 부트로더에 묻지 않는다)
    return "응답 없음(부트로더도 응답 없음)" if why == "응답 없음" and label != "게이트웨이" else why


def serial_unknown(sn: str) -> bool:
    """시리얼 칸이 없는 옛 펌웨어 — QC 탭 '모름 — …'(10-11) · '미상 — 시리얼 칸 응답 없음…'(10-08)."""
    return sn.startswith("모름 —") or (sn.startswith("미상") and "응답 없음" in sn)


def serial_empty(sn: str) -> bool:
    return sn in ("비어 있음", "미상 — 칸 비어 있음")


def serial_write_bad(wr: str) -> bool:
    return wr.startswith(("써 넣지 못함", "써 넣었지만", "못 씀", "썼으나"))


def serial_write_text(wr: str) -> str:
    """'시리얼.써 넣기' → 사람 말. 10-08 판 QC 탭 글('못 씀 — …')도 같은 말로."""
    old = {"못 씀 — 확정 응답 없음": "써 넣지 못함(모터 보드가 썼다는 답을 보내지 않음)",
           "못 씀 — 모터가 형식이 틀리다고 거절": "써 넣지 못함(모터 보드가 형식이 틀리다고 거절)",
           "썼으나 다시 읽기 무응답": "써 넣었지만 다시 읽을 때 응답 없음"}
    if wr in old:
        return old[wr]
    m = re.match(r"^못 씀 — EEPROM 쓰기 실패\(결과 (\d+)\)$", wr)
    if m:
        return f"써 넣지 못함(EEPROM 쓰기 실패, 결과 {m.group(1)})"
    m = re.match(r"^썼으나 다시 읽은 값이 다름\((.*)\)$", wr)
    if m:
        return f"써 넣었지만 다시 읽은 값이 다름({m.group(1)})"
    return wr


def fb_counted(d: dict, stage: str, missing: int) -> bool:
    """부하 시험 그 단계의 위치 신호 손실을 QC 탭이 손실로 셌나 — QC 탭은 축마다 FB_TOL 개를 봐주고, 넘은 단계만
    판정 사유에 '<단계> 피드백 손실 N장'으로 적는다(qcgwload.cpp). 사유 칸이 없는 옛 CSV 는 두 축 합계로 어림한다."""
    why = d.get("게이트웨이부하.판정 사유")
    if why is None:
        return missing > 2 * FB_TOL
    return re.search(rf"(?:^|[;·]\s*){re.escape(stage)} 피드백 손실 \d+", why) is not None


def _s1(v: StepView, s: dict, d: dict, L: dict):
    v.what = ("PC 가 게이트웨이와 두 모터 보드에 말을 걸어 셋 다 응답하는지 본다. PTZ 에는 카메라를 좌우로 돌리는 YAW 축과 "
              "위아래로 기울이는 PITCH 축이 있고, 축마다 모터 보드가 하나씩 있다. 게이트웨이는 PC 와 모터 보드 사이에서 통신을 "
              "이어 주는 보드다.")
    v.how = ("'살아 있나' 확인 신호(CAN_CHECK)를 셋에 한꺼번에 보내고, 응답이 없으면 0.8초마다 다시 보내며 2.5초까지 기다린다. "
             "응답이 없는 모터 보드에는 부트로더에 멈춰 있는지 한 번 더 물어, 보드가 아예 죽었는지 펌웨어가 없어 부트로더에서 "
             "멈췄는지 가려 적는다. 부트로더는 펌웨어를 칩에 새로 써 넣을 때(굽을 때) 쓰는 작은 시작 프로그램이다.")
    v.why = "응답이 없으면 뒤의 어떤 검사도 할 수 없다."
    v.crit = [("PASS", "세 보드가 2.5초 안에 모두 응답한다."),
              ("FAIL", "하나라도 응답이 없다. 뒤 단계는 모두 건너뛴다.")]
    t = Table(["보드", "응답"])
    bad = []
    for key, label in BOARDS:
        why = d.get(f"통신.{label}")
        if why:
            t.rows.append([label, (_comm_plain(label, why), "bad")])
            bad.append(f"{label} {_comm_plain(label, why)}")
        elif s["verdict"] == "PASS":
            t.rows.append([label, ("응답함", "ok")])
        else:
            t.rows.append([label, ("응답함", "ok") if v.verdict == "FAIL" else "—"])
    v.tables.append(t)
    v.line = ("게이트웨이 · YAW · PITCH 모두 응답함" if s["verdict"] == "PASS"
              else " · ".join(bad) if bad else plain(s["measured"]))


def _s2(v: StepView, s: dict, d: dict, L: dict):
    v.what = ("세 보드의 펌웨어(앱) 버전을 읽고 부트로더가 제자리에 있는지 본다. 게이트웨이 칩의 고유 번호(UID)와 YAW 보드에 "
              "저장된 제품 시리얼도 읽는다.")
    v.how = ("보드마다 앱 버전과 부트 정보(BOOT_INFO)를 묻는다. 부트 정보에는 부트로더가 있는지와 부트로더 · 앱의 CRC 가 들어 있다. "
             "CRC 는 파일 내용으로 계산한 짧은 숫자(0x 로 시작하는 16진수)로, 내용이 조금만 달라도 바뀌어서 지문처럼 쓴다. "
             "게이트웨이 칩 UID 는 칩마다 다른 번호라 보증서를 이 번호로 묶는다. 출하 검사에서는 시리얼 칸이 비어 있으면 검사를 "
             "시작할 때 입력한 시리얼을 써 넣고 다시 읽어 확인한다.")
    v.why = ("부트로더가 없으면 통신선으로는 펌웨어를 새로 구울 수 없고, SWD 장비(보드에 직접 꽂아 펌웨어를 쓰는 장비)를 "
             "연결해야 한다. YAW 와 PITCH 는 같은 펌웨어를 쓰므로 버전이 달라서는 안 된다.")
    v.crit = [("PASS", "세 보드 모두 응답하고 부트로더가 있다. QC 탭에 기준 버전을 적었으면 그 버전과 같다(비워 두면 기록만 한다)."),
              ("WARN", "YAW 와 PITCH 버전이 다르거나, 게이트웨이 UID 를 못 읽었거나, 이번에 전원을 켤 때 부트로더를 "
                       "거치지 않았다(디버거로 앱을 바로 띄운 경우 등). YAW 보드에 저장된 제품 시리얼이 검사 때 입력한 "
                       "시리얼과 다르거나, 빈 칸에 시리얼을 써 넣지 못해도 WARN 이다."),
              ("FAIL", "앱 버전이나 부트 정보에 응답이 없거나, 부트로더가 없거나, 기준 버전과 다르다.")]
    t = Table(["항목", "게이트웨이", "YAW", "PITCH"])

    def row(label, prefix, fmt=lambda x: x):
        t.rows.append([label] + [fmt(d.get(prefix + b, "—")) for b, _ in BOARDS])
    row("앱 버전", "버전.")
    row("부트로더", "부트.", lambda x: ("확인 못 함 — 옛 펌웨어라 응답 없음", "bad") if x.startswith("확인 불가")
        else (x, "bad") if x == "없음" else x)
    row("부트로더 CRC", "부트CRC.")
    row("앱 CRC", "앱CRC.")
    uid = d.get("UID.GW", "")
    t.rows.append(["칩 UID (보증 기준)", ("못 읽음", "warn") if uid == "무응답" else (uid or "—"), ("—", "muted"), ("—", "muted")])
    sn = d.get("시리얼.제품")
    sn_tail = ""
    if sn is not None:                                  # 10-08 부터 — YAW 보드 EEPROM 의 제품 시리얼 칸
        md = re.search(r"시리얼 \S+ — 입력한 (?:시리얼\()?([A-Z0-9-]{1,16})\)? ?[과와] 다름", s.get("measured") or "")
        typed = md.group(1) if md else ""
        wr = serial_write_text(d.get("시리얼.써 넣기", ""))
        if serial_unknown(sn):
            snc = ("모름(시리얼 칸이 없는 옛 펌웨어)", "muted")
        elif sn in ("모름", "미상"):
            snc = ("모름(검사가 시리얼을 읽기 전에 끊김)", "muted")
        elif serial_empty(sn):
            snc = ("비어 있음", "muted")
        elif typed and typed != sn:
            snc = (f"{sn} (입력한 시리얼은 {typed})", "warn")
            sn_tail = f"제품 시리얼({sn})이 입력한 시리얼({typed})과 다름"
        else:
            snc = sn
        if wr and serial_write_bad(d.get("시리얼.써 넣기", "")):
            snc = (("비어 있음 · " if serial_empty(sn) else f"{sn} · ") + wr, "warn")
            sn_tail = (sn_tail + " · " if sn_tail else "") + f"시리얼을 {wr}"
        elif wr:
            snc = f"{sn} (이번 검사가 써 넣음)"
        t.rows.append(["제품 시리얼", ("—", "muted"), snc, ("—", "muted")])
    t.note = ("UID 는 게이트웨이(LAN2CAN) 칩에서만 읽는다. 제품 시리얼은 YAW 보드 EEPROM 에 저장되고, 처음에는 출하 검사가 써 "
              "넣는다. 시리얼 칸이 없는 옛 펌웨어는 '모름'으로 적는다." if sn is not None else "UID 는 게이트웨이(LAN2CAN) 칩에서만 읽는다.")
    v.tables.append(t)
    msr = s["measured"] or ""
    # 이번 전원에 부트로더를 안 거친 보드(WARN) — QC 탭 글 'YAW 앱 … · 부트 0x…(이번 전원에 부트 안 거침)'
    names = {"GW": "게이트웨이", "YAW": "YAW", "PITCH": "PITCH"}
    noboot = [names[m.group(1)] for m in re.finditer(r"(GW|YAW|PITCH) 앱 \S+ · 부트 0x[0-9a-fA-F]+\(이번 전원에 부트 안 거침\)", msr)]
    noboot_t = (f"이번에 켤 때 부트로더를 안 거침: {', '.join(noboot)}" if noboot
                else "이번에 켤 때 부트로더를 안 거친 보드 있음" if "부트 안 거침" in msr else "")
    if s["verdict"] in ("PASS", "WARN") and d.get("버전.GW"):
        gw, ya, pi = d.get("버전.GW", "?"), d.get("버전.YAW", "?"), d.get("버전.PITCH", "?")
        mot = f"YAW·PITCH {ya}" if ya == pi else f"YAW {ya}, PITCH {pi}(다름)"
        boots = all(d.get(f"부트.{b}") == "있음" for b, _ in BOARDS)
        uidt = ("게이트웨이 UID 읽음" if uid and uid != "무응답" else "게이트웨이 UID 못 읽음" if uid == "무응답"
                else "UID 는 안 읽음(UID 를 읽기 전 QC 프로그램)")
        v.line = (f"펌웨어 버전 게이트웨이 {gw}, {mot} · 부트로더 {'셋 다 있음' if boots else '있는지 확인 못 함'} · {uidt}"
                  + (f" · {noboot_t}" if noboot_t else "") + (f" · {sn_tail}" if sn_tail else ""))
    else:
        bad = []
        for key, label in BOARDS:
            if d.get(f"부트.{key}") == "없음":
                bad.append(f"{label} 부트로더 없음")
            elif str(d.get(f"부트.{key}", "")).startswith("확인 불가"):
                bad.append(f"{label} 부트 정보에 응답 없음(옛 펌웨어)")
        if "앱 버전 무응답" in msr:
            bad.append("앱 버전에 응답 없음")
        ya, pi = d.get("버전.YAW", "?"), d.get("버전.PITCH", "?")
        for m in re.finditer(r"(GW|모터) 기준 (\S+) 과 다름", msr):
            got = d.get("버전.GW", "?") if m.group(1) == "GW" else (ya if ya == pi else f"YAW {ya} · PITCH {pi}")
            bad.append(f"{'게이트웨이' if m.group(1) == 'GW' else '모터'} 앱 {got} — 기준({m.group(2)})과 다름")
        if "YAW·PITCH 버전이 다름" in msr:
            bad.append(f"YAW 와 PITCH 앱 버전이 다름(YAW {ya} · PITCH {pi})")
        if uid == "무응답":
            bad.append("게이트웨이 UID 못 읽음")
        if noboot_t:
            bad.append(noboot_t)
        v.line = " · ".join(bad) if bad else plain(msr)


def _s3(v: StepView, s: dict, d: dict, L: dict):
    v.what = "세 보드의 펌웨어가 출하 기준 파일과 똑같은지 CRC 로 견주고, 다르면 새로 굽는다."
    v.how = ("기준 파일 목록(PTZ_GUI/bin/manifest.json)에 적힌 CRC 와 2단계에서 읽은 앱 CRC 를 견준다. "
             "CRC 가 다르고 QC 탭의 '구버전이면 자동으로 굽기'가 켜져 있으면, 통신선으로(OTA) YAW, PITCH, "
             "게이트웨이 차례로 굽는다. 보드가 다시 응답하면 CRC 를 다시 읽어 같아졌는지 확인한다.")
    v.why = "버전 번호가 같아도 다시 빌드한 파일이면 내용이 다를 수 있다. CRC 는 그것까지 가른다."
    v.crit = [("PASS", "세 보드의 앱 CRC 가 기준과 같다(처음부터 같거나, 구운 뒤 같아짐)."),
              ("FAIL", "굽다가 실패했거나, 보드 하나를 굽는 데 10분을 넘겼거나, 구운 뒤 60초 안에 응답이 없거나, 구운 뒤에도 "
                       "CRC 가 다르다. 자동 굽기를 꺼 둔 채 CRC 가 다르거나 기준 목록을 못 읽어도 FAIL 이다. 뒤 단계는 모두 건너뛴다."),
              ("SKIP", "기준 목록 경로 없이 QC 탭만 따로 돌렸을 때.")]
    if d.get("검사.레벨") == "1":
        # 로봇 기본 점검(레벨 1) — 굽지 않고 견주기만 한다(qc/03 §2)
        v.what = ("세 보드의 펌웨어가 출하 기준 파일과 똑같은지 CRC 로 견주기만 한다. 이번 검사는 기본 점검(레벨 1)이다. 기본 "
                  "점검에서는 펌웨어를 굽지 않고, 전류 영점을 다시 재지 않고, 프리셋을 바꾸는 11 · 18단계를 뺀다.")
        v.how = "기준 파일 목록에 적힌 CRC 와 2단계에서 읽은 앱 CRC 를 견준다. 다르면 다르다고만 적고 다음 단계로 간다."
        v.crit = [("PASS", "세 보드의 앱 CRC 가 기준과 같다."),
                  ("WARN", "기준과 다른 보드가 있다 — 최신 펌웨어가 아니다. 굽기는 전체 점검(레벨 2 이상)에서 한다."),
                  ("FAIL", "기준 목록을 못 읽었다. 뒤 단계는 모두 건너뛴다."),
                  ("SKIP", "기준 목록 없이 돌렸을 때.")]
    t = Table(["보드", "굽기 전 앱 CRC", "기준 CRC", "기준 파일", "구웠나", "굽기 뒤 CRC", "굽는 데 걸린 시간"], num=(6,))
    any_row = False
    for key, label in BOARDS:
        k = f"최신화.{key} "
        if k + "굽기 전 CRC" not in d:
            continue
        any_row = True
        want = d.get(k + "기준", "")
        m = re.match(r"(0x[0-9a-fA-F]+)\s*\((.*)\)\s*$", want)
        crc, img = (m.group(1), m.group(2)) if m else (want, "")
        img = re.sub(r"커밋 ([0-9a-f]{6,}) · 미커밋 변경 (\d+)", r"소스 커밋 \1 (커밋 안 된 변경 \2건)", img)
        img = re.sub(r"(?<!소스 )커밋 ([0-9a-f]{6,})", r"소스 커밋 \1", img)
        done = d.get(k + "구웠나", "—")
        cls = "bad" if done.startswith(("실패", "구웠으나")) or "없음" in done or "꺼짐" in done else ""
        done = {"아니오 — 같음": "안 구움 — 이미 같음", "아니오 — 레벨 1 은 견주기만": "안 구움 — 기본 점검은 견주기만"}.get(
            done, done.replace("구웠으나 무응답", "구웠으나 응답 없음"))
        if done.startswith("안 구움 — 기본 점검"):
            cls = "warn"
        secs = num(d.get(k + "걸린 시간[s]"))
        t.rows.append([label, d.get(k + "굽기 전 CRC", "—"), crc, img, (done, cls) if cls else done,
                       d.get(k + "굽기 뒤 CRC", "—"), "—" if not secs else fx(secs, 1, " s")])
    if any_row:
        man = d.get("최신화.기준 목록", "—")
        man = re.sub(r"(\d{4}-\d\d-\d\d)T(\d\d:\d\d)", r"\1 \2", man)
        man = re.sub(r"(?<!소스 )커밋 ([0-9a-f]{6,})", r"소스 커밋 \1", man)
        t.note = ("기준 목록 — " + man + ". 소스 커밋은 펌웨어 소스를 저장소에 기록해 둔 번호이고, "
                  "커밋 안 된 변경은 그 뒤에 고치고 아직 기록하지 않은 것이다.")
        v.tables.append(t)
    msr = s["measured"]
    if s["verdict"] == "WARN" and "견주기만" in msr:
        old = re.findall(r"(GW|YAW|PITCH) 0x", msr)
        names = {"GW": "게이트웨이", "YAW": "YAW", "PITCH": "PITCH"}
        v.line = (f"{', '.join(dict.fromkeys(names[o] for o in old)) or '일부 보드의'} 펌웨어가 기준과 달라 최신이 아님 · "
                  "기본 점검(레벨 1)이라 굽지 않음")
    elif s["verdict"] == "PASS" and "같음" in msr and "굽고" not in msr:
        v.line = "세 보드 모두 기준 펌웨어와 같아 굽지 않음"
    elif s["verdict"] == "PASS":
        v.line = plain(msr).replace("굽고 확인 — ", "구워서 기준과 맞춤 — ")
    else:
        v.line = plain(msr)


def _s4(v: StepView, s: dict, d: dict, L: dict):
    v.what = "모터를 돌리는 드라이버 칩이 스스로 이상을 알리고 있지 않은지 본다."
    v.how = ("두 모터 보드에 축 상태를 물어 세 가지를 읽는다. 드라이버가 이상을 감지했다는 폴트 표시, 이상을 알리는 "
             "nFAULT 핀(정상이면 High, 이상이면 Low), 그리고 칩 안에서 저전압이나 과열, 과전류, 잠금을 적어 두는 STATUS "
             "레지스터다. 모터 펌웨어는 폴트가 났을 때만 STATUS 를 읽기 때문에, 폴트가 없던 보드는 '읽은 적 없음'으로 나오는 "
             "게 정상이다. 표의 '모터 구동'은 모터에 힘을 주고 있는지를 적은 것이고, 6단계는 이 값이 꺼짐인 축만 남은 전류를 잰다.")
    v.why = "드라이버가 이상한 채로 모터를 돌리면 제대로 움직이지 않거나 부품이 상할 수 있다."
    v.crit = [("PASS", "두 축 모두 폴트가 없고, nFAULT 가 High 이고, STATUS 에 이상 표시가 없다."),
              ("FAIL", "위 셋 가운데 하나라도 이상을 가리키거나 응답이 없다. 축을 움직이는 검사는 건너뛴다.")]
    t = Table(["축", "드라이버 폴트", "nFAULT 핀", "STATUS 레지스터", "모터 구동"])
    for ax in AXES:
        st = d.get(f"{ax}.축상태")
        if st is None:
            continue
        try:
            bits = int(st, 16)
        except ValueError:
            bits = 0
        flt = ("있음", "bad") if bits & 0x20 else ("없음", "ok")
        pin = num(d.get(f"{ax}.nFAULT"))
        pin_t = (("High (정상)", "ok") if pin == 1 else ("Low (이상)", "bad") if pin == 0
                 else ("모름(옛 펌웨어라 안 알려 줌)", "muted"))
        sv = num(d.get(f"{ax}.DRV_STATUS"))
        if sv == 255:
            stt = ("읽은 적 없음 (정상)", "muted")
        elif sv == 254 or sv is None:
            stt = ("모름(옛 펌웨어라 안 알려 줌)", "muted")
        else:
            names = [n for b, n in ((1, "저전압"), (2, "과열"), (4, "과전류"), (8, "잠금")) if int(sv) & b]
            stt = (" · ".join(names), "bad") if names else ("이상 표시 없음", "ok")
        drv = "꺼짐" if bits & 0x10 else "켜짐"
        t.rows.append([ax, flt, pin_t, stt, drv])
    if t.rows:
        v.tables.append(t)
    v.line = "두 축 드라이버 모두 폴트(이상 표시) 없음 · nFAULT 핀 High(정상)" if s["verdict"] == "PASS" else plain(s["measured"])


def _s5(v: StepView, s: dict, d: dict, L: dict):
    v.what = "모터 보드마다 EEPROM(전원이 꺼져도 남는 저장 칸)에 든 설정 21가지를 모두 읽어, QC 가 허용하는 범위 안인지 본다."
    v.how = ("YAW · PITCH 보드에 설정 전체를 한꺼번에 묻고, 빠진 항목은 하나씩 다시 묻는다. "
             "보드 번호(BNO)가 축과 맞는지(YAW 1, PITCH 2)도 본다. 전류 영점(9~11번)은 여기서 보지 않고 6단계가 새로 재서 본다.")
    v.why = "설정이 틀린 채 모터를 돌리면 너무 세게 돌거나 엉뚱한 축이 움직일 수 있다."
    cm = L.get("pitch_cur_max_min")
    if cm:
        v.crit = [("PASS", f"두 축 21항목 모두 응답하고, BNO 가 축과 맞고, 모두 범위 안이고, PITCH 의 최대 전류(Cur MAX)가 {cm:g} A 이상이다."),
                  ("FAIL", "응답 없는 항목이 있거나, BNO 가 축과 뒤바뀌었거나, 범위 밖 값이 있거나, PITCH Cur MAX 가 "
                           f"{cm:g} A 보다 작다(10-26 전 펌웨어의 상한이 남은 제품 — 무거운 쪽을 들 때 전류 여유가 없다). 축을 움직이는 검사는 건너뛴다.")]
    else:
        v.crit = [("PASS", "두 축 21항목 모두 응답하고, BNO 가 축과 맞고, 모두 범위 안이다."),
                  ("FAIL", "응답 없는 항목이 있거나, BNO 가 축과 뒤바뀌었거나, 범위 밖 값이 있다. 축을 움직이는 검사는 건너뛴다.")]
    t = Table(["번호", "항목", "뜻", "YAW", "PITCH", "QC 허용 범위"], num=(0, 3, 4),
              fold="설정 21가지 값 모두 보기")
    for idx, name, mean, lo, hi in CFG:
        cells = []
        for ax in AXES:
            raw = d.get(f"{ax}.CFG{idx} {name}")
            val = num(raw)
            if raw is None:
                cells.append(("—", "muted"))
            elif idx not in (9, 10, 11) and val is not None and not (lo - 1e-5 * max(1, abs(lo), abs(hi)) <= val
                                                                      <= hi + 1e-5 * max(1, abs(lo), abs(hi))):
                cells.append((raw, "bad"))
            else:
                cells.append(raw)
        if idx == 0:
            for i, ax in enumerate(AXES):
                if num(d.get(f"{ax}.CFG0 BNO")) not in (None, i + 1.0):
                    cells[i] = (cells[i] if isinstance(cells[i], str) else cells[i][0], "bad")
        rng = f"{lo:g} ~ {hi:g}".replace("-", "−")
        t.rows.append([str(idx), name, mean, *cells, rng])
    if any(f"{ax}.CFG0 BNO" in d for ax in AXES):
        t.note = ("13번은 펌웨어에서 쓰지 않는 번호라 없다. P · I · D 게인은 제어가 오차에 얼마나 세게 반응할지 정하는 수다. "
                  "단위를 적지 않은 값은 펌웨어가 쓰는 값 그대로다.")
        v.tables.append(t)
    m = re.search(r"Cur MAX\s*([0-9.]+)/([0-9.]+)\s*A", s["measured"])
    if s["verdict"] == "PASS" and m:
        v.line = f"두 축 21항목 모두 범위 안 · 최대 전류(Cur MAX) YAW {m.group(1)} A, PITCH {m.group(2)} A"
    else:
        v.line = plain(s["measured"])


def _s6(v: StepView, s: dict, d: dict, L: dict):
    lo, hi, res = L["null_lo"], L["null_hi"], L["null_resid"]
    v.what = "모터 전류 센서가 '0 A' 일 때 내는 값(영점)을 새로 재서 저장하고, 그 값이 정상 범위인지 본다."
    v.how = ("모터 전류를 끊고 1초 기다린 뒤, 0.5초 동안 세 상(U · V · W, 모터 코일로 들어가는 세 갈래 전류)의 센서 출력을 "
             "평균 내 EEPROM 에 영점으로 저장한다. 센서 출력은 ADC(전압을 숫자로 바꾸는 회로)가 낸 0~4095 사이 숫자다. "
             "전류를 끊은 약 1.5초 동안은 힘이 빠져 PITCH 가 조금 처질 수 있다. 4단계 표에서 모터 구동이 꺼짐이던 축은 "
             "0.5초 동안 전류를 더 읽어 0 A 에 가까운지(남은 전류) 본다. 구동이 켜진 축은 모터가 힘을 쓰느라 전류가 흘러서 "
             "이 확인을 건너뛴다.")
    v.why = "영점이 틀어지면 모터가 실제와 다른 전류를 믿고 제어해서 떨리거나 힘이 고르지 않게 된다."
    v.crit = [("PASS", f"U · V · W 영점이 모두 {lo:g}~{hi:g}(ADC 범위의 가운데쯤)이고, 구동이 꺼진 축은 남은 전류가 {res:g} A 이하다."),
              ("WARN", f"남은 전류가 {res:g} A 를 넘거나, 전류를 물었는데 응답이 없다."),
              ("FAIL", "영점이 범위 밖이거나 7초 안에 완료 보고가 없다. 축을 움직이는 검사는 건너뛴다."),
              ("SKIP", "QC 탭에서 '전류 영점 보정'을 끄면 건너뛰고, EEPROM 에 있던 영점을 그대로 쓴다.")]
    if "읽기만" in d.get("영점.방식", ""):
        # 로봇 기본 점검(레벨 1) — 다시 재지 않고 저장된 영점만 본다(qc/03 §2)
        v.what = "모터 전류 센서의 영점(전류가 0 A 일 때 센서가 내는 값)이 저장된 그대로 정상 범위인지 본다. 기본 점검에서는 다시 재지 않는다."
        v.how = ("5단계가 읽은 설정값 가운데 세 상(U · V · W, 모터 코일로 들어가는 세 갈래 전류)의 영점이 범위 안인지 본다. "
                 "영점은 ADC(전압을 숫자로 바꾸는 회로)가 낸 0~4095 사이 숫자다. 4단계 표에서 모터 구동이 꺼짐이던 축은 0.5초 "
                 "동안 전류를 읽어 0 A 에 가까운지(남은 전류)도 본다.")
        v.crit = [("PASS", f"저장된 U · V · W 영점이 모두 {lo:g}~{hi:g}(ADC 범위의 가운데쯤)이고, 구동이 꺼진 축은 남은 전류가 "
                           f"{res:g} A 이하다."),
                  ("WARN", f"남은 전류가 {res:g} A 를 넘거나, 전류를 물었는데 응답이 없다."),
                  ("FAIL", "저장된 영점이 범위 밖이거나 5단계에서 영점 칸을 못 읽었다. 축을 움직이는 검사는 건너뛴다.")]
    t = Table(["축", "영점 U", "영점 V", "영점 W", "남은 전류"], num=(1, 2, 3, 4))
    for ax in AXES:
        uvw = d.get(f"{ax}.영점 U/V/W")
        if not uvw:
            continue
        parts = [num(x) for x in uvw.split("/")]
        cells = [(fx(p, 0), "bad") if p is None or not (lo <= p <= hi) else fx(p, 0) for p in (parts + [None] * 3)[:3]]
        r = num(d.get(f"{ax}.잔류전류[A]"))
        if r is not None:
            rc = (fx(r, 3, " A"), "warn" if r > res else "")
        elif "구동 ON" in s["measured"] or "구동 중" in s["measured"]:
            rc = ("구동이 켜져 있어 재지 않음", "muted")
        else:
            rc = ("—", "muted")
        t.rows.append([ax, *cells, rc])
    if t.rows:
        v.tables.append(t)
    vals = [num(x) for ax in AXES for x in d.get(f"{ax}.영점 U/V/W", "").split("/") if num(x) is not None]
    if s["verdict"] in ("PASS", "WARN") and vals:
        res_t = ", ".join(f"{ax} {fx(num(d.get(f'{ax}.잔류전류[A]')), 3, ' A')}" for ax in AXES
                          if num(d.get(f"{ax}.잔류전류[A]")) is not None)
        driving = "구동 ON" in s["measured"] or "구동 중" in s["measured"]
        tail = ("모터 구동이 켜져 있어 남은 전류는 재지 않음" if driving
                else (f"남은 전류 {res_t} (기준 {res:g} A 이하)" if res_t else ""))
        v.line = f"전류 영점 {min(vals):.0f}~{max(vals):.0f} (허용 {lo:g}~{hi:g})" + (f" · {tail}" if tail else "")
    else:
        v.line = plain(s["measured"])


def _s7(v: StepView, s: dict, d: dict, L: dict):
    v.what = "두 축을 양쪽 끝의 멈춤턱(하드스톱)까지 움직여 원점(0°)을 잡는다. 이 일을 홈잉이라 한다."
    v.how = ("축을 한쪽 끝까지 밀어 멈추면 거기서 모터 엔코더(모터가 돈 양을 펄스로 세는 센서)를 0 으로 놓고, "
             "반대쪽 끝까지 가며 펄스를 센다. 이렇게 잰 끝에서 끝까지 거리(펄스)가 최소 기준보다 커야 모터 펌웨어가 "
             "성공으로 답한다. 펌웨어는 이 펄스 수로 1펄스가 몇 도인지 정한다. 끝에서 끝까지를 YAW 345°, PITCH 114° 로 본다. "
             "각도 0° 는 YAW 는 두 끝의 한가운데, PITCH 는 −24° 쪽 끝에서 24° 떨어진 자리다.")
    v.why = ("원점이 없으면 각도 명령이 뜻이 없다. 끝에서 끝까지 거리가 설계보다 짧으면 기구가 어딘가 걸려 끝까지 못 간 것이고, "
             "그대로 두면 이후 모든 각도가 틀어진다.")
    v.crit = [("PASS", "두 축 모두 홈잉에 성공한다."),
              ("FAIL", "홈잉에 실패했거나, 드라이버 폴트 때문에 홈잉 명령이 거부됐거나, 80초 안에 안 끝났다. "
                       "축을 움직이는 검사는 건너뛴다."),
              ("기록만", "끝에서 끝까지 거리와, 그것이 최소 기준의 몇 배인지. 최소 기준은 모터 설정 5번(Home Zero TH)이고, "
                        "기본값은 설계상 끝에서 끝까지 거리의 95%다.")]
    t = Table(["축", "끝에서 끝까지 거리", "최소 기준", "최소 기준 대비", "출력축 1° 당 펄스", "홈 명령 다시 보낸 횟수"],
              num=(1, 2, 3, 4, 5))
    for ax in AXES:
        st = num(get(d, f"{ax}.홈 스트로크[count]", f"{ax}.홈 끝~끝 거리[펄스]", f"{ax}.홈 끝~끝[펄스]", default=None))
        th = num(get(d, f"{ax}.홈 임계[count]", f"{ax}.홈 최소 기준[펄스]", default=None))
        if st is None and th is None:
            continue
        ratio = (st / th) if st and th else None
        again = d.get(f"{ax}.HOME 다시 보냄")
        per = (st / STROKE_DEG[ax]) if st else None
        t.rows.append([ax, f"{comma(st)} 펄스", f"{comma(th)} 펄스",
                       ("—" if ratio is None else (f"{ratio:.2f}배", "bad" if ratio <= 1 else "")),
                       "—" if per is None else comma(per), f"{again}번" if again else "0번"])
    if t.rows:
        t.note = ("출력축 1° 당 펄스는 끝에서 끝까지 잰 펄스를 YAW 345°, PITCH 114° 로 나눈 값이다. 엔코더가 감속기(톱니바퀴로 "
                  "속도를 줄이고 힘을 키우는 장치) 앞 모터축에 있어서, 출력축(카메라를 실제로 돌리는 축)이 1° 돌 때 수백~천 "
                  "펄스가 나온다. 축이 홈잉을 시작하지 않으면 홈 명령을 다시 보낸다.")
        v.tables.append(t)
    mf = re.search(r"(YAW|PITCH)\s*(?:HOME_FAIL\(5\)|홈잉 실패[^—]*)\s*—\s*(?:스트로크|끝~끝(?: 거리)?)\s*(\d+)(?:\s*펄스)?\s*·\s*"
                   r"(?:임계|최소 기준)\s*(\d+)", s["measured"] or "")
    if mf and not t.rows:
        st, th = float(mf.group(2)), float(mf.group(3))
        t.rows.append([mf.group(1), f"{comma(st)} 펄스", f"{comma(th)} 펄스", f"{st / th:.2f}배",
                       comma(st / STROKE_DEG[mf.group(1)]), "—"])
        t.note = ("홈잉에 실패한 축의 값이다. 펌웨어는 끝에서 끝까지 거리가 모자랄 때 말고도, 제한 시간을 넘기거나 "
                  "헛도는 것(커플링 미끄러짐)을 감지했을 때 실패로 답한다.")
        v.tables.append(t)
    if mf:
        st, th = float(mf.group(2)), float(mf.group(3))
        v.line = (f"{mf.group(1)} 홈잉 실패 — 끝에서 끝까지 {comma(st)} 펄스로 최소 기준 {comma(th)} 펄스는 넘었지만 펌웨어가 "
                  "실패로 답함" if st > th else
                  f"{mf.group(1)} 홈잉 실패 — 끝에서 끝까지 {comma(st)} 펄스가 최소 기준 {comma(th)} 펄스에 못 미침")
        return
    rows = [(ax, num(get(d, f"{ax}.홈 스트로크[count]", f"{ax}.홈 끝~끝 거리[펄스]", default=None)),
             num(get(d, f"{ax}.홈 임계[count]", f"{ax}.홈 최소 기준[펄스]", default=None))) for ax in AXES]
    if s["verdict"] == "PASS" and all(st and th for _, st, th in rows):
        v.line = "끝에서 끝까지 " + ", ".join(
            f"{ax} {comma(st)} 펄스({'최소 기준의 ' if i == 0 else ''}{st / th:.2f}배)" for i, (ax, st, th) in enumerate(rows))
    else:
        v.line = plain(s["measured"])


def _s8(v: StepView, s: dict, d: dict, L: dict):
    p, w = L["hold_pass"], L["hold_warn"]
    v.what = "홈잉을 마친 두 축이 제자리에 가만히 서 있는지(흔들림)와, 서 있는 데 전류가 얼마나 드는지(유지전류)를 잰다."
    v.how = ("지금 자리를 목표로 잡고 1초 기다린 다음, 3초 동안 두 축의 위치(엔코더로 잰 각도)와 전류를 계속 읽는다. "
             "위치가 가장 클 때와 가장 작을 때의 차이가 흔들림 폭이고, 그동안 전류의 평균이 유지전류다.")
    v.why = ("멈춰 있어야 할 때 축이 떨면 카메라 화면이 흔들린다. 유지전류는 PITCH 가 카메라 무게를 받쳐야 해서 흐르는 게 "
             "정상이고, 무게를 받치지 않는 YAW 는 0 에 가깝다. 같은 모델끼리 견줘 평소보다 크면 무게 중심이 틀어졌거나 어딘가 "
             "끼어 모터가 버티고 있다는 신호다.")
    v.crit = [("PASS", f"두 축 흔들림 폭이 {p:g}° 이하다."),
              ("WARN", f"흔들림 폭이 {p:g}° 를 넘고 {w:g}° 이하다."),
              ("FAIL", f"흔들림 폭이 {w:g}° 를 넘는다."),
              ("기록만", "유지전류.")]
    t = Table(["축", "흔들림 폭 (3초 동안)", "판정", "유지전류 (3초 평균)"], num=(1, 3))
    for ax in AXES:
        pp = num(get(d, f"{ax}.정지 p-p[deg]", f"{ax}.정지 흔들림 폭[deg]", f"{ax}.흔들림 폭[deg]", default=None))
        hi = num(d.get(f"{ax}.유지전류[A]"))
        if pp is None and hi is None:
            continue
        g = grade(pp, p, w)
        t.rows.append([ax, fx(pp, 3, "°"), cell_v(g) if g else "—", fx(hi, 2, " A")])
    if t.rows:
        v.tables.append(t)
    vals = [(ax, num(get(d, f"{ax}.정지 p-p[deg]", f"{ax}.정지 흔들림 폭[deg]", default=None)),
             num(d.get(f"{ax}.유지전류[A]"))) for ax in AXES]
    if all(pp is not None for _, pp, _ in vals):
        v.line = ("흔들림 폭 " + ", ".join(f"{ax} {fx(pp, 3, '°')}" for ax, pp, _ in vals)
                  + " · 유지전류 " + ", ".join(f"{ax} {fx(hi, 2, ' A')}" for ax, _, hi in vals))
    else:
        v.line = plain(s["measured"])


def _move_table(v: StepView, d: dict, L: dict, mv: list, limit: bool) -> list[dict]:
    """9·10 이동 표를 v 에 붙이고, 이동마다 평가를 돌려준다(한 줄 결과용)."""
    ratio = L["cur_ratio"]
    cur_h = ("이동 중 최대 전류 (괄호는 YAW 의 Cur MAX 대비 %)" if L.get("pitch_abs") else "이동 중 최대 전류 (Cur MAX 대비 %)")
    if limit:
        cols = ["이동", "명령", "서야 할 자리", "멈춘 위치", "오차", "도착 시간", cur_h, "판정"]
    else:
        cols = ["이동", "목표", "멈춘 위치", "오차", "목표를 넘어간 정도", "도착 시간", cur_h, "판정"]
    t = Table(cols, num=(1, 2, 3, 4, 5, 6))
    out = []
    for m in mv:
        ax = m["axis"]
        what = (f"{ax} {'+' if m['cmd'] > 0 else '−'} 쪽 끝" if limit else f"{ax} → {fx(m['cmd'], 0, '°')}")
        if "fail" in m:
            row = [what, fx(m["cmd"], 1, "°")] + ([""] if limit else [])
            row += [(plain(m["fail"]), "bad")] + [""] * (len(cols) - len(row) - 2) + [cell_v("FAIL")]
            t.rows.append(row)
            out.append({"g": "FAIL", "m": m, "what": what})
            continue
        cm = cur_max(d, ax)
        if ax == "PITCH" and L.get("pitch_abs"):
            # PITCH — Cur MAX 와 상관없이 고정 문턱(10-08 결정). 비율은 적지 않는다
            pct = None
            cur_bad = m["i"] >= L["pitch_cur"] - 1e-9
            icell = (fx(m["i"], 2, " A"), "warn" if cur_bad else "")
        else:
            pct = 100.0 * m["i"] / cm if cm else None
            cur_bad = pct is not None and pct >= ratio * 100 - 1e-9
            icell = (f"{fx(m['i'], 2, ' A')} ({pct:.0f}%)" if pct is not None else fx(m["i"], 2, " A"), "warn" if cur_bad else "")
        tcell = "—" if m["t"] is None else fx(m["t"], 1, " s")
        if limit and abs(m["cmd"]) < 1e-9:
            t.rows.append([f"{ax} 0° 로 돌아옴", "0.0°", "0.0°", fx(m["pos"], 2, "°"), fx(m["err"], 2, "°"), tcell, icell,
                           ("도착만 봄", "muted")])
            continue
        e = {"m": m, "what": what, "pct": pct, "cur_bad": cur_bad}
        if limit:
            off = num(d.get(f"{ax}.CFG12 Pos Offset")) or 0.0
            if ax == "YAW":
                e["exp"] = (L["yaw_lim"] if m["cmd"] > 0 else -L["yaw_lim"]) - off
            else:
                e["exp"] = (L["pitch_max"] if m["cmd"] > 0 else L["pitch_min"]) - off
            e["ge"] = grade(m["err"], L["lim_pass"], L["lim_warn"])
            e["g"] = worse(e["ge"], "WARN") if cur_bad else e["ge"]
            t.rows.append([what, fx(m["cmd"], 1, "°"), fx(e["exp"], 1, "°"), fx(m["pos"], 2, "°"),
                           (fx(m["err"], 2, "°"), {"WARN": "warn", "FAIL": "bad"}.get(e["ge"], "")), tcell, icell, cell_v(e["g"])])
        else:
            e["ge"] = grade(m["err"], L["err_pass"], L["err_warn"])
            e["go"] = grade(m["os"], L["os_pass"], L["os_warn"])
            e["g"] = worse(e["ge"], e["go"])
            if cur_bad:
                e["g"] = worse(e["g"], "WARN")
            t.rows.append([what, fx(m["cmd"], 1, "°"), fx(m["pos"], 2, "°"),
                           (fx(m["err"], 2, "°"), {"WARN": "warn", "FAIL": "bad"}.get(e["ge"], "")),
                           (fx(m["os"], 2, "°"), {"WARN": "warn", "FAIL": "bad"}.get(e["go"], "")), tcell, icell, cell_v(e["g"])])
        out.append(e)
    # 9단계는 어떻게 문장이 칸 뜻을 다 푼다. 10단계는 도착 시간의 기준만 다르다
    t.note = "도착 시간은 명령부터 서야 할 자리 0.5° 안에 처음 들어올 때까지다. 나머지 칸은 9단계 표와 같은 뜻이다." if limit else ""
    if t.rows:
        v.tables.append(t)
    return out


def _bad_moves(ev: list[dict], L: dict, limit: bool) -> str:
    """PASS 가 아닌 이동만 짧게 — '… 이동: 무엇이 얼마'."""
    r = L["cur_ratio"] * 100
    parts = []
    for e in ev:
        if e["g"] == "PASS":
            continue
        m = e["m"]
        if "fail" in m:
            parts.append(f"{e['what']} {plain(m['fail'])}")
            continue

        def line(g, ok, bad):              # WARN 이면 PASS 선을, FAIL 이면 WARN 선을 붙인다
            return f"({ok:g}° 넘으면 WARN)" if g == "WARN" else f"({bad:g}° 넘으면 FAIL)"
        why = []
        if e.get("ge") not in ("PASS", ""):
            if limit:
                why.append(f"{fx(m['pos'], 2, '°')} 에 멈춤 — 서야 할 {fx(e['exp'], 1, '°')} 에서 {fx(m['err'], 2, '°')} 벗어남"
                           + line(e["ge"], L["lim_pass"], L["lim_warn"]))
            else:
                why.append(f"오차 {fx(m['err'], 2, '°')}" + line(e["ge"], L["err_pass"], L["err_warn"]))
        if e.get("go") not in (None, "PASS", ""):
            why.append(f"목표를 넘어간 정도 {fx(m['os'], 2, '°')}" + line(e["go"], L["os_pass"], L["os_warn"]))
        if e["cur_bad"] and e["pct"] is None:
            why.append(f"이동 중 최대 전류 {fx(m['i'], 2, ' A')} — {L['pitch_cur']:g} A 부터 WARN(걸림 의심)")
        elif e["cur_bad"]:
            why.append(f"이동 중 최대 전류 {fx(m['i'], 2, ' A')}, Cur MAX 의 {e['pct']:.0f}% — {r:.0f}% 부터 WARN(걸림 의심)")
        parts.append(f"{e['what']}: " + ", ".join(why))
    return " · ".join(parts)


def cur_rule(L: dict) -> tuple[str, str]:
    """9 · 10 전류 기준 글 — (PASS 쪽, WARN 쪽). 10-08 부터 PITCH 는 고정 문턱, 그 전 검사는 두 축 다 Cur MAX 비율."""
    r = L["cur_ratio"] * 100
    if L.get("pitch_abs"):
        return (f"YAW 에서 Cur MAX 의 {r:.0f}% 미만, PITCH 에서 {L['pitch_cur']:g} A 미만이다",
                f"YAW 에서 Cur MAX 의 {r:.0f}% 이상이거나 PITCH 에서 {L['pitch_cur']:g} A 이상이다")
    return f"Cur MAX 의 {r:.0f}% 미만이다", f"Cur MAX 의 {r:.0f}% 이상이다"


def cur_peak_text(ok: list[dict], L: dict) -> str:
    """PASS 한 줄의 전류 꼬리 — 축마다 가장 큰 이동 전류. PITCH 는 고정 문턱, YAW 는 Cur MAX 대비."""
    out = []
    for ax in AXES:
        es = [e for e in ok if e["m"]["axis"] == ax]
        if not es:
            continue
        top = max(es, key=lambda e: e["m"]["i"])
        if top["pct"] is None and ax == "PITCH" and L.get("pitch_abs"):
            out.append(f"PITCH {fx(top['m']['i'], 2, ' A')} (기준 {L['pitch_cur']:g} A 미만)")
        elif top["pct"] is not None:
            out.append(f"{ax} {fx(top['m']['i'], 2, ' A')} (Cur MAX 의 {top['pct']:.0f}%)")
        else:
            out.append(f"{ax} {fx(top['m']['i'], 2, ' A')}")
    return (" · 이동 중 최대 전류 " + ", ".join(out)) if out else ""


def _s9(v: StepView, s: dict, d: dict, L: dict, details):
    r = L["cur_ratio"] * 100
    v.what = "축을 정해진 각도로 보내 정확히, 지나치지 않고, 무리 없이 가는지 본다."
    v.how = ("한 번에 한 축씩, YAW 는 −90°, +90°, 0°, PITCH 는 +60°, −15°, 0° 순서로 보낸다. 가는 동안 목표를 지나친 "
             "가장 큰 양(넘어간 정도, 오버슈트)과 가장 큰 전류를 기록하고, 명령을 보낸 때부터 목표 0.5° 안에 처음 들어올 "
             "때까지를 도착 시간으로 적는다. 도착 시간은 판정에 쓰지 않는다. 위치가 0.3초 동안 0.03° 폭 안에 머물면 멈춘 "
             "것으로 보고 그때 각도를 멈춘 위치로 적는데, 이 각도는 모터 엔코더 펄스로 계산한 값이다(출력축에는 따로 센서가 "
             "없다). 멈춘 위치와 목표의 차이가 오차다.")
    cur_ok, cur_bad = cur_rule(L)
    v.why = ("명령한 곳에 정확히 서야 카메라가 원하는 곳을 본다. 이동 중 전류가 평소보다 크게 치솟으면 "
             "기구가 뻑뻑하거나 어딘가 걸린 것이다.")
    v.crit = [("PASS", f"여섯 번 모두 오차 {L['err_pass']:g}° 이하, 넘어간 정도 {L['os_pass']:g}° 이하이고, "
                       f"이동 중 최대 전류가 {cur_ok}."),
              ("WARN", f"오차가 {L['err_pass']:g}° 를 넘고 {L['err_warn']:g}° 이하이거나, 넘어간 정도가 {L['os_pass']:g}° 를 넘고 "
                       f"{L['os_warn']:g}° 이하이거나, 이동 중 최대 전류가 {cur_bad}(걸림 의심)."),
              ("FAIL", f"오차가 {L['err_warn']:g}° 를 넘거나 넘어간 정도가 {L['os_warn']:g}° 를 넘는다. 제한 시간(최고 속도로 "
                       "가는 데 걸릴 시간의 두 배에 4초를 더한 것) 안에 못 멈췄거나, 위치 신호(모터 보드가 초당 100번 보내는 "
                       "현재 각도)가 0.5초 넘게 안 와도 FAIL 이다."),
              ("", "단계 판정은 이동 여섯 번 가운데 가장 나쁜 것을 따른다.")]
    ev = _move_table(v, d, L, moves(details, 9), limit=False)
    ok = [e for e in ev if "fail" not in e["m"]]
    if not ev:
        v.line = plain(s["measured"])
    elif all(e["g"] == "PASS" for e in ev):
        v.line = (f"오차 최대 {fx(max(e['m']['err'] for e in ok), 2, '°')} · 목표를 넘어간 정도 최대 "
                  f"{fx(max(e['m']['os'] for e in ok), 2, '°')}" + cur_peak_text(ok, L))
    else:
        v.line = _bad_moves(ev, L, limit=False)


def _s10(v: StepView, s: dict, d: dict, L: dict, details):
    r = L["cur_ratio"] * 100
    yl, pmax, pmin = L["yaw_lim"], L["pitch_max"], L["pitch_min"]
    mv = moves(details, 10)
    cmds = {(m["axis"], m["cmd"] > 0): m["cmd"] for m in mv if abs(m["cmd"]) > 1e-9}
    ycmd = fx(abs(cmds.get(("YAW", True), 172.5)), 1, "°")
    pcmd = f"+{fx(cmds.get(('PITCH', True), 90.0), 0, '°')} 와 {fx(cmds.get(('PITCH', False), -24.0), 0, '°')}"
    v.what = ("기구 끝을 넘는 각도를 명령해도 모터 펌웨어가 끝에 닿기 전에 막아 세우는지 본다. 펌웨어가 정해 둔 이 움직임 "
              "한계를 소프트 리밋이라 한다.")
    v.how = (f"기구 끝(멈춤턱) 자리인 YAW ±{ycmd}, PITCH {pcmd} 로 가라고 명령한다. 펌웨어는 끝에서 4° 안쪽인 "
             f"YAW ±{yl:g}°, PITCH +{pmax:g}° 와 {fx(pmin, 0, '°')} 에서 막아야 하고, 원점 보정(Pos Offset)이 있으면 그만큼 "
             "옮긴 자리에서 막는다. 이 자리를 '서야 할 자리'로 두고, 축이 멈춘 위치가 여기서 얼마나 떨어졌는지 잰다. "
             "끝 하나를 밀 때마다 0° 로 돌아온다.")
    v.why = "사람이 실수로 끝을 넘는 각도를 보내도 멈춤턱에 부딪치지 않아야 기구와 케이블이 상하지 않는다."
    cur_ok, cur_bad = cur_rule(L)
    v.crit = [("PASS", f"두 축의 양쪽 끝 모두 멈춘 위치가 서야 할 자리에서 ±{L['lim_pass']:g}° 안이고, "
                       f"이동 중 최대 전류가 {cur_ok}."),
              ("WARN", f"멈춘 위치가 ±{L['lim_pass']:g}° 밖이지만 ±{L['lim_warn']:g}° 안이거나, "
                       f"이동 중 최대 전류가 {cur_bad}."),
              ("FAIL", f"멈춘 위치가 ±{L['lim_warn']:g}° 밖이거나, 제한 시간 안에 못 멈췄거나, 위치 신호가 0.5초 넘게 안 왔다."),
              ("", "0° 로 돌아오는 이동은 도착만 본다.")]
    ev = _move_table(v, d, L, mv, limit=True)
    ok = [e for e in ev if "fail" not in e["m"]]
    if not ev:
        v.line = plain(s["measured"])
    elif all(e["g"] == "PASS" for e in ev):
        by_ax = [f"{ax} " + ", ".join(fx(e['m']['pos'], 1, '°', sign=True) for e in ok if e['m']['axis'] == ax)
                 for ax in AXES if any(e['m']['axis'] == ax for e in ok)]
        v.line = (f"두 축의 양쪽 끝 모두 서야 할 자리에서 {fx(max(e['m']['err'] for e in ok), 2, '°')} 안에 멈춤 — 멈춘 곳 "
                  + " · ".join(by_ax))
    else:
        v.line = _bad_moves(ev, L, limit=True)


def _s11(v: StepView, s: dict, d: dict, L: dict):
    slot, tol = int(L["preset_slot"]), L["preset_tol"]
    v.what = "저장해 둔 자리로 돌아가는 기능(프리셋)과 모터 EEPROM 쓰기 · 읽기가 되는지 본다."
    v.how = (f"모터 프리셋 칸 {slot}번(검사 전용)이 비어 있는지 먼저 확인한다. 두 축을 YAW {PRESET_AT['YAW']:g}°, "
             f"PITCH {PRESET_AT['PITCH']:g}° 로 보내 그 자리를 {slot}번에 저장하고(모터 EEPROM 에 쓴다), 두 축을 0° 로 옮긴 다음 "
             f"'{slot}번으로 가라'고 명령한다. 모터가 스스로 돌아온 자리와 저장한 자리의 차이를 재고, 끝나면 {slot}번을 지운다.")
    v.why = "저장한 자리로 정확히 돌아가야 프리셋을 믿고 쓸 수 있다. EEPROM 에 제대로 못 쓰면 전원을 껐다 켰을 때 사라진다."
    v.crit = [("PASS", f"두 축 모두 돌아온 자리 오차가 {tol:g}° 이하이고, 저장 · 이동 · 삭제 응답이 모두 OK 다."),
              ("FAIL", f"오차가 {tol:g}° 를 넘거나, 저장 · 이동 · 삭제 명령에 1.5초 안에 응답이 없거나 OK 가 아닌 응답이 "
                       "왔거나, 이동이 제한 시간 안에 안 끝났다."),
              ("SKIP", f"{slot}번 칸에 이미 저장된 자리가 있으면 덮어쓰지 않고 건너뛴다. 로봇 점검에서는 프리셋 변경에 동의했으면 "
                       "그 자리를 지우고 검사하고, 동의하지 않았거나 기본 점검(레벨 1)이면 건너뛴다.")]
    if d.get("프리셋.254 칸"):
        v.notes.append(f"{slot}번 칸에 저장된 자리가 있었다 — 프리셋 변경 동의를 받아 지우고 검사했다.")
    raw = d.get("프리셋 복귀 오차 YAW/PITCH[deg]", "")
    errs = [num(x) for x in raw.split("/")] if raw else []
    if len(errs) == 2:
        t = Table(["축", "저장한 자리", "돌아온 자리 오차", "판정"], num=(1, 2))
        for ax, e in zip(AXES, errs):
            t.rows.append([ax, fx(PRESET_AT[ax], 1, "°"), fx(e, 3, "°"), cell_v("PASS" if e is not None and e <= tol else "FAIL")])
        v.tables.append(t)
        v.line = f"돌아온 자리 오차 YAW {fx(errs[0], 3, '°')}, PITCH {fx(errs[1], 3, '°')} (기준 {tol:g}° 이하)"
    else:
        v.line = plain(s["measured"])


def _s12(v: StepView, s: dict, d: dict, L: dict):
    v.what = "줌 카메라와 열화상 카메라 영상이 들어오기만 하는 게 아니라 제대로 보이는지 본다."
    v.how = (f"카메라마다 영상(RTSP, 네트워크 카메라가 영상을 보내는 방식)을 받아 {L['cam_first_s']:g}초 안에 "
             f"프레임 {L['cam_frames']:g}장이 오는지 보고, "
             f"첫 프레임부터 {L['cam_watch_s']:g}초 동안 프레임마다 밝기 평균, 밝기가 얼마나 고르지 않은지(σ, 표준편차), "
             "바로 앞 프레임과의 밝기 차이를 잰다. 열화상은 왼쪽 위 온도 표시(OSD) 칸의 σ 도 잰다. 밝기는 0(검정)~255(흰색)이다.")
    v.why = ("렌즈캡을 씌운 검은 화면, 센서가 죽은 한 가지 색 화면, 멈춘 화면도 프레임은 계속 온다. "
             "σ 는 화면에 무늬가 있으면 크고 한 가지 색이면 0 에 가깝다.")
    v.crit = [("PASS", "두 카메라 모두 아래 FAIL · WARN 에 걸리지 않는다."),
              ("WARN", f"열화상 온도 표시 칸의 σ 가 {L['cam_osd_std']:g} 보다 작다(온도 글자가 안 보임)."),
              ("FAIL", f"{L['cam_first_s']:g}초 안에 프레임 {L['cam_frames']:g}장을 못 받았거나, 화면이 검거나(밝기 평균 "
                       f"{L['cam_dark_mean']:g} 미만이고 σ {L['cam_dark_std']:g} 미만), 한 가지 색이거나(σ {L['cam_flat_std']:g} 미만), "
                       f"멈췄다(지켜본 {L['cam_watch_s']:g}초 내내 연속 프레임 밝기 차이 {L['cam_freeze']:g} 이하)."),
              ("SKIP", "QC 탭에서 '카메라·소리 검사'를 끄거나 CONFIG 탭에 카메라 설정이 없을 때.")]
    cams = [c for c in ("줌", "열화상") if f"카메라.{c}" in d]
    if cams:
        t = Table(["항목", *cams, "기준"], num=tuple(range(1, len(cams) + 1)))

        def rowf(label, key, fmt, crit, bad=lambda x: False):
            cells = []
            for c in cams:
                raw = d.get(f"카메라.{c}{key}")
                if raw is None:
                    cells.append(("—", "muted"))
                    continue
                val = num(raw)
                txt = fmt(raw, val)
                cells.append((txt, "bad") if val is not None and bad(val) else txt)
            t.rows.append([label, *cells, crit])
        # 첫 칸 '카메라.줌' = "줌 1920×1080 29.1fps" 또는 "프레임 없음"
        heads = []
        for c in cams:
            h = d.get(f"카메라.{c}", "")
            m = re.search(r"(\d+×\d+)(?:\s*([0-9.]+)fps)?", h)
            heads.append((f"{m.group(1)}" + (f" · {float(m.group(2)):.0f} fps" if m.group(2) else ""))
                         if m else (h.replace(c, "").strip() or "—", "bad"))
        t.rows.append(["해상도 · 초당 프레임", *heads, ""])
        rowf("첫 프레임이 온 시간", ".첫 프레임[s]", lambda r, x: fx(x, 2, " s"),
             f"{L['cam_frames']:g}장이 {L['cam_first_s']:g}초 안에 와야 함")
        rowf(f"{L['cam_watch_s']:g}초 동안 받은 프레임", ".본 프레임", lambda r, x: f"{x:.0f}장" if x is not None else r, "",
             bad=lambda x: x < L["cam_frames"])
        rowf("밝기 평균", ".밝기 평균", lambda r, x: fx(x, 1), f"검은 화면: < {L['cam_dark_mean']:g}")
        rowf("밝기 σ (가장 큰 값)", ".밝기 표준편차(최대)", lambda r, x: fx(x, 1),
             f"검은 화면: < {L['cam_dark_std']:g} · 한 가지 색: < {L['cam_flat_std']:g}", bad=lambda x: x < L["cam_flat_std"])
        rowf("연속 프레임 밝기 차 (가장 큰 값)", ".연속 프레임 차(최대)", lambda r, x: fx(x, 2) if x is not None else r,
             f"멈춘 화면: ≤ {L['cam_freeze']:g}", bad=lambda x: x <= L["cam_freeze"])
        if "열화상" in cams:
            cells = []
            for c in cams:
                raw = d.get(f"카메라.{c}.OSD 칸 표준편차(최대)")
                val = num(raw)
                cells.append(("—", "muted") if raw is None else
                             ((fx(val, 1), "warn") if val is not None and val < L["cam_osd_std"] else fx(val, 1)))
            t.rows.append(["온도 표시 칸 σ", *cells, f"비었음: < {L['cam_osd_std']:g}"])
        issues = [d.get(f"카메라.{c}.판정") for c in cams]
        if any(issues):
            t.rows.append(["문제", *[(x.replace("OSD 온도 칸 비어 있음", "온도 표시 칸 비었음"), "bad") if x else "없음"
                                     for x in issues], ""])

        def host(u: str) -> str:                   # rtsp://사용자:*****@주소:포트/경로 → 주소. 시험 파이프라인은 그대로
            m = re.match(r"rtsp://(?:[^@/]*@)?([^:/]+)", u or "")
            return m.group(1) if m else (u or "—")
        t.note = "카메라 주소 — " + " · ".join(f"{c} {host(d.get(f'카메라.{c}.주소', ''))}" for c in cams)
        v.tables.append(t)
    if s["verdict"] == "PASS" and cams:
        parts = []
        for c in cams:
            m = re.search(r"(\d+×\d+)(?:\s*([0-9.]+)fps)?", d.get(f"카메라.{c}", ""))
            parts.append(f"{c} {m.group(1)}" + (f" {float(m.group(2)):.0f} fps" if m and m.group(2) else "") if m else c)
        v.line = ", ".join(parts) + " — 검은 화면도, 한 가지 색 화면도, 멈춘 화면도 아님"
    elif cams and s["verdict"] in ("WARN", "FAIL"):
        bad = []
        for c in cams:
            issue = d.get(f"카메라.{c}.판정") or ("" if re.search(r"\d+×\d+", d.get(f"카메라.{c}", "")) else d.get(f"카메라.{c}", ""))
            if issue:
                bad.append(f"{c} " + ", ".join(_cam_issue(c, x.strip(), d, L) for x in issue.replace(c, "").split(",") if x.strip()))
        v.line = " · ".join(bad) if bad else plain(s["measured"])
    else:
        v.line = plain(s["measured"])


def _cam_issue(c: str, x: str, d: dict, L: dict) -> str:
    """12 WARN · FAIL 까닭(QC 탭 글 qctab.cpp camJudge) 하나 → 값과 기준을 붙인 말."""
    k = f"카메라.{c}."
    if x.startswith("OSD 온도 칸 비어"):
        return f"온도 표시 칸 비었음(σ {fx(num(d.get(k + 'OSD 칸 표준편차(최대)')), 1)} — {L['cam_osd_std']:g} 미만)"
    if x == "프레임 없음" or x.startswith("프레임 ") and x.endswith("장뿐"):
        return f"{L['cam_first_s']:g}초 안에 {x}({L['cam_frames']:g}장 받아야 함)"
    if x == "검은 화면":
        return f"검은 화면(밝기 {fx(num(d.get(k + '밝기 평균')), 1)}, σ {fx(num(d.get(k + '밝기 표준편차(최대)')), 1)})"
    if x == "한 가지 색 화면":
        return f"한 가지 색 화면(σ {fx(num(d.get(k + '밝기 표준편차(최대)')), 1)})"
    return x


def _s13(v: StepView, s: dict, d: dict, L: dict):
    p, f = L["mic_pass"], L["mic_fail"]
    v.what = "카메라 스피커에서 소리가 나고 카메라 마이크가 그 소리를 듣는지 본다."
    v.how = ("카메라 스피커로 1000 Hz 와 2000 Hz 삐 소리를 2초씩 차례로 틀고, 같은 카메라 마이크의 녹음에서 그 주파수 성분만 골라 "
             "틀기 전(배경)보다 몇 dB 커졌는지 잰다. 20 dB 는 소리 진폭으로 10배, 10 dB 는 약 3배다.")
    v.why = ("공장은 시끄러워서 전체 소리 크기로 보면 옆 라인 소음에도 통과해 버린다. 보낸 주파수만 보면 소음은 거의 걸러진다. "
             "두 소리를 트는 건 한 대역만 죽은 스피커나 마이크도 잡으려는 것이다.")
    v.crit = [("PASS", f"두 소리 모두 배경보다 {p:g} dB 이상 커진다."),
              ("WARN", f"약한 쪽이 {f:g} dB 이상 {p:g} dB 미만 커졌다."),
              ("FAIL", f"한 소리라도 {f:g} dB 미만이거나, 녹음 · 재생 연결이 안 된다."),
              ("SKIP", "QC 탭에서 '카메라·소리 검사'를 끄거나 줌 카메라 주소·암호가 없을 때.")]
    t = Table(["소리", "틀기 전 (그 주파수)", "틀었을 때", "커진 만큼", "판정"], num=(1, 2, 3))
    for hz in (1000, 2000):
        raw = d.get(f"스피커→마이크.{hz} Hz 배경/톤/상승[dB]")
        if not raw:
            continue
        parts = [num(x) for x in raw.split("(")[0].split("/")]
        if len(parts) < 3:
            continue
        bg, tone, rise = parts[:3]
        g = "PASS" if rise is not None and rise >= p else "WARN" if rise is not None and rise >= f else "FAIL"
        t.rows.append([f"{hz} Hz", fx(bg, 1, " dBFS"), fx(tone, 1, " dBFS"),
                       (fx(rise, 1, " dB", sign=True), {"WARN": "warn", "FAIL": "bad"}.get(g, "")), cell_v(g)])
    tone, ref = "", []
    if d.get("스피커→마이크.톤"):              # QC 탭 글 '1000 Hz · 2000 Hz, 각 2.0 s, 크기 70% (진폭 4029/32767)'
        tone = " 튼 소리 — " + re.sub(r"\s*\(진폭 [^)]*\)", "", d["스피커→마이크.톤"]).replace("크기 ", "크기 설정 ") + "."
    if d.get("스피커→마이크.배경 레벨[dBFS]"):
        ref.append(f"배경 소음 전체 크기 {fx(num(d['스피커→마이크.배경 레벨[dBFS]']), 1, ' dBFS')}")
    if d.get("스피커→마이크.지연 추정[ms]"):
        lat = d["스피커→마이크.지연 추정[ms]"]
        ref.append(f"스피커에서 마이크까지 걸린 시간(추정) {lat} ms" if num(lat) is not None else
                   "스피커에서 마이크까지 걸린 시간 — " + lat.replace("말하기 시작 표시 기준으로 잼", "소리를 보내기 시작한 때부터 잰 값"))
    if t.rows:
        t.note = ("dBFS 는 녹음이 담을 수 있는 가장 큰 소리를 0 으로 둔 dB 다(음수가 클수록 작은 소리)." + tone
                  + (" 참고 — " + ", ".join(ref) + "." if ref else ""))
        v.tables.append(t)
    if d.get("스피커→마이크.참고"):               # QC 탭 글(qcaudiotest.cpp judge) → 사람 말
        hint = d["스피커→마이크.참고"]
        mh = re.match(r"^(\d+ Hz) 이 약함 — 그 대역 스피커·마이크 의심$", hint)
        if mh:
            hint = f"{mh.group(1)} 쪽이 더 약하다. 그 주파수에서 스피커나 마이크가 약할 수 있다."
        elif hint.startswith("두 톤 모두 거의 안 오름"):
            hint = ("두 소리 모두 거의 안 커졌다. 스피커가 안 나거나 마이크가 나쁠 수 있고, 카메라의 에코 제거(AEC)가 "
                    "소리를 지웠을 수도 있다(실제 제품에서 확인해 봐야 한다).")
        v.notes.append("참고 — " + hint)
    if d.get("스피커→마이크.결과") and not t.rows:
        v.notes.append("결과 — " + d["스피커→마이크.결과"])
    rises = []
    for hz in (1000, 2000):
        raw = d.get(f"스피커→마이크.{hz} Hz 배경/톤/상승[dB]", "")
        m = re.search(r"/\s*([+-]?[0-9.]+)\s*\(", raw)
        if m:
            rises.append((hz, float(m.group(1))))
    if rises and s["verdict"] in ("PASS", "WARN", "FAIL"):
        lo = min(r for _, r in rises)
        hi = max(r for _, r in rises)
        v.line = ("배경보다 " + ", ".join(f"{hz} Hz {fx(r, 1, ' dB', sign=True)}" for hz, r in rises) + " 커짐"
                  + (f" — 둘 다 기준 {p:g} dB 이상" if lo >= p else
                     f" — 약한 쪽이 {p:g} dB 미만" if lo >= f else
                     f" — 둘 다 {f:g} dB 미만" if hi < f and len(rises) > 1 else f" — {f:g} dB 미만인 소리가 있음"))
    else:
        v.line = plain(s["measured"])


def _s14(v: StepView, s: dict, d: dict, L: dict):
    deg = L["rs485_deg"]
    v.what = ("카메라로 보낸 조종 명령이 카메라에서 RS-485 선(카메라와 게이트웨이를 잇는 통신선)을 지나 게이트웨이와 "
              "모터까지 이어져, 축이 실제로 움직이는지 본다.")
    v.how = (f"줌 카메라에 ONVIF(네트워크 카메라의 표준 명령)로 좌우(팬) 이동을 보내고, 0.5초마다 다시 보내며 YAW 가 {deg:g}° 움직일 때까지 "
             f"기다린다. {deg:g}° 움직이면 멈춤 명령을 보내고, 그때까지 걸린 시간을 적는다.")
    v.why = "PC 가 아니라 카메라를 거쳐 들어온 명령으로 PTZ 를 움직이는 길 전체, 특히 카메라와 게이트웨이 사이 RS-485 배선을 확인한다."
    v.crit = [("PASS", f"15초 안에 YAW 가 {deg:g}° 이상 움직인다."),
              ("FAIL", f"15초 동안 {deg:g}° 를 못 움직인다(카메라 연결 · RS-485 배선 · 게이트웨이를 볼 것)."),
              ("SKIP", "QC 탭에서 '카메라·소리 검사'를 끄거나 줌 카메라 주소·암호가 없을 때.")]
    msr = d.get("카메라 경유 구동") or s["measured"]
    m = re.search(r"YAW\s*(-?[0-9.]+)°\s*이동\s*·\s*반응\s*([0-9.]+)\s*s", msr or "")
    if m:
        t = Table(["움직인 YAW 각도", f"{deg:g}° 움직일 때까지 걸린 시간"], num=(0, 1))
        t.rows.append([fx(float(m.group(1)), 1, "°"), fx(float(m.group(2)), 1, " s")])
        v.tables.append(t)
        v.line = f"카메라 명령으로 YAW 가 {fx(float(m.group(1)), 1, '°')} 움직임 · {deg:g}° 움직이는 데 {fx(float(m.group(2)), 1, '초')}"
    else:
        v.line = plain(s["measured"])


def _s15(v: StepView, s: dict, d: dict, L: dict, details):
    v.what = "검사를 끝내기 전에 두 축을 0° 로 돌려놓는다."
    v.how = "YAW, PITCH 차례로 0° 로 보내고 멈출 때까지 기다린다."
    v.why = ("다음 사람이나 19단계 공회전(12시간 쉬지 않고 돌리는 시험)이 0° 에서 시작하게 한다. 오차는 앞의 9단계가 "
             "이미 봤으니 여기서는 도착만 본다.")
    v.crit = [("PASS", "두 축이 0° 로 돌아와 멈춘다."),
              ("FAIL", "제한 시간 안에 못 멈추거나 위치 신호가 0.5초 넘게 안 온다.")]
    mv = moves(details, 15)
    if mv:
        t = Table(["축", "멈춘 위치", "도착 시간"], num=(1, 2))
        for m in mv:
            if "fail" in m:
                t.rows.append([m["axis"], (plain(m["fail"]), "bad"), "—"])
            else:
                t.rows.append([m["axis"], fx(m["pos"], 2, "°"), "—" if m["t"] is None else fx(m["t"], 1, " s")])
        v.tables.append(t)
    if s["verdict"] == "PASS" and mv and all("fail" not in m for m in mv):
        v.line = "0° 로 돌아와 " + ", ".join(f"{m['axis']} {fx(m['pos'], 2, '°')}" for m in mv) + " 에 멈춤"
    else:
        v.line = plain(s["measured"])


FB_TOL = 5                                         # 16 위치 신호 손실 — 축마다 이만큼 봐준다(QC 탭 qcgwload 기준 '피드백 ±5장')
_STAGE = re.compile(
    r"추가\s*(?P<ach>[0-9.]+)/s\(계획\s*(?P<plan>[0-9.]+)\)\s*·\s*응답 손실\s*(?P<rl>[0-9.]+)%\s*\((?P<rr>\d+)/(?P<rs>\d+)\)\s*·\s*"
    r"(?:피드백|위치 신호) 손실\s*(?P<fl>[0-9.]+)%\s*\((?P<fr>\d+)/(?P<fe>\d+)\)\s*·\s*왕복 p50/p99/최대\s*(?P<p50>[0-9.]+|—)/(?P<p99>[0-9.]+|—)/"
    r"(?P<mx>[0-9.]+|—)\s*ms\s*\(n\s*(?P<n>\d+),\s*무응답\s*(?P<lost>\d+)\)\s*·\s*40 ms (?:튐|넘은 왕복)\s*(?P<sp>\d+)")


def _s16(v: StepView, s: dict, d: dict, L: dict):
    p99, lossless = L["gw_p99"], L["gw_lossless"]
    v.what = "요청이 몰려도 게이트웨이가 메시지를 빠뜨리지 않는지, 평소 부하에서 여유가 얼마나 되는지 본다."
    v.how = ("두 축을 세워 둔 채, 평소 오가는 통신에 더해 모터 전류를 묻는 요청을 보낸다. 더하는 요청은 초당 0번(L0), "
             "100번(L1, 평소 부하), 300번(L2), 500번(L3), 700번(L4)이고, 부하마다 3초씩 건다. 부하마다 요청에 돌아온 응답 수, "
             "모터가 초당 100번씩 보내는 위치 신호가 모자란 수, 확인 신호(CAN_CHECK)가 PC 에서 보드까지 갔다가 돌아오는 "
             "시간(왕복 시간)을 잰다. 왕복 시간은 p50(가운데 값)과 p99(100번 중 99번은 이보다 빠른 값)로 본다. "
             "1~15단계 동안에도 뒤에서 초당 4번쯤 확인 신호를 보내 연결 끊김과 왕복 시간을 적는다(검사 내내 감시).")
    v.why = ("게이트웨이는 모터에서 PC 쪽으로 초당 약 800개까지만 메시지를 보내고, 넘치는 것은 버린다. 버린 수를 "
             "게이트웨이가 세지 않아서 PC 쪽에서 센다.")
    v.crit = [("PASS", f"L0 · L1 에서 잃은 것이 없고, L1 왕복 p99 가 {p99:g} ms 이하이고, 손실 없이 버틴 최대 부하가 초당 "
                       f"{lossless:g}번 이상이고, 검사 내내 연결이 끊긴 적이 없다."),
              ("WARN", f"부하 시험에서 L1 왕복 p99 가 {p99:g} ms 를 넘었거나, 손실 없이 버틴 최대 부하가 초당 {lossless:g}번 "
                       "미만이거나, 계획한 요청의 90% 도 못 보냈다. 검사 내내 감시에서 연결 끊김, 부트로더 멈춤, 위치 신호 손실, "
                       "확인 신호 응답 없음이 하나라도 있어도 WARN 이다."),
              ("FAIL", "L0 이나 L1 에서 요청 응답, 위치 신호, 확인 신호 가운데 하나라도 잃었거나(위치 신호는 축마다 5개까지 "
                       "봐준다), 어느 부하든 거는 도중에 중단됐다."),
              ("기록만", "L4(초당 700번 더)는 한계를 보려고 일부러 넘기는 부하라 판정하지 않는다."),
              ("SKIP", "LAN 이 아니라 CAN 으로 바로 연결했거나 펌웨어를 굽는 중일 때.")]
    t = Table(["부하", "더 보낸 요청 (초당)", "응답 손실", "위치 신호 손실", "왕복 p50", "왕복 p99", "왕복 최대",
               "40 ms 이상 걸린 왕복"], num=(1, 2, 3, 4, 5, 6, 7), caption="부하 시험")
    stages = []
    for k, val in d.items():
        m = re.match(r"^게이트웨이부하\.(L\d)\s*\+(\d+)/s$", k)
        if not m:
            continue
        g = _STAGE.search(val)
        if not g:
            t.rows.append([m.group(1), f"+{m.group(2)}", (val, "muted"), "", "", "", "", ""])
            continue
        x = g.groupdict()
        stages.append((m.group(1), x))
        op = m.group(1) in ("L0", "L1")
        rl, fl = float(x["rl"]), float(x["fl"])
        rtt99 = None if x["p99"] == "—" else float(x["p99"])
        t.rows.append([
            m.group(1) + (" (평소)" if m.group(1) == "L1" else " (기록만)" if m.group(1) == "L4" else ""),
            (f"+{float(x['ach']):.0f}" + (f" (계획 {float(x['plan']):.0f})" if float(x['ach']) < 0.9 * float(x['plan']) else ""),
             "warn" if float(x['ach']) < 0.9 * float(x['plan']) and m.group(1) in ("L1", "L2", "L3") else ""),
            (f"{rl:.2f}% ({int(x['rs']) - int(x['rr'])}/{x['rs']})", "bad" if op and rl > 0 else ""),
            # 위치 신호는 축마다 5개까지 봐준다(QC 탭 기준) — 그 안이면 빨강이 아니라 주황
            (f"{fl:.2f}% ({int(x['fe']) - int(x['fr'])}개 빠짐)" if fl > 0 else f"{fl:.2f}%",
             ("bad" if fb_counted(d, m.group(1), int(x['fe']) - int(x['fr'])) else "warn") if op and fl > 0 else ""),
            "—" if x["p50"] == "—" else f"{float(x['p50']):.1f} ms",
            ("—" if rtt99 is None else f"{rtt99:.1f} ms", "warn" if m.group(1) == "L1" and rtt99 is not None and rtt99 > p99 else ""),
            "—" if x["mx"] == "—" else f"{float(x['mx']):.1f} ms",
            x["sp"] + "번"])
    if t.rows:
        up = get(d, "게이트웨이부하.무손실 상한", "게이트웨이부하.손실 없이 버틴 최대 부하", default="")
        n_up = num(up)
        cap = ("손실 없이 버틴 부하는 없다(L0 에서도 손실)" if n_up is None else
               f"손실 없이 버틴 최대 부하는 초당 {n_up:.0f}번{' 넘게' if up.strip().startswith('≥') else ''}이다")
        t.note = f"{cap}. 응답 손실 괄호 안은 못 받은 응답 수 / 보낸 요청 수다."
        v.tables.append(t)
    mon = Table(["항목", "값"], caption="검사 내내 감시 (1~15단계)")

    def m_add(label, key, fmt=lambda x: x):
        if key in d:
            mon.rows.append([label, fmt(d[key])])
    m_add("지켜본 시간", "통신감시.관측[s]",
          lambda x: x.replace(" (", " s (").replace("세지 않은 구간", "멈춤 · 연결 끊김 · 굽기로 빼 둔 시간"))
    for ax in AXES:
        m_add(f"{ax} 위치 신호 받은 수 / 기대한 수", f"통신감시.{ax} 피드백 수신/기대", lambda x: x.replace("/", " / ", 1))
        m_add(f"{ax} 위치 신호 간격 p50 / p99 / 최대", f"통신감시.{ax} 수신 간격 p50/p99/최대[ms]", lambda x: x.replace("/", " / ") + " ms")
        k = next((kk for kk in d if kk.startswith(f"통신감시.{ax} 끊김")), None)
        if k:
            gap = re.search(r">\s*([0-9.]+)\s*ms", k)
            mon.rows.append([f"{ax} 위치 신호 간격이 {gap.group(1) if gap else '25'} ms 를 넘은 횟수", d[k] + "번"])
    def rtt_cell(x: str) -> str:                    # '5.4/13.6/20.4 (n 271, 무응답 0)' → '5.4 / 13.6 / 20.4 ms (응답 271번, 응답 없음 0번)'
        x = re.sub(r"^([^ ]+)", lambda mm: mm.group(1).replace("/", " / ") + " ms", x)
        return re.sub(r"\(n (\d+), 무응답 (\d+)\)", r"(응답 \1번, 응답 없음 \2번)", x)
    for b in ("게이트웨이", "YAW", "PITCH"):
        m_add(f"{b if b == '게이트웨이' else b + ' 보드'} 왕복 p50 / p99 / 최대", f"통신감시.왕복 {b} p50/p99/최대[ms]", rtt_cell)
    m_add("40 ms 이상 걸린 왕복", "통신감시.40 ms 이상 튐", lambda x: x + "번")
    m_add("연결 끊김 / 다시 붙음", "통신감시.연결 끊김/재연결", lambda x: x.replace("/", " / "))
    m_add("부트로더에 멈춘 횟수", "통신감시.부트 정체 감지", lambda x: x + "번")
    m_add("사건", "통신감시.사건")
    if mon.rows:
        mon.note = ("모터 보드 왕복은 게이트웨이에서 CAN(게이트웨이와 모터 보드를 잇는 통신선)을 한 번 더 거쳐서 "
                    "게이트웨이 왕복보다 길다.")
        v.tables.append(mon)

    def reasons(x: str) -> str:                     # QC 탭 판정 사유(qcgwload.cpp) — 이름만 이 절의 말로
        x = plain(x.replace("CAN_CHECK 무응답", "확인 신호 응답 없음")).replace(";", " ·").replace(" /s", "번/초")
        return re.sub(r"(위치 신호 손실 \d+)장", r"\1개", x)
    if d.get("게이트웨이부하.판정 사유"):
        v.notes.append("판정 사유 — " + reasons(d["게이트웨이부하.판정 사유"]))
    if d.get("통신감시.제안 판정 사유"):
        v.notes.append("검사 내내 감시에서 걸린 것 — " + reasons(d["통신감시.제안 판정 사유"]))
    l1 = next((x for n, x in stages if n == "L1"), None)
    if l1 and s["verdict"] in ("PASS", "WARN", "FAIL"):
        lost = float(l1["rl"]) + float(l1["fl"]) + int(l1["lost"])
        fb_miss = int(l1["fe"]) - int(l1["fr"])
        # 위치 신호만 몇 개 빠졌고 봐주는 수 안이면 '잃음'이 아니다 — PASS 줄에 '있음'이 붙어 헷갈리지 않게
        lost_t = ("잃은 메시지 0" if lost == 0 else
                  f"빠진 메시지는 위치 신호 {fb_miss}개뿐(축마다 {FB_TOL}개까지 봐줌)"
                  if float(l1["rl"]) == 0 and int(l1["lost"]) == 0 and fb_miss > 0 and not fb_counted(d, "L1", fb_miss)
                  else "잃은 메시지 있음")
        up = get(d, "게이트웨이부하.무손실 상한", "게이트웨이부하.손실 없이 버틴 최대 부하", default="")
        n_up = num(up)
        cap = ("부하를 더 걸지 않은 L0 에서도 손실" if n_up is None else
               f"초당 {n_up:.0f}번 넘게 더 걸어도 손실 없음" if up.strip().startswith("≥") else f"초당 {n_up:.0f}번 더 걸어도 손실 없음")
        drops = re.search(r"끊김\s*(\d+)회", s["measured"])
        p99t = "—" if l1["p99"] == "—" else f"{float(l1['p99']):.1f} ms"
        v.line = (f"평소 부하(L1)에서 {lost_t} · 왕복 시간 p99 {p99t} · {cap}"
                  + (f" · 검사 내내 연결 끊김 {drops.group(1)}회" if drops else ""))
    else:
        v.line = plain(s["measured"])


def _s17(v: StepView, s: dict, d: dict, L: dict):
    v.what = "검사하는 동안 모터 보드가 스스로 센 이상(EEPROM 쓰기 실패, CAN 메시지 버림, 드라이버 폴트 등)이 늘었는지 본다."
    v.how = ("모터 보드는 켜진 뒤로 이상한 일이 생길 때마다 그 수를 센다(진단 카운터). 2단계와 이 단계에서 한 번씩 읽어, "
             "검사하는 동안 늘어난 수만 본다. 게이트웨이는 이런 카운터가 없어서, 검사 중 받은 크래시 기록(게이트웨이가 "
             "뜻밖에 다시 시작한 뒤 보내는 리셋 원인 알림)과 부트로더에 멈춘 횟수를 대신 본다.")
    v.why = ("검사 중 잠깐 났다가 스스로 되살아난 문제는 다른 단계에 안 드러난다. "
             "EEPROM 쓰기가 사라졌다면 방금 저장한 영점이나 프리셋이 안 남았을 수 있다.")
    v.crit = [("PASS", "표에서 '늘면 판정'이 FAIL 이나 WARN 인 다섯 가지가 하나도 안 늘었고, 게이트웨이 크래시 기록도 "
                       "부트로더 멈춤도 없다."),
              ("WARN", "CAN TX 드롭, 드라이버 폴트, bus-off 가운데 하나라도 늘었거나, 게이트웨이 크래시 기록이나 부트로더 "
                       "멈춤이 있다. 카운터가 시작보다 줄어도(검사 중 재부팅 의심) WARN 이다."),
              ("FAIL", "EEPROM 에 쓸 것을 버렸거나 EEPROM 이 제때 응답하지 않은 수가 늘었다.")]
    t = Table(["항목", "뜻", "YAW", "PITCH", "늘면 판정"], num=(2, 3))
    have = False
    for name, mean, on_up in DIAG:
        cells = []
        for ax in AXES:
            raw = d.get(f"{ax}.진단 {name}")
            if raw is None:
                cells.append(("—", "muted"))
                continue
            have = True
            m = re.search(r"시작\s*(\d+)\s*·\s*끝\s*(\d+)\s*·\s*차이\s*(\d+)", raw)
            if m:
                dlt = int(m.group(3))
                if "되감김" in raw:                 # 끝 < 시작 — 검사 중 재부팅. 차이 = 끝 값(다시 켜진 뒤 센 수)
                    txt = f"{m.group(1)} → {m.group(2)} (카운터가 줄어 재부팅 의심" + (f" · 그 뒤 +{dlt}" if dlt else "") + ")"
                else:
                    txt = f"{m.group(1)} → {m.group(2)}" + (f" (+{dlt})" if dlt else "")
                cls = ("bad" if on_up == "FAIL" else "warn" if on_up == "WARN" else "") if dlt and on_up else ""
                cells.append((txt, cls) if cls else txt)
            else:
                cells.append((raw.replace("(시작값 없음)", "(시작값이 없어 켜진 뒤 누적)"), "muted" if "응답 없음" in raw else ""))
        t.rows.append([name, mean, *cells, (on_up or "기록만", "muted")])
    if have:
        t.note = ("값은 '시작 → 끝'이고 괄호는 검사 동안 늘어난 수다. STATUS · nFAULT · 복구 I2C 는 횟수가 아니라 상태 값이다. "
                  "'응답 없음'은 보드가 그 카운터 값을 보내지 않았다는 뜻이다(판정에 쓰는 줄이면 WARN).")
        v.tables.append(t)
    if "게이트웨이.크래시 기록" in d or "게이트웨이.부트 정체 감지" in d:
        g = Table(["게이트웨이", "검사 동안"])
        cr = re.sub(r"\[경고!*\]\s*", "", d.get("게이트웨이.크래시 기록", "—"))
        g.rows.append(["크래시 기록", (cr, "warn") if cr not in ("없음", "—") else cr])
        bs = d.get("게이트웨이.부트 정체 감지", "—")
        g.rows.append(["부트로더에 멈춘 횟수", (bs + "번", "warn") if num(bs) else bs + ("번" if num(bs) is not None else "")])
        v.tables.append(g)
    if s["verdict"] == "PASS":
        v.line = "YAW · PITCH 모두 늘어난 이상 없음 · 게이트웨이 크래시도 부트로더 멈춤도 없음"
    else:
        parts = [x for x in plain(s["measured"]).split(" · ") if "늘어난 것 없음" not in x and "없음" != x[-2:]]
        v.line = " · ".join(parts) if parts else plain(s["measured"])


def _s18_text(v: StepView, L: dict):
    tp, tw = L.get("homep_pass", 0.2), L.get("homep_warn", 0.5)
    v.what = ("카메라 표준 명령(ONVIF)으로 축을 움직일 수 있는지 보고, 15단계가 두 축을 0° 로 돌려놓은 자리를 카메라 프리셋 "
              "'home' 으로 저장해 둔다. 현장에서 카메라로 home 을 부르면 PTZ 가 이 자리(정면)로 돌아온다.")
    v.how = ("먼저 한화 카메라 전용 명령(SUNAPI)으로 프리셋 목록을 읽어, home 이 이미 있으면 그 번호에, 없으면 비어 있는 가장 "
             "작은 번호(보통 1번)에 home 을 저장한다. 저장하면 카메라가 RS-485 선으로 게이트웨이에 알리고, 두 모터가 지금 자리를 "
             "EEPROM 에 적는다. 이 카메라는 ONVIF 로는 프리셋을 저장할 수 없어서 저장만 SUNAPI 로 한다. 다음으로 ONVIF 명령으로 "
             "YAW 를 10° 넘게 옮기고, ONVIF 프리셋 목록에서 home 을 찾아 부른다(GotoPreset). 두 축이 멈추면 멈춘 위치와 저장한 "
             f"자리의 차이(돌아온 자리 오차, 두 축 가운데 큰 쪽)와 {tp:g}° 안에 들어오기까지 걸린 시간을 잰다. home 은 지우지 않고 "
             "출하 설정으로 남긴다.")
    v.why = ("현장에서 home 을 부르면 늘 같은 자리로 돌아와야 한다. 명령이 카메라에서 RS-485 선과 게이트웨이를 거쳐 "
             "모터까지 가는 길도 함께 본다.")
    v.crit = [("PASS", f"home 을 부르면 두 축이 저장한 자리에서 {tp:g}° 안으로 돌아온다."),
              ("WARN", f"{tp:g}° 는 넘지만 {tw:g}° 안으로 돌아온다."),
              ("FAIL", f"{tw:g}° 밖에 멈추거나 돌아오지 않는다. ONVIF 명령으로 YAW 가 움직이지 않거나, 카메라가 home 저장이나 "
                       "목록 요청을 거절하거나 5초 안에 응답하지 않거나, ONVIF 목록에 home 이 없어도 FAIL 이다."),
              ("SKIP", "QC 탭의 '카메라·소리 검사'를 껐거나, CONFIG 탭에 줌 카메라 주소·암호가 없거나, 줌 카메라가 시험 영상일 때. "
                       "로봇 점검에서는 프리셋 변경에 동의하지 않았거나 기본 점검(레벨 1)일 때.")]


# 18 세부 값 — (키, 표 이름, 단위). 없는 키는 원래 이름 그대로 뒤에 붙인다(QC 탭이 칸을 더 적어도 보이게)
_S18_KEYS = [("18.SUNAPI 목록", "저장 전 카메라 프리셋 목록 (SUNAPI)", ""), ("18.home 번호", "home 을 저장한 번호", ""),
             ("18.SUNAPI 저장", "SUNAPI 로 저장한 결과", ""), ("18.SUNAPI 목록(저장 뒤)", "저장 뒤 카메라 프리셋 목록 (SUNAPI)", ""),
             ("18.ONVIF 이동 YAW[deg]", "ONVIF 로 옮긴 YAW 각도", "°"), ("18.ONVIF 목록", "ONVIF 프리셋 목록", ""),
             ("18.복귀 YAW/PITCH[deg]", "home 을 부른 뒤 멈춘 위치 YAW / PITCH", "°"), ("18.복귀 오차[deg]", "돌아온 자리 오차", "°"),
             ("18.걸린 시간[s]", "돌아오는 데 걸린 시간", " s")]

# 18 FAIL 한 줄 — QC 탭 글(qctab.cpp hpTick note) → 사람 말. 못 알아보면 원문 그대로
_S18_FAIL = [
    (r"^home 으로 안 돌아옴 — (YAW|PITCH) 오차 ([0-9.]+)°( \(부른 뒤 움직이지 않음\))?",
     lambda m: f"home 을 불렀지만 {m.group(1)} 가 {m.group(2)}° 떨어진 채 멈춤" + (" — 부른 뒤 움직이지 않음" if m.group(3) else "")),
    (r"^ONVIF 로 YAW 가 안 움직임 — (\d+)초 동안 ([0-9.]+)°",
     lambda m: f"ONVIF 명령으로 YAW 가 안 움직임 — {m.group(1)}초 동안 {m.group(2)}°"),
    (r"^ONVIF 목록에 home 없음\(SUNAPI 목록(엔 있음|에도 없음)\)",
     lambda m: "ONVIF 프리셋 목록에 home 이 없음" + (" (SUNAPI 목록에는 있음)" if m.group(1) == "엔 있음" else " (SUNAPI 목록에도 없음)")),
    (r"^SUNAPI 저장 실패 (.+)$", lambda m: f"카메라가 home 저장을 거절함({m.group(1)})"),
    (r"^SUNAPI 프리셋 목록 조회 실패 (.+)$", lambda m: f"카메라가 프리셋 목록 요청을 거절함({m.group(1)})"),
    (r"^빈 프리셋 번호가 없음\(1~253 이 다 참\)", lambda m: "카메라 프리셋 1~253번이 다 차 있어 home 을 저장할 번호가 없음"),
    (r"^(.+?) 무응답\((\d+) s\)", lambda m: f"카메라가 {m.group(2)}초 안에 답하지 않음 — {m.group(1)}"),
]


def _s18(v: StepView, s: dict, d: dict, L: dict):
    _s18_text(v, L)
    t = Table(["항목", "값"])
    seen = set()
    for key, label, unit in _S18_KEYS:
        if key in d:
            seen.add(key)
            val = d[key]
            parts = [x.strip() for x in val.split("/")]
            ok = unit and all(num(x) is not None for x in parts)
            t.rows.append([label, " / ".join(x.replace("-", "−") + unit for x in parts) if ok else val])
    for key, val in d.items():
        if key.startswith("18.") and key not in seen:
            t.rows.append([key[3:], val])
    if t.rows:
        t.note = ("돌아온 자리 오차는 home 을 부른 뒤 두 축이 멈춘 위치와 저장한 자리의 차이 가운데 큰 쪽이다. "
                  f"걸린 시간은 부른 때부터 {L.get('homep_pass', 0.2):g}° 안에 처음 들어올 때까지다.")
        v.tables.append(t)
    msr = s["measured"] or ""
    m = re.search(r"home = (\d+)번(\(덮어씀\))? · ONVIF 로 YAW ([+-]?[0-9.]+)° · 복귀 오차 ([0-9.]+)° \(([0-9.]+) s\)", msr)
    if m and s["verdict"] in ("PASS", "WARN"):
        v.line = (f"home 을 {m.group(1)}번에 저장{'(있던 home 을 덮어씀)' if m.group(2) else ''} · ONVIF 로 불러 "
                  f"{fx(float(m.group(5)), 1, '초')} 만에 돌아옴 · 돌아온 자리 오차 {m.group(4)}° "
                  f"(기준 {L.get('homep_pass', 0.2):g}° 이하)")
        return
    for pat, say in _S18_FAIL:
        mm = re.search(pat, msr)
        if mm:
            v.line = say(mm)
            return
    v.line = plain(msr)


def _s18_absent(v: StepView, L: dict):
    """CSV 에 18단계가 없다 — 이 검사를 돌린 QC 탭에 아직 없던 단계(09-29 전)."""
    _s18_text(v, L)
    v.verdict = "기록 없음"
    v.how = v.why = ""
    v.crit = []
    v.skip = "이 검사를 돌린 QC 탭에는 이 단계가 아직 없었다."
    v.line = "이 검사 때 QC 탭에 없던 단계"


def soak_length(soak: dict | None, mdir: Path | None) -> str:
    """공회전 길이 — '12시간' · '2시간' · '30분'. soak.json 의 plannedS, 없으면 로봇 run.json 의 soakMin(로봇 에이징은
    30분 · 2시간 · 12시간), 둘 다 없으면 QC 스테이션의 12시간."""
    sec = 0.0
    try:
        sec = float((soak or {}).get("plannedS") or 0)
    except (TypeError, ValueError):
        sec = 0.0
    if sec <= 0 and mdir:
        try:
            rb = json.loads((Path(mdir) / "run.json").read_text(encoding="utf-8")).get("robot") or {}
            sec = float(rb.get("soakMin") or 0) * 60 if rb.get("kind") == "aging" else 0.0
        except (OSError, ValueError, AttributeError, TypeError):
            sec = 0.0
    if sec <= 0:
        return "12시간"
    m = int(round(sec / 60))
    if m >= 2880 and m % 1440 == 0:      # 이틀 이상만 일로(24시간은 그대로)
        return f"{m // 1440}일"
    return f"{m // 60}시간" if m >= 60 and m % 60 == 0 else f"{m // 60}시간 {m % 60}분" if m >= 60 else f"{m}분"


def _s19(v: StepView, soak: dict | None, stage: str | None = None, qc_overall: str = "", length: str = "12시간"):
    """stage — 보고서 단계(interim = 중간, final = 최종, None = 모름 · PDF). soak.json 이 없을 때만 쓴다.
    qc_overall — QC 종합. PASS · WARN 이 아니면 자동 흐름이 공회전을 잇지 않는다(qcflow.cpp soakNext).
    length — 공회전 길이(soak_length). 단계 이름도 이것으로 — 로봇 에이징은 30분 · 2시간도 있다."""
    v.name = f"{length} 공회전"
    v.what = f"QC 를 통과한 제품을 {length} 동안 쉬지 않고 왕복시키면서, 위치 신호 · 통신 · 카메라 영상이 끊기거나 느려지지 않는지 본다."
    v.how = ("공회전 탭이 두 축을 사인파로 계속 왕복시키며 1초마다 네 가지를 잰다. 축마다 받은 위치 신호 수, 명령을 따라가는 "
             "오차, 통신 왕복 시간(PC 가 확인 신호 CAN_CHECK 를 보내고 답이 돌아오기까지), 카메라 초당 프레임이다. 따라가는 "
             "오차와 카메라 프레임은 공회전 처음 구간(기준 구간)의 제 값으로 한계를 정하고 그 뒤 1초마다 견준다. 통신이 밀린 "
             "때는 따라가는 오차의 한계를 늦어진 만큼 늘려서, 통신 탓인지 제품 탓인지 가른다. 잰 값은 soak.csv 에 한 줄씩 쓰고, "
             "끝나면 soak.json 에 판정을 남긴다.")
    v.why = "짧은 QC 에서는 안 보이고 오래 돌려야 드러나는 끊김이나 느려짐을 잡는다."
    # 판정 v2(09-29) 요약 — 숫자까지 든 글은 qc_soak.crit_v2 가 이 자리를 덮는다
    v.crit = [("PASS", "끝까지 돌았고 통신 지연도, 진짜 이상도 없다."),
              ("FAIL", "연결 끊김, 위치 신호 부족, 카메라 프레임 부족이 1초라도 있었다. 따라가는 오차가 늘린 한계를 넘거나 확인 신호에 "
                       "응답이 없는 상태가 3초 이상 이어졌다. 드라이버 폴트가 났거나, 도중에 모터가 다시 켜졌거나, PC 가 잠들었다."),
              ("중지", "FAIL 에는 안 걸렸지만 끝까지 못 돌고 멈췄다. 통신 지연이 있었어도 중지로 본다."),
              ("WARN", "끝까지 돌았지만 통신 지연(확인 신호 왕복이 100 ms 를 넘은 때)이 있었다. 제품 고장이 아니라 PC 와 게이트웨이 "
                       "사이 통신이 밀린 것이다. 따라가는 오차가 늘린 한계를 넘거나 확인 신호의 응답이 오지 않은 때가 있었지만, 3초 "
                       "이상 이어지지 않은 것도 WARN 이다.")]
    if not soak:
        if qc_overall and qc_overall not in ("PASS", "WARN"):
            v.verdict = "안 함"
            v.skip = "QC 종합이 PASS 나 WARN 이 아니면 공회전을 이어 돌리지 않는다."
            v.line = "QC 가 PASS 나 WARN 이 아니어서 공회전을 돌리지 않음"
        elif stage == "interim":
            v.verdict = "공회전 중"
            v.skip = f"QC 가 끝나고 {length} 공회전을 돌리는 중이다. 끝나면 이 페이지가 최종 보고서로 바뀌고 여기에 판정이 붙는다."
            v.line = "공회전을 돌리는 중 — 끝나면 판정이 붙는다"
        elif stage == "final":
            v.verdict = "안 함"
            v.skip = "공회전 기록(soak.json) 없이 최종 보고서를 냈다. 공회전을 돌리지 않았거나 시작하지 못했다."
            v.line = f"{length} 공회전을 돌리지 않음"
        else:
            v.verdict = "기록 없음"
            v.skip = "이 보고서를 만들 때 공회전 기록(soak.json)이 없었다. 공회전을 아직 안 돌렸거나 돌리지 않았다."
            v.line = "공회전 기록 없음"
        return
    st = soak.get("stats") or {}
    ran, plan = soak.get("ranS"), soak.get("plannedS")
    out = st.get("outlierRows")
    outt = f"기준을 벗어난 시간 {out if out is not None else '—'}초"
    if soak.get("state") == "running":
        v.verdict = "공회전 중"
        v.line = f"{dur(plan)} 가운데 {dur(ran)} 돌림 · {outt}"
    else:
        v.verdict = soak.get("verdict") or "—"
        whole = soak.get("completed") or (ran is not None and plan is not None and float(ran) >= float(plan) - 0.5)
        v.line = (f"계획한 {dur(plan)} 다 돌림 · {outt}" if whole else f"{dur(plan)} 가운데 {dur(ran)} 돌고 멈춤 · {outt}")
    # 드라이버 폴트 — 도는 중에 걸려 멈춘 것(fault) 또는 진단 카운터로 센 것(reasons 에 적힘)
    fault = soak.get("fault") or next((r for r in soak.get("reasons") or [] if "드라이버 폴트" in str(r)), "")
    t = Table(["판정", "돌린 시간 / 계획", "기준을 벗어난 시간", "드라이버 폴트", "시작 ~ 끝"], num=(2,))
    t.rows.append([cell_v(v.verdict) if v.verdict in ("PASS", "WARN", "FAIL") else (v.verdict, "muted"),
                   f"{dur(ran)} / {dur(plan)}", f"{out if out is not None else '—'}초",
                   (str(fault), "bad") if fault else "없음",
                   f"{soak.get('startedAt') or '—'} ~ {soak.get('endedAt') or '—'}"])
    v.tables.append(t)
    if soak.get("reasons"):
        v.notes.append("판정 사유 — " + " · ".join(str(x) for x in soak["reasons"] if x))


# ---------------------------------------------------------------- 옛 글자 → 새 글자(파싱 못 한 한 줄 결과에만)
PLAIN = [
    ("선 곳", "멈춘 위치"), (" OS ", " 목표를 넘어간 정도 "), ("스트로크", "끝에서 끝까지 거리"), ("임계의", "최소 기준의"), ("임계", "최소 기준"),
    ("p-p", "흔들림 폭"), ("정착오차", "멈춘 위치 오차"), ("잔류전류", "남은 전류"), ("잔류", "남은 전류"),
    ("구동 ON", "구동 켜짐"), ("구동 OFF", "구동 꺼짐"), ("HOME_FAIL(5)", "홈잉 실패"), ("부트 정체", "부트로더 멈춤"),
    ("무손실 상한", "손실 없이 버틴 최대 부하"), ("피드백", "위치 신호"), ("무응답", "응답 없음"),
    ("카운터 되감김", "카운터가 시작보다 줄어듦"), ("시작값 없어 누적으로 봄", "시작값이 없어 켜진 뒤 누적으로 봄"),
]


def plain(text: str) -> str:
    t = text or ""
    t = re.sub(r"(\d)s I (\d)", r"\1 s 전류 \2", t)
    t = re.sub(r"(\d(?:\.\d+)?)A\b", r"\1 A", t)
    for a, b in PLAIN:
        t = t.replace(a, b)
    return t.strip() or "—"


# ---------------------------------------------------------------- WARN · FAIL 까닭이 한 줄 결과에 빠졌을 때
def missed_reason(no: int, s: dict, line: str, aborted_here: bool) -> str:
    """단계 글이 만든 한 줄이 판정 까닭을 못 담는 경우만 — 도중 중지 · 단계 시간 초과 · 응답 없음 · 16 감시 사유."""
    m_ = s.get("measured") or ""
    why = []
    if aborted_here:
        why.append("도중에 사람이 검사를 중지함")
    t = re.search(r"(?:^| · )시간 초과 ([0-9.]+) s$", m_)
    if t and "시간" not in line:
        why.append(f"단계 제한 시간 {float(t.group(1)):g}초를 넘김")
    if no in (6, 11):
        why += [plain(seg) for seg in m_.split(" · ") if ("무응답" in seg or "응답 없음" in seg or "실패" in seg)
                and plain(seg) not in line]
    if no == 16:
        for part in m_.split(" · 상시 감시(1~15단계): "):
            tail = part.partition(" — ")[2]
            why += [plain(w) for w in re.split(r";\s*", tail) if w.strip()]
    return " · ".join(dict.fromkeys(w for w in why if w))


# ---------------------------------------------------------------- 전체
def read_soak(mdir: Path | None) -> dict | None:
    if not mdir:
        return None
    try:
        j = json.loads((Path(mdir) / "soak.json").read_text(encoding="utf-8"))
        return j if isinstance(j, dict) else None
    except (OSError, ValueError):
        return None


def soak_section(mdir: Path | None, web: bool, stage: str | None = None) -> str:
    """공회전 그래프 · 재검사 전후 비교가 들어갈 자리 — 제품 페이지와 PDF 가 '단계별 자세히' 바로 뒤에 끼운다.

    report/qc_soak.py(따로 만드는 모듈)가 있으면 그 section_html(mdir, web, stage) 이 돌려준 HTML 조각을
    그대로 쓴다(제목까지 그 모듈이 만든다). 모듈이 없으면 빈 글, 모듈이 실패하면 한 줄 알림 — 보고서는 늘 나온다."""
    if not mdir or not Path(__file__).with_name("qc_soak.py").exists():
        return ""
    try:
        import qc_soak  # noqa: PLC0415  있을 때만 부른다
        fn = getattr(qc_soak, "section_html", None)
        return (fn(Path(mdir), web, stage) or "") if fn else ""
    except Exception as e:                         # 공회전 절이 틀려도 보고서는 나와야 한다
        return '<p class="tnote">(공회전 그래프를 못 그림 — ' + esc(f"{type(e).__name__}: {e}") + ")</p>"


def soak_fill(v: StepView, mdir: Path | None, stage: str | None = None) -> None:
    """19 절에 공회전 자료(한눈에 표 · 항목별 표 · 시간 그래프 · 기준을 벗어난 1초 · 사건 · 기준 칸의 문턱 숫자)를 더한다 —
    qc_soak.fill_step19. 모듈이 없으면 _s19 가 쓴 그대로, 모듈이 실패하면 한 줄 알림만 — 보고서는 늘 나온다."""
    if not Path(__file__).with_name("qc_soak.py").exists():
        return
    try:
        import qc_soak  # noqa: PLC0415  있을 때만 부른다
        qc_soak.fill_step19(v, mdir, stage)
    except Exception as e:                         # 공회전 자료가 틀려도 보고서는 나와야 한다
        v.notes.append(f"(공회전 자료를 못 붙임 — {type(e).__name__}: {e})")


def build(qc: dict, mdir: Path | None = None, stage: str | None = None) -> list[StepView]:
    """CSV 한 건(+ 검사 폴더의 soak.json) → 단계 1~19. stage = 보고서 단계(interim · final, 모르면 None) — 19 에만 쓴다."""
    L = limits(qc)
    d = dict(qc["details"])
    views: dict[int, StepView] = {}
    last_fail = max([step_no(s["name"]) for s in qc["steps"] if s["verdict"] == "FAIL"] or [0])
    for s in qc["steps"]:
        no = step_no(s["name"])
        if not no:
            continue
        name = re.sub(r"^\s*\d+\s*", "", s["name"]).strip() or TITLE.get(no, s["name"])
        v = StepView(no, name, s["verdict"], "")
        fn = {1: _s1, 2: _s2, 3: _s3, 4: _s4, 5: _s5, 6: _s6, 7: _s7, 8: _s8, 11: _s11, 12: _s12, 13: _s13,
              14: _s14, 16: _s16, 17: _s17, 18: _s18}.get(no)
        try:
            if fn:
                fn(v, s, d, L)
            elif no == 9:
                _s9(v, s, d, L, qc["details"])
            elif no == 10:
                _s10(v, s, d, L, qc["details"])
            elif no == 15:
                _s15(v, s, d, L, qc["details"])
            else:
                v.line = plain(s["measured"])
        except Exception as e:                     # 모양이 뜻밖이면 원문 한 줄로라도 — 보고서가 통째로 죽지 않게
            v.tables = []
            v.line = plain(s["measured"])
            v.notes.append(f"(세부 표를 만들지 못해 QC 탭이 남긴 한 줄만 적었다. 값은 아래 부록에 있다 — {type(e).__name__}: {e})")
        if s["verdict"] == "SKIP":
            v.line, v.skip = skip_reason(no, s["measured"], views)
            v.tables = []
        elif s["verdict"] in ("WARN", "FAIL"):
            # 단계 글이 만든 한 줄이 판정 까닭을 못 담았으면(도중 중지 · 단계 시간 초과 · 응답 없음 …) 앞에 붙인다
            aborted_here = qc["head"].get("종합") == "중지" and s["verdict"] == "FAIL" and no == last_fail
            miss = missed_reason(no, s, v.line, aborted_here)
            if miss:
                v.line = miss + " — " + v.line if v.line and v.line != "—" else miss
            if aborted_here:
                v.notes.append("이 단계 도중에 사람이 검사를 중지했다. 뒤 단계는 돌지 않았다.")
        views[no] = v
    for no in (18, 19):
        if no not in views:
            v = StepView(no, TITLE[no], "", "")
            if no == 18:
                _s18_absent(v, L)
            else:
                soak = read_soak(mdir)
                _s19(v, soak, stage, qc["head"].get("종합", ""), soak_length(soak, mdir))
            if no == 19:
                soak_fill(v, mdir, stage)
            views[no] = v
    return [views[k] for k in sorted(views)]


# ---------------------------------------------------------------- HTML — 웹과 PDF 가 같이 쓴다
def _attr(classes: str) -> str:
    return ' class="' + classes + '"' if classes else ""


def _cell(c, numeric: bool) -> str:
    text, cls = (c if isinstance(c, tuple) else (c, ""))
    classes = " ".join(x for x in (("num" if numeric else ""), cls) if x)
    return "<td" + _attr(classes) + ">" + esc(text) + "</td>"


def table_html(t: Table | str, web: bool) -> str:
    """표 하나 → HTML. v.tables 에는 Table 말고 HTML 조각(str)도 들어올 수 있다 — 공회전 그래프처럼 표가 아닌 것(qc_soak)."""
    if isinstance(t, str):
        return t
    head = "".join("<th" + _attr("num" if i in t.num else "") + ">" + esc(c) + "</th>" for i, c in enumerate(t.cols))
    body = "".join("<tr>" + "".join(_cell(c, i in t.num) for i, c in enumerate(r)) + "</tr>" for r in t.rows)
    out = ('<p class="cap">' + esc(t.caption) + "</p>" if t.caption else "") + \
        '<div class="table-wrap"><table class="mt"><thead><tr>' + head + "</tr></thead><tbody>" + body + \
        "</tbody></table></div>" + ('<p class="tnote">' + esc(t.note) + "</p>" if t.note else "")
    if web and t.fold:
        return '<details class="fold"><summary>' + esc(t.fold) + "</summary>" + out + "</details>"
    return out


table_html._takes_str = True                   # qc_soak 미리보기가 보는 표지 — HTML 조각을 그대로 그린다


def summary_html(views: list[StepView], pill, link: bool = True) -> str:
    rows = []
    for v in views:
        name = '<a href="#s-' + str(v.no) + '">' + esc(v.name) + "</a>" if link else esc(v.name)
        rows.append('<tr class="r-' + (v.vclass or "none") + '"><td class="no">' + str(v.no) + "</td>"
                    '<td class="nm">' + name + '</td><td class="vd">' + pill(v) + '</td><td class="ln">'
                    + esc(v.line) + "</td></tr>")
    return ('<div class="table-wrap"><table class="sum"><thead><tr><th class="no">번호</th><th>검사</th><th>판정</th>'
            "<th>결과</th></tr></thead><tbody>" + "".join(rows) + "</tbody></table></div>")


def _crit_tag(k: str, pill) -> str:
    if k in ("PASS", "WARN", "FAIL", "SKIP", "중지"):
        return pill(StepView(0, "", k, ""))
    return '<span class="ctag">' + esc(k) + "</span>" if k else "<span></span>"


def sections_html(views: list[StepView], pill, web: bool) -> str:
    out = []
    for v in views:
        wh = "".join("<dt>" + a + "</dt><dd>" + esc(b) + "</dd>"
                     for a, b in (("무엇을", v.what), ("어떻게", v.how), ("왜", v.why)) if b)
        crit = ""
        if v.crit:
            items = "".join("<li>" + _crit_tag(k, pill) + "<span>" + esc(txt) + "</span></li>" for k, txt in v.crit)
            crit = '<div class="crit"><p class="lbl">기준</p><ul>' + items + "</ul></div>"
        head = ('<p class="skipwhy"><b>' + ("건너뜀" if v.verdict == "SKIP" else esc(v.verdict)) + "</b> — "
                + esc(v.skip) + "</p>") if v.skip else '<p class="line">' + esc(v.line) + "</p>"
        tables = "".join(table_html(t, web) for t in v.tables)
        notes = "".join('<p class="tnote">' + esc(n) + "</p>" for n in v.notes)
        out.append('<section class="st" id="s-' + str(v.no) + '"><h3><span class="sno">' + str(v.no) + "</span> "
                   + esc(v.name) + " " + pill(v) + "</h3>" + head + '<dl class="wh">' + wh + "</dl>"
                   + crit + tables + notes + "</section>")
    return "\n".join(out)


# 두 판이 같이 쓰는 모양 — 색은 웹 토큰(var(--…))이 있으면 그걸, PDF 는 qc_report 가 덮는다
STEPS_CSS = """
table.sum td.no, table.sum th.no { width: 3.5ch; text-align: right; color: var(--muted, #666); font-variant-numeric: tabular-nums; }
table.sum td.vd { white-space: nowrap; }
table.sum td.nm { white-space: nowrap; }
table.sum td.nm a { text-decoration: none; }
section.st { border-top: 1px solid var(--line, #ccc); padding-top: 16px; margin-top: 18px; }
section.st h3 { font-size: 16px; font-weight: 600; margin: 0 0 6px; display: flex; flex-wrap: wrap; align-items: center; gap: 4px 10px; }
section.st .sno { font-family: var(--mono, monospace); color: var(--muted, #666); font-weight: 500; }
section.st, table.sum { word-break: keep-all; overflow-wrap: break-word; }   /* 한글은 낱말 사이에서만 줄을 바꾼다 */
section.st p.line { margin: 0 0 10px; font-weight: 500; }
section.st p.skipwhy { margin: 0 0 10px; color: var(--muted, #555); }
section.st dl.wh { display: grid; grid-template-columns: max-content 1fr; gap: 4px 14px; margin: 0 0 10px; max-width: 86ch; }
section.st dl.wh dt { color: var(--accent, #0f6b74); font-weight: 600; font-size: 13px; padding-top: 1px; }
section.st dl.wh dd { margin: 0; }
section.st .crit { margin: 0 0 10px; }
section.st .crit .lbl { margin: 0 0 4px; font-weight: 600; font-size: 13px; color: var(--accent, #0f6b74); }
section.st .crit ul { list-style: none; margin: 0; padding: 0; display: grid; gap: 4px; max-width: 86ch; }
section.st .crit li { display: grid; grid-template-columns: 6.5em 1fr; gap: 10px; align-items: baseline; }
section.st .crit li > :first-child { justify-self: start; }
section.st .crit .ctag { font-size: 12px; color: var(--muted, #666); white-space: nowrap; }
section.st p.cap { margin: 10px 0 4px; font-weight: 600; font-size: 13px; }
section.st table.mt { margin-bottom: 2px; width: auto; min-width: min(100%, 520px); }
section.st table.mt td.ok { color: var(--pass, #1e7d3c); }
section.st table.mt td.warn { color: var(--warn, #a86400); font-weight: 600; }
section.st table.mt td.bad { color: var(--fail, #c0392b); font-weight: 600; }
section.st table.mt td.muted { color: var(--muted, #666); }
section.st p.tnote { margin: 4px 0 0; font-size: 12.5px; color: var(--muted, #666); max-width: 100ch; }
section.st details.fold summary { cursor: pointer; font-weight: 600; font-size: 13px; margin: 6px 0; }
@media (max-width: 640px) {
  section.st dl.wh { grid-template-columns: 1fr; gap: 0 0; }
  section.st dl.wh dd { margin-bottom: 6px; }
  section.st .crit li { grid-template-columns: 1fr; gap: 2px; }
  section.st table.mt th { white-space: normal; }
  section.st table.mt th, section.st table.mt td { padding: 6px 7px; }
  /* 요약표는 폰에서 줄마다 두 줄로 — 번호 · 검사 · 판정, 그 밑에 결과 */
  table.sum thead { display: none; }
  table.sum, table.sum tbody { display: block; }
  table.sum tr { display: grid; grid-template-columns: 3ch 1fr auto; gap: 2px 10px; padding: 9px 0;
                 border-bottom: 1px solid var(--line, #ccc); }
  table.sum td { display: block; border: 0; padding: 0; }
  table.sum td.nm { white-space: normal; font-weight: 500; }
  table.sum td.ln { grid-column: 2 / -1; min-width: 0; }
}
"""


# ---------------------------------------------------------------- 확인용
def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from qc_report import read_qc_csv
    for p in sys.argv[1:]:
        path = Path(p)
        qc = read_qc_csv(path)
        print(f"== {path.name}")
        for v in build(qc, path.with_suffix("")):
            print(f"{v.no:>2} {v.name:<14} {v.verdict or '—':<6} {v.line}")
            if v.skip:
                print(f"   ↳ {v.skip}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
