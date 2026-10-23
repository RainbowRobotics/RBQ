#!/usr/bin/env python3
"""19 12시간 공회전 자료와 20 공회전 뒤 재검사(전·후 비교) — 웹 제품 페이지와 PDF 가 같이 쓴다(09-29).

    python3 qc_soak.py <처음 QC CSV> [--stage interim|final] [--html 미리보기.html]   # 19 · 20 · 출하 판정을 터미널에

읽는 것 — 처음 검사 폴더(<QC 폴더>/<stem>/, CSV 와 같은 이름):
  soak.json   soaktab.cpp writeJson — 판정 · 시간 · 한계 · 기준 구간 · 합계 · 진단 카운터 · 기준을 벗어난 1초(앞 500) · 사건
  soak.csv    1초 한 줄. BOM + '# PTZ 공회전 · …' 설명 줄 + 머리줄
              시각,경과[s],YAW 수신,PITCH 수신,YAW 추종[°],PITCH 추종[°],왕복[ms],무응답,줌 fps,열화상 fps,이상,YAW 공백[ms],PITCH 공백[ms]
              (공백 두 칸은 10-16 부터 — 위치 신호 사이 가장 긴 공백, 판정에는 안 씀)
  soak_clip/  공회전 끝 30초 녹화(zoom.mkv · thermal.mkv)
  after.json  qcflow.cpp writeLinks — 재검사 CSV {csv, stem, startedAt, overall}. 재검사 폴더에는 before.json
붙는 자리 — qc_steps 가 부른다(E 가 둔 자리, 09-29).
  fill_step19(v, mdir, stage)     19 절(qc_steps._s19 가 글 · 판정 · 한 줄을 쓴 뒤) — 한눈에 표 · 항목별 표 · 항목별 시간
                                  그래프(인라인 SVG) · 기준을 벗어난 1초 · 사건, 기준 칸에 문턱 숫자
  section_html(mdir, web, stage)  '단계별 자세히' 바로 뒤(qc_steps.soak_section) — 공회전 끝 영상(웹) · 20 절(재검사)
그 밖에 출하 판정 한 단어(final_verdict · row_fields — 대시보드 줄 · 판정 도장)와 재검사 사진(recheck_photos).

판정 — 재검사는 그 CSV 의 '종합'을 옮긴다. 공회전은 판정 v2(09-29, judge() — C++ soakcore · soaktab 과 같은 규칙)로 본다.
soak.json 의 "rule" 이 지금 규칙 판(RULE)이면 그 판정과 칸을 쓰고, soak.csv 로 다시 센 값과 다르면 stderr 에 알린다. 옛 판이거나
없으면(옛 기록) soak.csv 로 다시 세고, 기록 당시 판정(옛 기준)은 작게 곁들인다. 여기서 새로 정하는 것은 '크게 변함' 하나다
(CHANGE_FRAC): 단계 판정이 나빠졌거나, 한계가 있는 주요 수치가 한계 쪽으로 한계의 25% 이상 움직였다.

주의 — soak.json 의 stats.trackMax · fpsMin 은 기준 구간(사인파에 들어가는 첫 몇 초 포함)까지 넣은 값이다(09-29
실보드: YAW 5.90° 가 경과 1초 값). 한계와 견주는 값은 soak.csv 에서 경과 ≥ 기준 구간인 줄로 다시 센다.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import qc_frames  # noqa: E402  재검사 녹화 밀착 인화
from qc_steps import DIAG, STROKE_DEG, StepView, Table, cell_v, comma, dur, esc, fx, num  # noqa: E402

MAX_POINTS = 720                  # 그래프 한 장의 칸 수 — 12시간이면 1분 칸
BUCKETS = (1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600)
OUTLIER_SHOW = 50                 # 목록에 적는 기준을 벗어난 1초(나머지는 soak.csv 에)
EVENTS_SHOW = 60
CHANGE_FRAC = 0.25                # 크게 변함 — 한계 쪽으로 한계의 25% 이상
CUR_SKIP_S = 30                   # 전류 앞쪽 창에서 뺄 처음 초 — 사인파로 들어가는 순간의 튐
STALE_S = 300                     # 도는 중인데 soak.json · soak.csv 가 이만큼 안 바뀌면 기록이 멈춘 것(soak.json 은 60초마다)
AX_NAME = ("YAW", "PITCH")
CAM_NAME = ("줌", "열화상")
CAM_KEY = ("zoom", "thermal")
RANK = {"PASS": 1, "WARN": 2, "FAIL": 3}

# 공회전 판정 v2(09-29) — soakcore · soaktab(C++)과 같은 규칙. 12시간 세 대 실측에서 기준을 벗어난 1초가 거의 다 PC ↔
# 게이트웨이 통신 지연이었다. 왕복이 늘면 위치 소식이 늦게 와서 PC 가 조금 전 각도를 명령과 견주니 따라가는 오차도 커 보인다.
# 그래서 통신 지연은 WARN(제품 고장 아님)으로 따로 세고, 따라가는 오차는 그 1초에 늦어진 만큼 한계를 늘려 본다.
# 개정 1(09-29): 판정 순서 FAIL → 중지 → WARN → PASS · 기준 구간도 판정(빼는 것은 추종 · fps 비율뿐) ·
# 늘릴 때 쓰는 왕복 = max(직전 줄, 그 줄, 다음 줄)(규칙 4 — 개정 1 은 직전 줄까지).
# 규칙 4(10-16): 위치 신호가 모자란 초도 바로 다음 초와 합이 2 × fbMinPerS 이상이면 '위치 신호 밀림'(통신 · WARN).
# 숫자 · 경계(> · ≥) · 연속 세기 · 빈칸 메우기가 C++ 과 한 글자라도 다르면 안 된다 — 바꾸면 soaktab.cpp 도 같이.
RULE = 4                         # 3: 확인 신호 답 없음(10-14) · 4: 위치 신호 밀림 · 다음 줄 왕복(10-16). 옛 판 기록은 soak.csv 로 다시 센다(재판정)
LATENCY_IS_WARN = True           # False 면 통신 지연은 기록만(판정에 안 씀)
RUN_FAIL_S = 3                   # 따라가는 오차가 이만큼 연달아면 FAIL
# 확인 신호 답 없음(v3, 10-14 결정) — 따로 한 번씩은 기록만. 연달아 3초 WARN · 5초 FAIL. 횟수는 24시간에 30번을 넘으면
# FAIL, 그 2/3 이상이면 WARN — 한도는 공회전 시간(1초 줄 수)에 비례해 줄인다. soakcore.h SoakRule 과 같다
LOST_WARN_RUN_S = 3
LOST_FAIL_RUN_S = 5
LOST_FAIL_PER_DAY = 30.0
LOST_WARN_FRAC = 2.0 / 3.0


def lost_fail_count(rows: int) -> float:
    return LOST_FAIL_PER_DAY * rows / 86400.0


def lost_level(count: int, run: int, rows: int) -> str:
    """확인 신호 답 없음의 몫 — FAIL · WARN · ''(기록만 · 없음)."""
    lim = lost_fail_count(rows)
    if run >= LOST_FAIL_RUN_S or count > lim:
        return "FAIL"
    if run >= LOST_WARN_RUN_S or (count > 0 and count >= LOST_WARN_FRAC * lim):
        return "WARN"
    return ""
RTT_BASE_DEFAULT_MS = 10.0       # 기준 구간에 왕복 값이 하나도 없을 때의 평소 왕복 [ms]
RTT_FILL_S = 5                   # 왕복이 빈 1초는 직전 이만큼 안의 마지막 왕복으로 메운다 [s]
DIAG_FAULT_IDX = 4               # 진단 카운터 '드라이버 폴트' — 늘면 FAIL
DIAG_STATE_IDX = (6, 7, 8)       # STATUS · nFAULT · 복구 I2C — 상태값이라 줄어듦 검사에서 뺀다
REAL = ("hard0", "hard1", "lost", "fb0", "fb1", "cam0", "cam1", "link")   # 진짜 이상(FAIL 요소) 표지 — late0/1(밀림)은 통신 탓

# soak.csv 머리 — 이름으로 찾는다(칸이 늘거나 순서가 바뀌어도 읽히게)
COLS = {"t": "시각", "sec": "경과[s]", "fb0": "YAW 수신", "fb1": "PITCH 수신", "tr0": "YAW 추종[°]", "tr1": "PITCH 추종[°]",
        "rtt": "왕복[ms]", "lost": "무응답", "fps0": "줌 fps", "fps1": "열화상 fps", "why": "이상",
        "gap0": "YAW 공백[ms]", "gap1": "PITCH 공백[ms]"}


# ---------------------------------------------------------------- 읽기
def read_json(p: Path) -> dict | None:
    try:
        j = json.loads(p.read_text(encoding="utf-8"))
        return j if isinstance(j, dict) else None
    except (OSError, ValueError):
        return None


def _int(s: str):
    s = s.strip()
    return int(float(s)) if s else None


def _flt(s: str):
    s = s.strip()
    return float(s) if s else None


def read_rows(path: Path) -> dict | None:
    """soak.csv → 열마다 리스트. 빈 칸(위치 신호 없음 · 왕복 없음 · 안 보는 카메라)은 None.
    도는 중이면 마지막 줄이 반쯤 쓰였을 수 있다 — 못 읽는 줄은 건너뛴다."""
    try:
        f = path.open(encoding="utf-8-sig", newline="")
    except OSError:
        return None
    out = {k: [] for k in COLS}
    out["bad"] = 0
    with f:
        idx = None
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            if idx is None:
                if row[0].strip() == COLS["t"]:
                    names = [c.strip() for c in row]
                    idx = {k: (names.index(n) if n in names else None) for k, n in COLS.items()}
                continue
            try:
                vals = {}
                for k, i in idx.items():
                    raw = row[i] if i is not None and i < len(row) else ""
                    if k in ("t", "why"):
                        vals[k] = raw
                    elif k in ("sec", "fb0", "fb1", "lost", "fps0", "fps1"):
                        vals[k] = _int(raw)
                    else:
                        vals[k] = _flt(raw)
                if vals["sec"] is None:
                    raise ValueError
            except (ValueError, IndexError):
                out["bad"] += 1
                continue
            for k, v in vals.items():
                out[k].append(v)
    return out if idx is not None else None


def read_cur(path: Path) -> dict | None:
    """soak_cur.csv → {"sec", "cur0", "cur1"}. 1초에 한 번 물어 본 두 축 전류 크기 [A] — 판정에는 쓰지 않는다.
    없으면(옛 기록 · 전류를 안 잰 회차) None. 답이 안 온 축은 None."""
    try:
        f = path.open(encoding="utf-8-sig", newline="")
    except OSError:
        return None
    out = {"sec": [], "cur0": [], "cur1": []}
    with f:
        head = False
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            if not head:
                head = row[0].strip() == COLS["t"]
                continue
            try:
                sec = _int(row[1])
                c0, c1 = _flt(row[2]), _flt(row[3])
            except (ValueError, IndexError):
                continue                           # 도는 중이면 마지막 줄이 반쯤 쓰였을 수 있다
            if sec is None:
                continue
            out["sec"].append(sec)
            out["cur0"].append(c0)
            out["cur1"].append(c1)
    return out if out["sec"] else None


def cur_trend(cur: dict | None, ax: int, extent: int) -> dict | None:
    """전류가 올랐나 — 앞쪽 · 끝쪽 창(전체의 10%, 60초~10분)에서 가장 큰 값과 전체에서 가장 큰 값.
    사인파 한 바퀴(20초)가 여러 번 들어가는 창이라 '그 무렵의 최고 전류'가 된다. 앞쪽 창은 처음 CUR_SKIP_S 를
    빼고 잡는다 — 사인파로 들어가는 순간의 튐이 앞쪽 값을 부풀린다(10-01 실물: PITCH 시작 10초에 2.24 A, 그 뒤 1.5 A 안팎)."""
    if not cur:
        return None
    pts = [(s, v) for s, v in zip(cur["sec"], cur[f"cur{ax}"]) if v is not None]
    if not pts:
        return None
    last = max(s for s, _ in pts)
    win = int(min(600, max(60, extent * 0.1)))
    head = [v for s, v in pts if CUR_SKIP_S <= s < CUR_SKIP_S + win]
    tail = [v for s, v in pts if s > last - win]
    out = {"max": max(v for _, v in pts), "win": win, "first": max(head) if head else None, "last": None, "pct": None}
    if head and tail and last + 1 >= CUR_SKIP_S + 2 * win:   # 두 창이 겹치지 않을 만큼 돌았을 때만 견준다
        out["last"] = max(tail)
        if out["first"] > 0:
            out["pct"] = 100.0 * (out["last"] - out["first"]) / out["first"]
    return out


def read_after(mdir: Path | None) -> dict | None:
    return read_json(Path(mdir) / "after.json") if mdir else None


def recheck_csv(csv1: Path, after: dict | None) -> Path | None:
    """after.json 이 가리키는 재검사 CSV. 그 경로가 없으면(폴더를 옮김) 처음 CSV 옆에서 같은 이름을 찾는다."""
    if not after:
        return None
    cands = []
    if after.get("csv"):
        cands.append(Path(str(after["csv"])))
    stem = after.get("stem") or (Path(str(after.get("csv"))).stem if after.get("csv") else "")
    if stem:
        cands.append(Path(csv1).parent / f"{stem}.csv")
    return next((p for p in cands if p.is_file()), None)


# ---------------------------------------------------------------- 기준을 벗어난 1초의 까닭 → 어느 지표
_WHY = [(re.compile(r"^연결 끊김"), "link"), (re.compile(r"^YAW 수신"), "fb0"), (re.compile(r"^PITCH 수신"), "fb1"),
        (re.compile(r"^YAW 추종"), "tr0"), (re.compile(r"^PITCH 추종"), "tr1"), (re.compile(r"^왕복"), "rtt"),
        (re.compile(r"^줌 fps"), "fps0"), (re.compile(r"^열화상 fps"), "fps1")]
CAT_NAME = {"link": "연결 끊김", "fb0": "YAW 위치 신호 수", "fb1": "PITCH 위치 신호 수", "tr0": "YAW 따라가는 오차",
            "tr1": "PITCH 따라가는 오차", "rtt": "통신 왕복 시간", "fps0": "줌 초당 프레임", "fps1": "열화상 초당 프레임",
            "other": "그 밖"}


def cats(why: str) -> set[str]:
    out = set()
    for part in (why or "").split(" · "):
        part = part.strip()
        if not part:
            continue
        out.add(next((c for rx, c in _WHY if rx.search(part)), "other"))
    return out


def plain_why(why: str) -> str:
    """CSV 의 까닭 글 — '수신' · '추종' 을 보고서 말로."""
    t = why or ""
    t = re.sub(r"(YAW|PITCH) 수신 (\d+)", r"\1 위치 신호 \2번", t)
    t = re.sub(r"(YAW|PITCH) 추종 ", r"\1 따라가는 오차 ", t)
    t = re.sub(r"왕복 무응답 (\d+)", r"확인 신호 응답 없음 \1번", t)
    t = re.sub(r"(줌|열화상) fps (\d+)", r"\1 \2장/초", t)
    return t


# ---------------------------------------------------------------- 공회전 한 건
@dataclass
class Soak:
    j: dict                                   # soak.json
    rows: dict | None                         # soak.csv (없으면 None)
    mdir: Path
    state: str = ""                           # running · done · stale
    base_s: int = 0                           # 기준 구간 [s]
    extent: int = 0                           # 가로축 길이 [s] — 계획 시간(도는 중 · 멈춘 것도 끝까지 보이게)
    last: int = -1                            # 마지막 줄의 경과 [s]
    stale_s: float = 0.0
    out_rows: list = field(default_factory=list)     # [(sec, 시각, 까닭)] — soak.csv 전부(옛 셈 — 보이기만 한다)
    after: dict = field(default_factory=dict)        # 기준 구간 뒤 다시 센 값
    jd: "Judge | None" = None                        # 판정 v2 — judge()
    cur: dict | None = None                          # soak_cur.csv — 두 축 전류(없으면 None · 판정과 무관)

    @property
    def verdict(self) -> str:
        if self.state == "running":
            return "공회전 중"
        if self.state == "stale":
            return "기록 멈춤"
        return (self.jd.verdict if self.jd else "") or self.j.get("verdict") or "—"


def load(mdir: Path | None) -> Soak | None:
    if not mdir:
        return None
    mdir = Path(mdir)
    j = read_json(mdir / "soak.json")
    if not j:
        return None
    rows = read_rows(mdir / (j.get("csv") or "soak.csv"))
    s = Soak(j=j, rows=rows, mdir=mdir)
    s.cur = read_cur(mdir / (j.get("curCsv") or "soak_cur.csv"))
    s.state = "done" if j.get("state") == "done" else "running"
    if s.state == "running":
        mt = [p.stat().st_mtime for p in (mdir / "soak.json", mdir / "soak.csv") if p.exists()]
        s.stale_s = time.time() - max(mt) if mt else 0.0
        if s.stale_s > STALE_S:
            s.state = "stale"
    lim = j.get("limits") or {}
    base = j.get("baseline") or {}
    s.base_s = int(base.get("seconds") or lim.get("baselineS") or 0)
    if rows and rows["sec"]:
        s.last = max(rows["sec"])
    s.extent = int(max(float(j.get("plannedS") or 0), s.last + 1, 1))
    _recount(s)
    s.jd = judge(s)
    return s


def _recount(s: Soak) -> None:
    """한계와 견줄 값을 soak.csv 에서 다시 센다 — 따라가는 오차 · 카메라 fps 는 기준 구간 뒤만(한계가 그때부터 걸린다)."""
    r = s.rows
    a = {"trAfter": [None, None], "trAfterAt": [None, None], "trBase": [None, None],
         "fpsAfter": [None, None], "fpsZero": [0, 0], "fbMin": [None, None], "fbMinAt": [None, None],
         "rttMax": None, "rttMaxAt": None, "rttP99": None, "lost": 0, "link": 0, "rows": 0}
    s.after = a
    if not r:
        return
    a["rows"] = len(r["sec"])
    rtts = []
    for i, sec in enumerate(r["sec"]):
        for ax in (0, 1):
            fb = r[f"fb{ax}"][i]
            if fb is not None and (a["fbMin"][ax] is None or fb < a["fbMin"][ax]):
                a["fbMin"][ax], a["fbMinAt"][ax] = fb, sec
            tr = r[f"tr{ax}"][i]
            if tr is not None:
                if sec >= s.base_s:
                    if a["trAfter"][ax] is None or tr > a["trAfter"][ax]:
                        a["trAfter"][ax], a["trAfterAt"][ax] = tr, sec
                elif a["trBase"][ax] is None or tr > a["trBase"][ax]:
                    a["trBase"][ax] = tr
            fp = r[f"fps{ax}"][i]
            if fp is not None:
                if fp == 0:
                    a["fpsZero"][ax] += 1
                if sec >= s.base_s and (a["fpsAfter"][ax] is None or fp < a["fpsAfter"][ax]):
                    a["fpsAfter"][ax] = fp
        rt = r["rtt"][i]
        if rt is not None:
            rtts.append(rt)
            if a["rttMax"] is None or rt > a["rttMax"]:
                a["rttMax"], a["rttMaxAt"] = rt, sec
        a["lost"] += r["lost"][i] or 0
        why = r["why"][i]
        if why:
            if "link" in cats(why):
                a["link"] += 1
            s.out_rows.append((sec, r["t"][i], why))
    if rtts:
        rtts.sort()
        a["rttP99"] = rtts[min(len(rtts) - 1, int(math.ceil(0.99 * len(rtts))) - 1)]


# ---------------------------------------------------------------- 판정 v2 — 1초 분류와 판정
V2_STATS = ("latencyS", "trackHardS", "trackHardRunMax", "lostRunMax", "fbLateS", "rttBaseMs")   # soak.json stats 새 칸(rule 2 · fbLateS 4)
V2_NAME = {"latencyS": "통신 지연(초)", "trackHardS": "늘린 한계를 넘은 따라가는 오차(초)", "fbLateS": "위치 신호 밀림(초)",
           "trackHardRunMax": "그 가장 긴 연속(초)", "lostRunMax": "응답 없음 가장 긴 연속(초)", "rttBaseMs": "평소 왕복(ms)"}
_WARNED: set = set()                  # 같은 경고를 한 번만(내보내기 한 번에 load 가 여러 번 불린다)


@dataclass
class Judge:
    """공회전 판정 v2 — judge() 가 만든다. 표지(marks · n 의 열쇠): lat 통신 지연 · hard0/1 따라가는 오차가 지연 몫을
    더한 한계도 넘음(진짜) · soft0/1 한계는 넘었지만 지연 몫 안 · lost 확인 신호 답 없음 · fb0/1 위치 신호 부족 ·
    late0/1 위치 신호 밀림(모자랐지만 다음 초에 몰려 옴 — 통신 탓) · cam0/1 카메라 프레임 부족 · link 연결 끊김. REAL 에 든 것이 진짜 이상(FAIL 요소)이다. 까닭은 판정 순서대로
    FAIL · 끝까지 못 돎 · WARN."""
    verdict: str = ""                                # PASS · WARN · FAIL · 중지
    fail: list = field(default_factory=list)         # FAIL 까닭(사람 말)
    warn: list = field(default_factory=list)         # WARN 까닭
    stop: str = ""                                   # 끝까지 못 돎(— 멈춘 까닭)
    stop_why: str = ""                               # 멈춘 까닭(공회전 탭이 남긴 것)
    n: dict = field(default_factory=dict)            # 표지별 1초 수 + hard(두 축 합친 것) · fb · cam · real
    st: dict = field(default_factory=dict)           # V2_STATS — rule 2 면 soak.json 값
    marks: dict = field(default_factory=dict)        # 경과[s] → 표지 집합(걸린 1초만)
    eff: dict = field(default_factory=dict)          # 경과[s] → 그 1초에 쓴 왕복 rtt_eff [ms](걸린 1초만)
    rtt_base: float = RTT_BASE_DEFAULT_MS            # 평소 왕복 — 기준 구간 가운데값 [ms]
    lost: str = ""                                   # 확인 신호 답 없음의 몫(lost_level) — FAIL · WARN · ''(기록만)
    rtt_max: float | None = None                     # 가장 긴 왕복 [ms]
    lost_n: int = 0                                  # 답 없는 확인 신호 수
    hard_axes: list = field(default_factory=list)    # 따라가는 오차 가장 긴 연속에 든 축
    source: str = ""                                 # soak.csv(다시 셈) · soak.json(rule 2 — C++ 판정) · soak.json v1(다시 못 셈)
    old: str = ""                                    # 옛 기준 판정 — 옛 기록은 soak.json verdict, 새 기록은 verdictV1
    recount: str = ""                                # soak.csv 로 다시 센 판정(rule 2 와 견줄 때)
    diff: list = field(default_factory=list)         # rule 2 인데 다시 센 값과 다른 칸

    @property
    def reasons(self) -> list[str]:
        return self.fail + ([self.stop] if self.stop else []) + self.warn


def _num(x, default: float) -> float:
    try:
        return default if x is None else float(x)
    except (TypeError, ValueError):
        return default


def _pair(x) -> list:
    """soak.json 의 [YAW, PITCH] · [줌, 열화상] 짝 — 없거나 음수(C++ 은 null 로 쓴다)면 None."""
    x = x if isinstance(x, (list, tuple)) else []
    out = []
    for k in (0, 1):
        v = _num(x[k] if k < len(x) else None, -1.0)
        out.append(v if v >= 0 else None)
    return out


def _pct(v: list, p: float) -> float:
    """QcGw::percentile 과 같은 셈 — 정렬 뒤 선형 보간(numpy 기본). 50 이면 가운데값(짝수 개면 가운데 둘의 평균)."""
    if not v:
        return -1.0
    v = sorted(v)
    if p <= 0:
        return v[0]
    if p >= 100:
        return v[-1]
    r = p / 100.0 * (len(v) - 1)
    i = int(math.floor(r))
    f = r - i
    return v[i] + f * (v[i + 1] - v[i]) if i + 1 < len(v) else v[i]


def cams_on(s: Soak) -> list[bool]:
    """보는 카메라 — soak.json cams.<zoom|thermal>.measured. 칸이 없으면(옛 기록) soak.csv 에 값이 있나로."""
    cams = s.j.get("cams")
    if isinstance(cams, dict) and cams:
        return [bool((cams.get(k) or {}).get("measured")) for k in CAM_KEY]
    return [bool(s.rows) and any(v is not None for v in s.rows[f"fps{c}"]) for c in (0, 1)]


def diag_fail(j: dict) -> list[str]:
    """진단 카운터 FAIL 까닭 — 드라이버 폴트(idx 4)가 늘었거나 누적 카운터(idx 0~5 · 9~11)가 시작보다 줄었다(= 도중에
    모터가 다시 켜짐). idx 6~8(STATUS · nFAULT · 복구 I2C)은 상태값이라 줄어듦 검사에서 뺀다. 줄어든 것은 축마다 처음
    하나만 적는다(soaktab.cpp 처럼)."""
    index = {name: i for i, (name, _, _) in enumerate(DIAG)}
    by_axis: dict = {}
    for x in j.get("diag") or []:
        by_axis.setdefault(x.get("axis"), []).append(x)
    out = []
    for ax in AX_NAME:
        items = [(index.get(x.get("name"), k), x) for k, x in enumerate(by_axis.get(ax) or [])]
        for i, x in items:
            s0, e0 = x.get("start"), x.get("end")
            if i == DIAG_FAULT_IDX and s0 is not None and e0 is not None and e0 > s0:
                out.append(f"{ax} 드라이버 폴트 {e0 - s0}번(진단 카운터)")
        for i, x in items:
            s0, e0 = x.get("start"), x.get("end")
            if i in DIAG_STATE_IDX or s0 is None or e0 is None:
                continue
            if e0 < s0:
                out.append(f"{ax} 진단 카운터가 시작보다 줄어듦({x.get('name')} {s0} → {e0}) — 도중에 모터가 다시 켜짐")
                break
    return out


def _names(idx, names=AX_NAME) -> str:
    return " · ".join(names[k] for k in sorted(idx))


def classify(s: Soak) -> Judge:
    """soak.csv → 1초 분류와 판정 v2(개정 1). 줄마다(기준 구간 포함)
      통신 지연 = 왕복 > rttMaxMs · 답 없음 = 무응답 > 0 · 위치 신호 부족 = 수신 < fbMinPerS · 카메라 이상 = 보는 카메라의
      fps == 0 · 연결 끊김 = '이상' 글에 '연결 끊김'
    위치 신호가 모자란 축은 경과가 바로 다음인 줄과 그 축 수신 합이 2 × fbMinPerS 이상이면 밀림(late), 아니면 부족(fb)(규칙 4)
    기준 구간 뒤(경과 ≥ 기준 구간 길이) 줄만 — 기준 구간 값으로 한계를 정하는 두 가지
      카메라 이상에 fps < fpsLimit 을 더하고, 진짜 추종 이상 = 추종 > trackLimit[a] + peakDps[a] × 지연 몫
      지연 몫[s] = max(0, rtt_eff − 평소 왕복) / 1000,  rtt_eff = max(직전 줄 왕복', 그 줄 왕복', 다음 줄 왕복')(없는 쪽은 뺀다)
      왕복' = 그 줄의 왕복. 빈칸이면 직전 5초 안(경과 차 ≤ 5)의 마지막 왕복, 그것도 없으면 평소 왕복
    평소 왕복 = 기준 구간 왕복 값들의 가운데값(빈칸 빼고, 없으면 10 ms). 연달아 = 경과가 1초씩 이어진 줄(기준 구간과
    그 뒤를 한 줄로 잇는다). 직전 줄 · 다음 줄 = 파일에서 바로 앞 · 뒤 줄."""
    j, r = s.j, s.rows
    jd = Judge(source="soak.csv")
    lim, base, sin = j.get("limits") or {}, j.get("baseline") or {}, j.get("sin") or {}
    fbmin = _num(lim.get("fbMinPerS"), 90.0)
    rttmax = _num(lim.get("rttMaxMs"), 100.0)
    tl, fl = _pair(base.get("trackLimit")), _pair(base.get("fpsLimit"))
    pk = [_num((sin.get(k) or {}).get("peakDps"), 0.0) for k in ("yaw", "pitch")]
    on = cams_on(s)
    bs = s.base_s
    n = dict.fromkeys(("lat", "hard0", "hard1", "soft0", "soft1", "lost", "fb0", "fb1", "late0", "late1", "cam0", "cam1", "link",
                       "hard", "fb", "late", "cam", "real"), 0)
    jd.n = n
    rb_vals = [rt for sec, rt in zip(r["sec"], r["rtt"]) if sec < bs and rt is not None]
    rb = _pct(rb_vals, 50.0) if rb_vals else RTT_BASE_DEFAULT_MS
    jd.rtt_base = rb
    real = set(REAL)
    # 왕복' — 빈칸이면 직전 5초 안의 마지막 왕복, 그것도 없으면 평소 왕복. rtt_eff 는 직전 · 그 · 다음 줄 왕복' 가운데 큰 것
    rps, last_rt, last_at = [], None, None
    for sec, rt in zip(r["sec"], r["rtt"]):
        if rt is not None:
            rps.append(rt)
            last_rt, last_at = rt, sec
        elif last_rt is not None and sec - last_at <= RTT_FILL_S:
            rps.append(last_rt)
        else:
            rps.append(rb)
    nrows = len(r["sec"])
    hard_run, hard_prev, hard_axes, best_hard = 0, None, set(), 0
    lost_run, lost_prev, best_lost = 0, None, 0
    for i, sec in enumerate(r["sec"]):
        rt = r["rtt"][i]
        eff = max(rps[max(0, i - 1):i + 2])
        nxt = i + 1 if i + 1 < nrows and r["sec"][i + 1] == sec + 1 else None   # 경과가 바로 다음인 줄
        if rt is not None and (jd.rtt_max is None or rt > jd.rtt_max):
            jd.rtt_max = rt
        after = sec >= bs                              # 기준 구간 뒤 — 추종 · fps 비율도 본다
        share = max(0.0, eff - rb) / 1000.0
        m = set()
        if rt is not None and rt > rttmax:
            m.add("lat")
        for a in (0, 1):
            tr = r[f"tr{a}"][i]
            if after and tr is not None and tl[a] is not None:
                if tr > tl[a] + pk[a] * share:
                    m.add(f"hard{a}")
                elif tr > tl[a]:
                    m.add(f"soft{a}")
            fb = r[f"fb{a}"][i]
            if fb is not None and fb < fbmin:
                nf = r[f"fb{a}"][nxt] if nxt is not None else None
                m.add(f"late{a}" if nf is not None and fb + nf >= 2 * fbmin else f"fb{a}")
        for c in (0, 1):
            fp = r[f"fps{c}"][i]
            if on[c] and fp is not None and (fp == 0 or (after and fl[c] is not None and fp < fl[c])):
                m.add(f"cam{c}")
        lost = r["lost"][i] or 0
        if lost > 0:
            m.add("lost")
            jd.lost_n += lost
        if r["why"][i] and "link" in cats(r["why"][i]):
            m.add("link")
        if not m:
            continue
        jd.marks[sec] = m
        jd.eff[sec] = eff
        for k in m:
            n[k] += 1
        now = {a for a in (0, 1) if f"hard{a}" in m}
        if now:
            n["hard"] += 1
            if hard_prev is not None and sec == hard_prev + 1:
                hard_run, hard_axes = hard_run + 1, hard_axes | now
            else:
                hard_run, hard_axes = 1, set(now)
            hard_prev = sec
            if hard_run > best_hard:
                best_hard, jd.hard_axes = hard_run, sorted(hard_axes)
        # 답 없음 연달아 = 사이에 답이 온 확인 신호 없이 이어진 답 없음 초. 빠진 줄도 끊는다(v3 — soakcore.cpp 와 같다)
        if lost_prev is None or sec != lost_prev + 1:
            lost_run = 0
        lost_prev = sec
        if "lost" in m:
            lost_run = lost_run + 1 if lost_run > 0 else 1
            best_lost = max(best_lost, lost_run)
        elif rt is not None:
            lost_run = 0
        n["fb"] += bool(m & {"fb0", "fb1"})
        n["late"] += bool(m & {"late0", "late1"})
        n["cam"] += bool(m & {"cam0", "cam1"})
        n["real"] += bool(m & real)
    jd.st = {"latencyS": n["lat"], "trackHardS": n["hard"], "trackHardRunMax": best_hard, "lostRunMax": best_lost,
             "fbLateS": n["late"], "rttBaseMs": round(rb, 1)}

    # 판정 — 위에서부터 먼저 걸리는 것(개정 1): FAIL · 중지 · WARN · PASS. WARN 까닭은 중지여도 적는다
    fail, warn = jd.fail, jd.warn
    if j.get("fault"):
        fail.append(f"드라이버 폴트 — {j['fault']}")
    slept = _num(j.get("sleptS"), 0.0)
    if slept > 0:
        fail.append(f"PC 가 {slept:.0f}초 잠듦 — 그동안 기록 없음")
    fail += diag_fail(j)
    if n["link"]:
        fail.append(f"게이트웨이 연결 끊김 {n['link']}초")
    if n["fb"]:
        fail.append(f"{_names([a for a in (0, 1) if n[f'fb{a}']])} 위치 신호 부족 {n['fb']}초(1초에 {fbmin:g}번보다 적음)")
    if n["cam"]:
        fail.append(f"카메라 초당 프레임 부족 {n['cam']}초({_names([c for c in (0, 1) if n[f'cam{c}']], CAM_NAME)})")
    if best_hard >= RUN_FAIL_S:
        fail.append(f"{_names(jd.hard_axes)} 따라가는 오차가 늘린 한계도 넘음 {best_hard}초 연속")
    jd.lost = lost_level(n["lost"], best_lost, nrows)
    lim_txt = f"이 공회전 한도 {lost_fail_count(nrows):.1f}번 — 24시간에 {LOST_FAIL_PER_DAY:.0f}번"
    if jd.lost == "FAIL":
        fail.append(f"확인 신호 답 없음 {best_lost}초 연속" if best_lost >= LOST_FAIL_RUN_S
                    else f"확인 신호 답 없음 {n['lost']}번({lim_txt})")
    if LATENCY_IS_WARN and n["lat"]:
        warn.append(f"통신 지연 {n['lat']}초(왕복 최대 {jd.rtt_max:.0f} ms) — 제품 고장 아님, 게이트웨이 통신이 밀림")
    if n["late"]:
        warn.append(f"{_names([a for a in (0, 1) if n[f'late{a}']])} 위치 신호 밀림 {n['late']}초(1초에 {fbmin:g}번보다 적었지만 "
                    f"다음 1초에 몰려 와 두 초 합은 {2 * fbmin:g}번 이상) — 제품 고장 아님, 게이트웨이 통신이 밀림")
    if n["hard"] and best_hard < RUN_FAIL_S:
        k = n["hard"]
        warn.append(f"{_names([a for a in (0, 1) if n[f'hard{a}']])} 따라가는 오차가 늘린 한계도 넘음 {k}초"
                    + ("" if k == 1 else "(따로따로)" if best_hard == 1 else f"(가장 길게 {best_hard}초 연속)"))
    if jd.lost == "WARN":
        warn.append(f"확인 신호 답 없음 {best_lost}초 연속" if best_lost >= LOST_WARN_RUN_S
                    else f"확인 신호 답 없음 {n['lost']}번({lim_txt} 의 2/3 이상)")
    completed = bool(j.get("completed"))
    if s.state == "done" and not completed:
        why = next((str(x) for x in reversed(j.get("reasons") or []) if x and not str(x).startswith("이상값")), "")
        jd.stop_why = why if why not in ("끝까지 못 감", "끝까지 못 돎", "끝까지 못 돌았음") else ""
        jd.stop = "끝까지 못 돌았음" + (f" — {jd.stop_why}" if jd.stop_why else "")
    jd.verdict = ("FAIL" if fail else "중지" if s.state == "done" and not completed else "WARN" if warn else "PASS")
    return jd


def judge(s: Soak) -> Judge:
    """공회전 판정 v2. soak.json 에 "rule": 2 가 있으면(새 실행) C++ 이 낸 판정과 칸(stats 새 칸)을 쓰고, soak.csv 로 다시
    센 값과 다르면 stderr 에 알린다. 없으면(옛 기록) soak.csv 로 다시 센다 — 옛 판정은 old 에 남긴다. soak.csv 가 없으면
    다시 셀 수 없어 soak.json 판정을 그대로 쓴다(source 'soak.json v1' 이면 옛 기준 판정이다)."""
    j = s.j
    rule2 = j.get("rule") == RULE
    st = j.get("stats") or {}
    if s.rows is None:
        jd = Judge(source="soak.json" if rule2 else "soak.json v1", verdict=str(j.get("verdict") or ""),
                   old=str(j.get("verdictV1") or "") if rule2 else "")
        jd.st = {k: st[k] for k in V2_STATS if st.get(k) is not None}
        jd.fail = [str(x) for x in j.get("reasons") or [] if x]
        return jd
    jd = classify(s)
    jd.recount = jd.verdict
    if not rule2:
        jd.old = str(j.get("verdict") or "")
        return jd
    jd.source = "soak.json"
    jd.old = str(j.get("verdictV1") or "")
    if s.state != "done":
        return jd                     # 도는 중 — soak.json 은 60초마다라 soak.csv 와 견주지 않는다
    mine = dict(jd.st)
    for k in V2_STATS:
        if st.get(k) is None:
            continue
        jd.st[k] = st[k]
        tol = 0.15 if k == "rttBaseMs" else 0.0       # 평소 왕복은 soak.csv 가 0.1 ms 로 반올림해 적는다
        if mine.get(k) is None or abs(_num(st[k], 0.0) - _num(mine[k], 0.0)) > tol:
            jd.diff.append(f"{V2_NAME.get(k, k)} 공회전 탭 {st[k]}, 다시 셈 {mine.get(k)}")
    if j.get("verdict"):
        if j["verdict"] != jd.verdict:
            jd.diff.insert(0, f"판정 공회전 탭 {j['verdict']}, 다시 셈 {jd.verdict}")
        jd.verdict = str(j["verdict"])
    key = (str(s.mdir), tuple(jd.diff))
    if jd.diff and key not in _WARNED:
        _WARNED.add(key)
        print(f"공회전 판정 v2 — {s.mdir.name}: soak.json(공회전 탭) 값과 soak.csv 로 다시 센 값이 다름 — "
              + " · ".join(jd.diff), file=sys.stderr)
    return jd


# ---------------------------------------------------------------- 그래프 — 묶기
def bucket_size(extent: int) -> int:
    return next((b for b in BUCKETS if math.ceil(extent / b) <= MAX_POINTS), BUCKETS[-1])


def aggregate(secs: list, vals: list, b: int, n: int):
    mn, mx, sm, ct = [None] * n, [None] * n, [0.0] * n, [0] * n
    for s, v in zip(secs, vals):
        if v is None:
            continue
        k = s // b
        if k < 0 or k >= n:
            continue
        if ct[k] == 0:
            mn[k] = mx[k] = v
        else:
            if v < mn[k]:
                mn[k] = v
            if v > mx[k]:
                mx[k] = v
        sm[k] += v
        ct[k] += 1
    av = [sm[k] / ct[k] if ct[k] else None for k in range(n)]
    return mn, av, mx


def nice_step(span: float, count: int = 4) -> float:
    raw = span / max(1, count)
    mag = 10 ** math.floor(math.log10(raw)) if raw > 0 else 1
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            return m * mag
    return 10 * mag


def nice_top(v: float) -> float:
    st = nice_step(v, 4)
    return math.ceil(v / st - 1e-9) * st if st else v


X_STEPS = (10, 30, 60, 120, 300, 600, 900, 1800, 3600, 7200, 10800, 21600)


def x_ticks(extent: int) -> list[tuple[int, str]]:
    step = next((s for s in X_STEPS if extent / s <= 7), X_STEPS[-1])
    out = []
    t = 0
    while t <= extent + 1e-9:
        if step >= 3600:
            lab = f"{t // 3600}시간" if t else "0"
        elif step >= 60:
            lab = f"{t // 60}분" if t else "0"
        else:
            lab = f"{t}초" if t else "0"
        out.append((t, lab))
        t += step
    return out


@dataclass
class Panel:
    key: str            # fb0 · fb1 · tr0 · tr1 · rtt · fps0 · fps1
    group: str          # fb · tr · rtt · fps
    who: str            # YAW · PITCH · 줌 · 열화상 · 게이트웨이 · YAW · PITCH
    unit: str
    nd: int
    lo: float | None = None       # 이보다 작으면 이상
    hi: float | None = None       # 이보다 크면 이상
    lim_from: int = 0             # 한계가 걸리기 시작하는 경과 [s]
    worst: str = ""               # 머리글 옆 한 마디


GROUPS = [
    ("fb", "위치 신호 수", "번/초",
     "1초에 받은 위치 신호(모터가 지금 각도를 알려 주는 신호) 수. 모터는 1초에 100번 보낸다."),
    ("tr", "따라가는 오차", "°",
     "사인파로 움직이라고 명령한 각도와 실제 각도의 차이. 1초마다 그사이 가장 큰 값을 적는다. 기구마다 크기가 달라서 한계는 "
     "제품마다 기준 구간에서 정한다."),
    ("rtt", "통신 왕복 시간", "ms",
     "확인 신호(CAN_CHECK)를 PC 에서 보드로 보내 응답이 돌아오기까지 걸린 시간. 1초마다 그사이 가장 긴 값을 적는다. "
     "1초에 4번쯤 게이트웨이 · YAW · PITCH 에 돌아가며 보낸다."),
    ("fps", "카메라 초당 프레임", "장/초",
     "1초에 받은 영상 장수. 한계는 제품마다 기준 구간에서 정하고, 0장이면 언제나 기준을 벗어난다."),
    ("cur", "모터 전류", "A",
     "모터에 물어 받은 전류 크기다(10-08 부터는 0.1초마다, 그 전에는 1초마다 물었다). 판정에는 쓰지 않고, 공회전하는 동안 "
     "전류가 점점 오르는지 보려고 남긴다. 같은 움직임에 전류가 더 들면 기구가 뻑뻑해졌거나 모터가 달아오른 것일 수 있다. "
     "그래프 위 한 줄의 '앞쪽'은 사인파로 들어가는 처음 30초를 빼고 잡는다. PITCH 그래프의 점선은 판정 한계가 아니라 공회전 "
     "동안만 걸어 둔 임시 전류 상한이다(걸었을 때만 그린다). 모터를 보호하려고 공회전 동안만 낮추고, 검사가 끝나면 풀며 모터 "
     "전원을 다시 넣어도 풀린다."),
]


GROUP_TITLE = {g[0]: g[1] for g in GROUPS}
# 그래프 눈금 — 패널마다 (통신 지연 탓 표지, 진짜 이상 표지). 연결이 끊긴 1초는 모든 패널에 빨강
PANEL_MARKS = {"fb0": ({"late0"}, {"fb0", "link"}), "fb1": ({"late1"}, {"fb1", "link"}),
               "tr0": ({"soft0"}, {"hard0", "link"}), "tr1": ({"soft1"}, {"hard1", "link"}),
               "rtt": ({"lat"}, {"lost", "link"}), "fps0": (set(), {"cam0", "link"}), "fps1": (set(), {"cam1", "link"}),
               "cur0": (set(), {"link"}), "cur1": (set(), {"link"})}


def _tick(v: float) -> str:
    return (f"{v:.2f}".rstrip("0").rstrip(".") or "0").replace("-", "−")


def panels(s: Soak) -> list[Panel]:
    j = s.j
    lim = j.get("limits") or {}
    base = j.get("baseline") or {}
    cams = j.get("cams") or {}
    fbmin = float(lim.get("fbMinPerS") or 90)
    rttmax = float(lim.get("rttMaxMs") or 100)
    tl = base.get("trackLimit") or [None, None]
    fl = base.get("fpsLimit") or [None, None]
    ready = bool(base.get("ready"))
    out = [Panel(f"fb{a}", "fb", AX_NAME[a], "번/초", 0, lo=fbmin) for a in (0, 1)]
    out += [Panel(f"tr{a}", "tr", AX_NAME[a], "°", 2, hi=(tl[a] if ready and a < len(tl) else None), lim_from=s.base_s)
            for a in (0, 1)]
    out.append(Panel("rtt", "rtt", "게이트웨이 · YAW · PITCH", "ms", 1, hi=rttmax))
    for c in (0, 1):
        measured = (cams.get(CAM_KEY[c]) or {}).get("measured")
        have = s.rows is not None and any(v is not None for v in s.rows[f"fps{c}"])
        if measured or have:
            out.append(Panel(f"fps{c}", "fps", CAM_NAME[c], "장/초", 0,
                             lo=(fl[c] if ready and c < len(fl) and fl[c] not in (None, -1) else None), lim_from=s.base_s))
    cap = cur_cap(j)
    for a in (0, 1):                                  # 전류 — 잰 회차만(판정과 무관). PITCH 는 임시 상한을 걸었으면 그 선
        if s.cur and any(v is not None for v in s.cur[f"cur{a}"]):
            out.append(Panel(f"cur{a}", "cur", AX_NAME[a], "A", 2, hi=(cap["pitchA"] if a == 1 and cap and cap["on"] else None)))
    return out


def cur_cap(j: dict | None) -> dict | None:
    """soak.json curCap(10-08 부터) — 공회전 동안 PITCH 에 건 임시 전류 상한. 없으면(옛 기록) None."""
    c = (j or {}).get("curCap")
    if not isinstance(c, dict):
        return None
    return {"pitchA": _num(c.get("pitchA"), 3.0), "state": str(c.get("state") or ""), "on": c.get("state") == "on",
            "atCapS": _num(c.get("atCapS"), 0.0), "ratio": _num(c.get("hitRatio"), 0.95)}


def robot_info(mdir: Path | None) -> dict | None:
    """로봇 앱 점검(qc/03) — 검사 폴더 run.json 의 robot 칸. QC PC 검사면 None."""
    r = (read_json(Path(mdir) / "run.json") or {}).get("robot") if mdir else None
    return r if isinstance(r, dict) else None


def _y(v: float, lo: float, hi: float) -> float:
    return round(100.0 * (1.0 - (min(max(v, lo), hi) - lo) / (hi - lo)), 2)


def _runs(n: int, ok) -> list[tuple[int, int]]:
    """None 이 아닌 칸이 이어진 구간들 [(첫, 끝)]."""
    out, st = [], None
    for k in range(n):
        if ok(k):
            if st is None:
                st = k
        elif st is not None:
            out.append((st, k - 1))
            st = None
    if st is not None:
        out.append((st, n - 1))
    return out


def chart_panel(p: Panel, s: Soak, b: int, n: int, data: dict) -> str:
    """패널 하나 — SVG(늘여 그리는 판) 위에 글자는 HTML 로 얹는다(폰에서도 글자가 작아지지 않게)."""
    r = s.cur if p.group == "cur" else s.rows      # 전류는 따로 적은 파일(soak_cur.csv)에서
    mn, av, mx = aggregate(r["sec"], r[p.key], b, n)
    # 눈금 — 이 지표가 걸린 1초(판정 v2). 칸마다 통신 지연 탓 · 진짜 이상 수. 둘 다면 진짜 이상으로 센다
    lat_keys, real_keys = PANEL_MARKS[p.key]
    latk: dict[int, int] = {}
    realk: dict[int, int] = {}
    for sec, m in (s.jd.marks.items() if s.jd else ()):
        k = sec // b
        if not 0 <= k < n:
            continue
        if m & real_keys:
            realk[k] = realk.get(k, 0) + 1
        elif m & lat_keys:
            latk[k] = latk.get(k, 0) + 1
    vals_after = [v for sec, v in zip(r["sec"], r[p.key]) if v is not None and sec >= p.lim_from]
    vmax_all = max((v for v in mx if v is not None), default=0.0)
    vmax_after = max(vals_after, default=vmax_all) if p.group == "tr" else vmax_all
    # 세로축 — 0 부터. 한계가 보이게, 기준 구간의 튐(사인파에 들어가는 순간)이 나머지를 눌러 버리지 않게
    if p.group == "fb":
        top = 110.0 if vmax_all <= 105 else nice_top(vmax_all * 1.05)
    elif p.group == "tr":
        want = max(vmax_after * 1.15, (p.hi or 0) * 1.3, 0.1)
        top = nice_top(want)
    elif p.group == "rtt":
        top = nice_top(min(max(vmax_all * 1.1, (p.hi or 0) * 1.2), (p.hi or 100) * 3))
    elif p.group == "cur":
        top = nice_top(max(vmax_all * 1.15, 0.5))
    else:
        top = nice_top(max(vmax_all * 1.15, (p.lo or 0) * 1.3, 5))
    lo_y = 0.0
    over = [(k, mx[k]) for k in range(n) if mx[k] is not None and mx[k] > top]
    xs = lambda k: round(k + 0.5, 1)  # noqa: E731

    parts = []
    # 기준 구간 · 아직 안 돈(멈춘 뒤) 곳
    if s.base_s and p.group in ("tr", "fps"):
        parts.append(f'<rect class="sk-base" x="0" y="0" width="{min(n, s.base_s / b):.2f}" height="100"/>')
    done_x = (s.last + 1) / b if s.last >= 0 else 0
    if done_x < n - 0.01:
        parts.append(f'<rect class="sk-rest" x="{done_x:.2f}" y="0" width="{n - done_x:.2f}" height="100"/>')
    # 눈금 줄
    st = nice_step(top - lo_y, 4)
    yt = []
    v = lo_y
    while v <= top + 1e-9:
        yt.append(v)
        v = round(v + st, 10)
    for tv in yt:
        parts.append(f'<line class="sk-grid" x1="0" x2="{n}" y1="{_y(tv, lo_y, top)}" y2="{_y(tv, lo_y, top)}"/>')
    # 띠(최소~최대)와 평균 선 — 칸이 1초면 띠가 선과 같아 그리지 않는다
    if b > 1:
        for a0, a1 in _runs(n, lambda k: mn[k] is not None):
            up = " ".join(f"{xs(k)},{_y(mx[k], lo_y, top)}" for k in range(a0, a1 + 1))
            dn = " ".join(f"{xs(k)},{_y(mn[k], lo_y, top)}" for k in range(a1, a0 - 1, -1))
            if a0 == a1:
                up = f"{a0 + 0.1},{_y(mx[a0], lo_y, top)} {a0 + 0.9},{_y(mx[a0], lo_y, top)}"
                dn = f"{a0 + 0.9},{_y(mn[a0], lo_y, top)} {a0 + 0.1},{_y(mn[a0], lo_y, top)}"
            parts.append(f'<polygon class="sk-band" points="{up} {dn}"/>')
    line = []
    for a0, a1 in _runs(n, lambda k: av[k] is not None):
        pts = [f"{xs(k)},{_y(av[k], lo_y, top)}" for k in range(a0, a1 + 1)]
        if len(pts) == 1:
            pts = [f"{a0 + 0.1},{_y(av[a0], lo_y, top)}", f"{a0 + 0.9},{_y(av[a0], lo_y, top)}"]
        line.append("M" + " L".join(pts))
    if line:
        parts.append(f'<path class="sk-line" d="{" ".join(line)}"/>')
    # 한계 — 점선. 기준 구간 뒤에만 걸리는 한계는 거기서부터
    lim = p.lo if p.lo is not None else p.hi
    labels = []
    if lim is not None and lo_y <= lim <= top:
        x0 = min(n, p.lim_from / b) if p.lim_from else 0
        yl = _y(lim, lo_y, top)
        parts.append(f'<line class="sk-lim" x1="{x0:.2f}" x2="{n}" y1="{yl}" y2="{yl}"/>')
        side = "lo" if p.lo is not None else "hi"
        labels.append(f'<span class="sk-limlab {side}" style="top:{yl}%">{"임시 상한" if p.group == "cur" else "한계"} {_tick(lim)}</span>')
    # 눈금(아래쪽) — 통신 지연 탓은 짧은 주황, 진짜 이상은 긴 빨강(색만으로 가르지 않게 길이도 다르다)
    lat_only = sorted(k for k in latk if k not in realk)
    if lat_only:
        parts.append('<path class="sk-lat" d="' + " ".join(f"M{xs(k)} 100 V90" for k in lat_only) + '"/>')
    if realk:
        parts.append('<path class="sk-out" d="' + " ".join(f"M{xs(k)} 100 V78" for k in sorted(realk)) + '"/>')
    # 지금(도는 중)
    if s.state in ("running", "stale") and s.last >= 0:
        parts.append(f'<line class="sk-now" x1="{done_x:.2f}" x2="{done_x:.2f}" y1="0" y2="100"/>')
        labels.append(f'<span class="sk-nowlab" style="left:{100 * done_x / n:.2f}%">지금</span>')
    if s.base_s and p.group in ("tr", "fps") and s.base_s / b >= n * 0.06:
        labels.append('<span class="sk-baselab">기준 구간</span>')
    ylabs = "".join(f'<span style="top:{_y(tv, lo_y, top)}%">{_tick(tv)}</span>' for tv in yt)
    xlabs = "".join(f'<span style="left:{100 * t / s.extent:.2f}%">{esc(lab)}</span>' for t, lab in x_ticks(s.extent))
    # 머리글 한 마디 — 가장 나쁜 값
    a = s.after
    ax = int(p.key[-1]) if p.key[-1].isdigit() else 0
    if p.group == "fb":
        worst = f"가장 적을 때 {fx(a['fbMin'][ax], 0)}번"
    elif p.group == "tr":
        worst = f"가장 클 때 {fx(a['trAfter'][ax], 2, '°')}" + (" (기준 구간 뒤)" if s.base_s else "")
    elif p.group == "rtt":
        worst = f"가장 긴 것 {fx(a['rttMax'], 1, ' ms')}" + (f" · 응답 없음 {a['lost']}번" if a["lost"] else "")
    elif p.group == "cur":
        t = cur_trend(s.cur, ax, s.extent) or {}
        worst = f"가장 큰 값 {fx(t.get('max'), 2, ' A')}"
        if t.get("last") is not None:
            worst += (f" · 앞쪽 {dur(t['win'])} 최고 {fx(t['first'], 2, ' A')} → 끝쪽 {dur(t['win'])} 최고 {fx(t['last'], 2, ' A')}"
                      + (f"(앞쪽보다 {'+' if t['pct'] >= 0 else '−'}{abs(t['pct']):.0f}%)" if t.get("pct") is not None else ""))
    else:
        worst = f"가장 적을 때 {fx(a['fpsAfter'][ax], 0)}장" + (" (기준 구간 뒤)" if s.base_s else "")
    note = ""
    if over:
        k, v = max(over, key=lambda x: x[1])
        note = (f'<p class="sk-over">위쪽 끝에 닿아 잘린 곳은 실제로 더 큰 값이다 — 가장 큰 것 {fx(v, p.nd if p.group != "tr" else 2)}'
                f'{" " + p.unit if p.unit != "°" else "°"} (시작부터 {dur(k * b)} 즈음'
                + (", 사인파에 들어가는 순간" if p.group == "tr" and k * b < max(s.base_s, 1) else "") + ")</p>")
    rnd = (lambda x: None if x is None else round(x, p.nd))
    rnd2 = (lambda x: None if x is None else round(x, p.nd + 1))
    data[p.key] = {"l": f"{p.who} {GROUP_TITLE[p.group]}", "u": p.unit, "d": p.nd,
                   "mn": [rnd(x) for x in mn], "av": [rnd2(x) for x in av], "mx": [rnd(x) for x in mx],
                   "o": {str(k): [latk.get(k, 0), realk.get(k, 0)] for k in sorted(set(latk) | set(realk))}}
    aria = f"{p.who} {GROUP_TITLE[p.group]} 시간 그래프 — {worst}"
    return (f'<div class="sk-panel"><p class="sk-ph"><b>{esc(p.who)}</b> <span>{esc(worst)}</span></p>'
            f'<div class="sk-plot" data-k="{p.key}" tabindex="0" role="img" aria-label="{esc(aria)}">'
            f'<div class="sk-yax" aria-hidden="true">{ylabs}</div>'
            f'<div class="sk-area"><svg class="sk-svg" viewBox="0 0 {n} 100" preserveAspectRatio="none" aria-hidden="true">'
            + "".join(parts) + f'</svg>{"".join(labels)}<div class="sk-cross" hidden></div></div>'
            f'<div class="sk-xax" aria-hidden="true">{xlabs}</div></div>{note}</div>')


def charts_html(s: Soak) -> str:
    if not s.rows or not s.rows["sec"]:
        return ""
    b = bucket_size(s.extent)
    n = int(math.ceil(s.extent / b))
    data: dict = {}
    ps = panels(s)
    groups = []
    for g, title, unit, desc in GROUPS:
        mine = [p for p in ps if p.group == g]
        if not mine:
            continue
        body = "".join(chart_panel(p, s, b, n, data) for p in mine)
        groups.append(f'<div class="sk-group"><p class="sk-gt"><b>{esc(title)}</b> <span class="muted">[{esc(unit)}]</span></p>'
                      f'<p class="sk-gd">{esc(desc)}</p><div class="sk-grid-{len(mine)}">{body}</div></div>')
    base_t = f"기준 구간(처음 {dur(s.base_s)})" if s.base_s else "기준 구간"
    if b == 1:
        how = "1초마다 잰 값을 그대로 이었다(진한 선)."
    else:
        how = (f"{dur(b)}씩 묶어 그렸다. 진한 선은 그 {dur(b)}의 평균, 옅은 띠는 그 {dur(b)} 안에서 가장 작은 값부터 "
               "가장 큰 값까지다.")
    guide = (f"가로축은 공회전을 시작하고 지난 시간이다. {how} 점선은 한계다(모터 전류의 점선만 임시 상한). 아래쪽 눈금은 "
             "기준을 벗어난 때가 있던 자리로, 짧은 주황 눈금은 통신이 밀려서 벗어난 것(제품 고장 아님), 긴 빨간 눈금은 통신 지연 몫을 "
             "빼고도 남은 진짜 이상이다. 따라가는 오차의 점선은 통신이 밀리지 않았을 때의 한계라서, 통신이 밀린 때는 점선을 넘어도 "
             f"주황 눈금일 수 있다. 따라가는 오차와 카메라 초당 프레임 그래프의 앞쪽 회색 칸은 {base_t}으로, 이 두 항목의 한계를 이 "
             "구간에서 정한다. 그래서 이 구간에서는 따라가는 오차가 큰 것과 프레임이 기준보다 적은 것은 판정하지 않는다(프레임 0장은 본다).")
    if s.last + 1 < s.extent:
        guide += (" 오른쪽 옅은 칸은 아직 돌지 않은 시간이다." if s.state in ("running", "stale")
                  else " 오른쪽 옅은 칸은 멈춘 뒤라 기록이 없는 시간이다.")
    key = ('<p class="sk-key" aria-hidden="true"><span><i class="k-line"></i>평균</span>'
           + ('<span><i class="k-band"></i>가장 작은 값 ~ 가장 큰 값</span>' if b > 1 else "")
           + '<span><i class="k-lim"></i>한계</span><span><i class="k-lat"></i>통신 지연 탓(제품 고장 아님)</span>'
           '<span><i class="k-out"></i>진짜 이상</span><span><i class="k-base"></i>기준 구간</span></p>')
    blob = json.dumps({"b": b, "n": n, "t0": s.j.get("startedAt") or "", "p": data}, ensure_ascii=False,
                      separators=(",", ":")).replace("</", "<\\/")
    hover = '<p class="sk-hint">그래프에 마우스를 올리거나 눌러 방향키로 옮기면 그 칸의 값이 나온다.</p>'
    cap = "공회전 동안 값이 어떻게 변했나"
    return (f'<div class="sk-charts"><p class="cap">{cap}</p><p class="sk-guide">{esc(guide)}</p>{key}'
            + "".join(groups) + hover
            + f'<script type="application/json" class="sk-data">{blob}</script></div>')


# ---------------------------------------------------------------- 19 절 — 표
def _attr(classes: str) -> str:
    return f' class="{classes}"' if classes else ""


class Stacked(str):
    """stack_html 이 돌려주는 HTML — 웹 · PDF 에는 여느 글자(str)이고, 앱 보고서(qc_export)는 안의 Table 을 꺼내 쓴다."""
    _table: Table


def stack_html(t: Table) -> str:
    """Table → HTML. 넓은 화면은 여느 표, 폰에서는 줄마다 카드처럼(첫 칸이 제목, 나머지는 '칸 이름 · 값').
    qc_steps.table_html 과 같은 틀(cap · table-wrap · mt · tnote)에 칸 이름(data-l)만 더한다."""
    out = Stacked(_stack_html(t))
    out._table = t
    return out


def _stack_html(t: Table) -> str:
    head = "".join(f'<th{_attr("num" if i in t.num else "")}>{esc(c)}</th>' for i, c in enumerate(t.cols))
    body = []
    for r in t.rows:
        tds = []
        for i, c in enumerate(r):
            text, cls = c if isinstance(c, tuple) else (c, "")
            classes = " ".join(x for x in ("num" if i in t.num else "", cls) if x)
            tds.append(f'<td data-l="{esc(t.cols[i]) if i < len(t.cols) else ""}"{_attr(classes)}>{esc(text)}</td>')
        body.append("<tr>" + "".join(tds) + "</tr>")
    return ((f'<p class="cap">{esc(t.caption)}</p>' if t.caption else "")
            + f'<div class="table-wrap"><table class="mt sk-stack"><thead><tr>{head}</tr></thead><tbody>{"".join(body)}'
            + "</tbody></table></div>" + (f'<p class="tnote">{esc(t.note)}</p>' if t.note else ""))


def _diag_changes(j: dict) -> list[dict]:
    """진단 카운터 — 시작과 끝이 다른 것만. [{name, mean, YAW:(s,e), PITCH:(s,e)}]"""
    by = {}
    for x in j.get("diag") or []:
        by.setdefault(x.get("name"), {})[x.get("axis")] = (x.get("start"), x.get("end"))
    means = {n: m for n, m, _ in DIAG}
    out = []
    for name, axes in by.items():
        if any(se[0] is not None and se[1] is not None and se[0] != se[1] for se in axes.values()):
            out.append({"name": name, "mean": means.get(name, ""), **axes})
    return out


def _diag_read(j: dict) -> str:
    d = j.get("diag") or []
    if not d:
        return "none"
    if all(x.get("end") is None for x in d):
        return "start" if any(x.get("start") is not None for x in d) else "none"
    return "both"


REAL_NAME = {"hard0": "YAW 따라가는 오차", "hard1": "PITCH 따라가는 오차", "lost": "확인 신호 답 없음",
             "fb0": "YAW 위치 신호 부족", "fb1": "PITCH 위치 신호 부족", "cam0": "줌 프레임 부족",
             "cam1": "열화상 프레임 부족", "link": "연결 끊김"}


def real_parts(jd: Judge) -> str:
    """진짜 이상을 까닭별로 — 'YAW 따라가는 오차 1초 · 확인 신호 답 없음 3초'."""
    return " · ".join(f"{REAL_NAME[k]} {jd.n[k]}초" for k in REAL if jd.n.get(k))


def real_fails(jd: Judge) -> bool:
    """진짜 이상 가운데 FAIL 로 가는 것이 있나 — 연결 끊김 · 위치 신호 부족 · 카메라 이상은 1초라도, 따라가는 오차 ·
    답 없음은 lost_level 이 FAIL(연달아 LOST_FAIL_RUN_S 초 · 횟수 한도를 넘음)."""
    return bool(jd.n.get("link") or jd.n.get("fb") or jd.n.get("cam")
                or _num(jd.st.get("trackHardRunMax"), 0) >= RUN_FAIL_S or lost_bad(jd))


def lost_bad(jd) -> bool:
    """답 없음이 FAIL 몫인가 — 다시 센 것(jd.lost)이 있으면 그것, 없으면 연달아 초만 본다."""
    if not jd:
        return False
    return jd.lost == "FAIL" if jd.lost or jd.n else _num(jd.st.get("lostRunMax"), 0) >= LOST_FAIL_RUN_S


def _old_rows(s: Soak) -> list:
    """한눈에 표 — 판정을 어디서 가져왔나. 옛 판정은 작게(흐린 글자로) 곁들인다."""
    jd, vd = s.jd, s.verdict
    if not jd or s.state != "done":
        return []
    out = []
    if jd.source == "soak.csv" and jd.old:
        then = [str(x).replace("이상값 ", "기준을 벗어난 시간 ") for x in s.j.get("reasons") or [] if x]
        if jd.old == vd:
            text = f"{jd.old} — soak.csv 를 새 기준(09-29)으로 다시 세어도 같다"
        else:
            text = (f"{jd.old}" + (f"(그때 까닭: {' · '.join(then)})" if then else "")
                    + " — 이 보고서는 soak.csv 를 새 기준(09-29)으로 다시 셌다")
        out.append(["기록 당시 판정 (옛 기준)", (text, "muted")])
    elif jd.source == "soak.json" and jd.old and jd.old != vd:
        out.append(["옛 기준으로는", (f"{jd.old} — 한 번이라도 기준을 벗어나면 FAIL 로 보던 09-29 전 기준", "muted")])
    elif jd.source == "soak.json v1":
        out.append(["판정 기준", ("옛 기준(09-29 전) 그대로 — soak.csv 가 없어 새 기준으로 다시 세지 못했다", "warn")])
    if jd.diff:
        out.append(["다시 센 판정", (f"soak.csv 로 다시 세면 {jd.recount} — 공회전 탭 기록과 다른 값: {' · '.join(jd.diff)}", "warn")])
    return out


def overview_table(s: Soak) -> Table:
    j, a, jd = s.j, s.after, s.jd
    st = j.get("stats") or {}
    lim = j.get("limits") or {}
    t = Table(["항목", "값"], caption="한눈에")
    vd = s.verdict
    t.rows.append(["판정", cell_v(vd) if vd in ("PASS", "WARN", "FAIL") else (vd, "muted")])
    if jd and s.state == "done":
        why = jd.reasons
        t.rows.append(["까닭", (" · ".join(why), {"FAIL": "bad", "WARN": "warn"}.get(vd, "")) if why
                       else "기준을 벗어난 때 없음"])
    t.rows += _old_rows(s)
    ran, plan = j.get("ranS"), j.get("plannedS")
    if s.state in ("running", "stale"):
        ran = max(float(ran or 0), s.last + 1)
    t.rows.append(["돌린 시간", f"{dur(ran)} (계획 {dur(plan)})"])
    t.rows.append(["시작과 끝", f"{j.get('startedAt') or '—'} ~ {j.get('endedAt') or ('도는 중' if s.state == 'running' else '—')}"])
    if jd and jd.source != "soak.json v1":
        rl = _num(lim.get("rttMaxMs"), 100.0)
        lat = int(_num(jd.st.get("latencyS"), 0))
        base_t = f"평소 {fx(jd.rtt_base, 1, ' ms')}" if s.rows is not None else ""
        if lat:
            t.rows.append([f"통신 지연 (왕복 {rl:g} ms 넘음)",
                           (f"{lat}초 — 제품 고장이 아니라 PC 와 게이트웨이 사이 통신이 밀린 것" + (
                               f". 가장 긴 왕복 {fx(jd.rtt_max, 0, ' ms')}, {base_t}" if base_t else "")
                            + ("" if LATENCY_IS_WARN else " (판정에는 안 씀)"), "warn" if LATENCY_IS_WARN else "muted")])
        else:
            t.rows.append([f"통신 지연 (왕복 {rl:g} ms 넘음)", "0초" + (f" ({base_t})" if base_t else "")])
        late = int(_num(jd.st.get("fbLateS"), 0))
        if late:
            t.rows.append(["위치 신호 밀림 (한 초 모자라고 다음 초에 몰려 옴)",
                           (f"{late}초 — 신호를 잃은 게 아니라 늦게 온 것이라 제품 고장이 아니라 통신이 밀린 것", "warn")])
        if s.rows is not None:
            real = jd.n.get("real", 0)
            t.rows.append(["진짜 이상 (통신 지연 몫을 빼고도 남은 것)",
                           (f"{real}초 — {real_parts(jd)}", "bad" if real_fails(jd) else "warn") if real else "0초"])
    nout = st.get("outlierRows") if st.get("outlierRows") is not None else len(s.out_rows)
    if s.rows is not None and s.state != "done":
        nout = len(s.out_rows)
    by = " · ".join(f"{CAT_NAME.get(c, c)} {n}초" for c, n in primary_counts(s))
    t.rows.append(["기준을 벗어난 시간 (옛 기준으로 센 것)", (f"{nout}초" + (f" — {by}" if by else ""), "muted") if nout else "0초"])
    link = a["link"] if s.rows is not None else st.get("linkDownS")
    t.rows.append(["게이트웨이 연결이 끊긴 시간", (f"{link}초", "bad") if link else "0초"])
    pings, lost = st.get("pings"), st.get("lost")
    if s.rows is not None and s.state != "done":
        lost = a["lost"]
    lost_cls = "bad" if lost_bad(jd) else "warn" if jd and jd.lost else ""
    t.rows.append(["확인 신호(CAN_CHECK)", (f"{comma(pings)}번 보냄 · 응답 없음 {comma(lost)}번", lost_cls if lost else "")])
    slept = j.get("sleptS") or 0
    t.rows.append(["PC 가 잠든 시간", (f"{slept:.0f}초 — 그동안 기록 없음", "bad") if slept else "0초"])
    fault = j.get("fault")
    t.rows.append(["드라이버 폴트", (f"{fault} — 그 자리에 멈춤", "bad") if fault else "없음"])
    cap = cur_cap(j)
    if cap:
        if cap["on"]:
            hit = cap["atCapS"]
            t.rows.append([f"PITCH 임시 전류 상한 {cap['pitchA']:g} A (공회전 동안만)",
                           (f"걸었음 · 전류가 상한의 {cap['ratio'] * 100:.0f}% 이상이던 시간 모두 {hit:g}초. 이때 따라가는 오차가 "
                            "한계를 넘었다면 상한 탓일 수 있다", "warn") if hit > 0
                           else "걸었음 · 상한에 닿은 적 없음"])
        elif cap["state"] == "noReply":
            t.rows.append([f"PITCH 임시 전류 상한 {cap['pitchA']:g} A (공회전 동안만)",
                           ("못 걸었음 — 모터 펌웨어에 임시 상한 기능이 없어 상한 없이 돌았다", "muted")])
        elif cap["state"] == "refused":
            t.rows.append([f"PITCH 임시 전류 상한 {cap['pitchA']:g} A (공회전 동안만)", ("모터가 거절해 상한 없이 돌았다", "warn")])
    rb = robot_info(s.mdir)
    if rb and rb.get("robotSeen"):
        mv = int(_num(rb.get("movingSoakS"), 0))
        t.rows.append(["로봇이 움직인 시간", (f"{mv}초 — 그동안 잰 값에는 로봇이 움직여 생긴 흔들림이 섞였을 수 있다", "muted") if mv
                       else "0초 — 공회전 동안 로봇은 서 있었다"])
    dr = _diag_read(j)
    ch = _diag_changes(j)
    if dr == "both":
        t.rows.append(["진단 카운터", (f"{len(ch)}가지 바뀜 — 아래 표", "warn") if ch else "12가지 모두 그대로"])
    elif dr == "start":
        t.rows.append(["진단 카운터", ("끝나면 한 번 더 읽어 견준다", "muted")])
    sin = j.get("sin") or {}
    parts = []
    for k, name in (("yaw", "YAW"), ("pitch", "PITCH")):
        x = sin.get(k) or {}
        if x:
            parts.append(f"{name} {fx(x.get('center'), 0, '°')} ± {fx(x.get('amp'), 0, '°')}, {fx(x.get('periodS'), 0)}초 주기"
                         f" (가장 빠를 때 {fx(x.get('peakDps'), 0)} °/s)")
    if parts:
        t.rows.append(["움직인 모양 (사인파)", " · ".join(parts)])
    clip = j.get("clip") or {}
    if clip:
        secs = clip.get("seconds") or 0
        if not secs:
            ctext = ("안 찍음 — 공회전 탭 설정이 0초", "muted")
        elif clip.get("ok"):
            ctext = f"끝 {secs}초 녹화 있음 (soak_clip/)"
        elif clip.get("started"):
            ctext = ("못 찍음 — 카메라 설정이 없거나 녹화 탭이 이미 녹화 중", "warn")
        else:
            ctext = ("아직 안 찍음 — 끝나기 30초 전에 찍는다", "muted") if s.state == "running" else ("안 찍음", "muted")
        t.rows.append(["끝 영상", ctext])
    if j.get("csvError"):
        t.rows.append(["soak.csv 기록", ("쓰다가 실패함 — 그 뒤 1초 기록이 없다", "bad")])
    if s.rows is None:
        t.rows.append(["soak.csv", ("없음 — 그래프와 기준을 벗어난 때 목록을 만들지 못함", "warn")])
    elif s.rows.get("bad"):
        t.rows.append(["soak.csv 에서 못 읽은 줄", (f"{s.rows['bad']}줄", "muted")])
    return t


def metrics_table(s: Soak) -> Table:
    j, a, jd = s.j, s.after, s.jd
    lim = j.get("limits") or {}
    base = j.get("baseline") or {}
    st = j.get("stats") or {}
    sin = j.get("sin") or {}
    ready = bool(base.get("ready"))
    have = s.rows is not None
    na = ("—", "muted")
    t = Table(["항목", "대상", "가장 나쁜 값", "한계", "통신 지연 탓", "진짜 이상"], num=(4, 5),
              caption="항목별 가장 나쁜 값과 한계")
    fbmin = lim.get("fbMinPerS", 90)
    gap_max = [(st.get("gapMaxMs") or [None, None])[ax] for ax in (0, 1)]
    if have:
        gap_max = [max((g for g in s.rows[f"gap{ax}"] if g is not None), default=None) for ax in (0, 1)]
    for ax in (0, 1):
        v = a["fbMin"][ax] if have else (st.get("fbMin") or [None, None])[ax]
        v = None if v is not None and v < 0 else v
        mk = jd.marks.get(a["fbMinAt"][ax], ()) if have and jd else ()
        cls = "" if v is None or v >= fbmin else "warn" if f"late{ax}" in mk else "bad"
        g = gap_max[ax] if gap_max[ax] is not None and gap_max[ax] >= 0 else None
        vt = f"{fx(v, 0)}번/초 (가장 적을 때)" + (f" · 신호 사이 가장 긴 공백 {fx(g, 0, ' ms')}" if g is not None else "")
        t.rows.append([("위치 신호 수", "sk-nw"), AX_NAME[ax], (vt, cls),
                       f"{fbmin:g}번/초 이상. 모자라도 다음 초에 몰려 와 두 초 합이 {2 * fbmin:g}번 이상이면 늦게 온 것(통신 밀림)",
                       _nv2(s, f"late{ax}", "warn"), _nv2(s, f"fb{ax}")])
    tl = base.get("trackLimit") or [None, None]
    tp = base.get("trackP99") or [None, None]
    for ax in (0, 1):
        pk = (sin.get(("yaw", "pitch")[ax]) or {}).get("peakDps")
        if have:
            v = a["trAfter"][ax]
            vt = f"{fx(v, 2, '°')} (가장 클 때, 기준 구간 뺌)" if s.base_s else f"{fx(v, 2, '°')} (가장 클 때)"
        else:
            v = (st.get("trackMax") or [None, None])[ax]
            vt = f"{fx(v, 2, '°')} (가장 클 때, 기준 구간 포함 — soak.csv 가 없어 빼지 못함)"
        lim_v = tl[ax] if ready and ax < len(tl) else None
        if lim_v is None:
            lt = ("기준 구간이 끝나면 정한다", "muted")
        else:
            lt = (f"{fx(lim_v, 2, '°')} 이하 — 기준 구간 p99 {fx(tp[ax] if ax < len(tp) else None, 2, '°')} "
                  f"× {lim.get('trackFactor', 1.5):g} + {lim.get('trackAddDeg', 0.5):g}°. 통신이 밀린 때는 늦어진 시간 × "
                  f"{fx(pk, 1)} °/s(가장 빠를 때 속도)만큼 늘린다")
        # 가장 나쁜 1초의 색 — 늘린 한계마저 넘었으면 빨강, 한계는 넘었지만 늘린 한계 안이면 주황
        mk = jd.marks.get(a["trAfterAt"][ax], ()) if have and jd else ()
        over = v is not None and lim_v is not None and v > lim_v
        cls = "bad" if f"hard{ax}" in mk else "warn" if f"soft{ax}" in mk else ("bad" if over and not have else "")
        t.rows.append([("따라가는 오차", "sk-nw"), AX_NAME[ax], (vt, cls), lt, _nv2(s, f"soft{ax}", "warn"), _nv2(s, f"hard{ax}")])
    rtt = a["rttMax"] if have else st.get("rttMaxMs")
    p99 = a["rttP99"] if have else st.get("rttP99Ms")
    lost = a["lost"] if have else st.get("lost")
    rl = lim.get("rttMaxMs", 100)
    to = (lim.get("pingTimeoutMs") or 1000) / 1000
    cls = ("bad" if lost_bad(jd)
           else "warn" if (rtt is not None and rtt > rl) or (jd and jd.lost) else "")
    t.rows.append([("통신 왕복 시간", "sk-nw"), "게이트웨이 · YAW · PITCH 번갈아",
                   (f"{fx(rtt, 1, ' ms')} (가장 긴 것) · p99 {fx(p99, 1, ' ms')}" + (f" · 응답 없음 {lost}번" if lost else ""), cls),
                   f"{rl:g} ms 를 넘으면 통신 지연, {to:g}초 안에 응답이 없으면 응답 없음", _nv2(s, "lat", "warn"),
                   _nv2(s, "lost", suffix=" (응답 없음)")])
    fl = base.get("fpsLimit") or [None, None]
    fm = base.get("fpsMedian") or [None, None]
    med_all = st.get("fpsMedian") or [None, None]
    for c in (0, 1):
        measured = ((j.get("cams") or {}).get(CAM_KEY[c]) or {}).get("measured")
        if not measured and not (have and any(v is not None for v in s.rows[f"fps{c}"])):
            t.rows.append([("카메라 초당 프레임", "sk-nw"), CAM_NAME[c], ("안 봄 — CONFIG 탭에 주소·암호 없음", "muted"), "", "", ""])
            continue
        v = a["fpsAfter"][c] if have else (st.get("fpsMin") or [None, None])[c]
        v = None if v is not None and v < 0 else v
        zero = a["fpsZero"][c] if have else None
        lv = fl[c] if ready and c < len(fl) and fl[c] not in (None, -1) else None
        med = med_all[c] if c < len(med_all) and med_all[c] not in (None, -1) else None
        vt = f"{fx(v, 0)}장/초 (가장 적을 때" + (", 기준 구간 뺌)" if s.base_s and have else ")") + \
            (f" · 가운데 값 {fx(med, 0)}" if med is not None else "") + (f" · 0장 {zero}초" if zero else "")
        lt = ((f"{_tick(lv)}장/초 이상 — 기준 구간 가운데 값 {_tick(fm[c])} × {lim.get('fpsMinRatio', 0.8):g}. 0장이면 언제나 이상")
              if lv is not None else ("0장이 아니어야 한다. 비율 한계는 기준 구간이 끝나면 정한다", "muted"))
        t.rows.append([("카메라 초당 프레임", "sk-nw"), CAM_NAME[c], (vt, "bad" if zero or (v is not None and lv is not None and v < lv) else ""),
                       lt, na, _nv2(s, f"cam{c}")])
    notes = []
    if s.base_s:
        notes.append(f"기준 구간은 처음 {dur(s.base_s)}이다. 따라가는 오차와 카메라 초당 프레임은 제품마다 크기가 달라서, 이 구간에서 잰 "
                     "값으로 한계를 정하고 그 뒤부터 견준다. p99 는 100번 가운데 99번이 이보다 작다는 값이다.")
    if have and s.base_s and any(x is not None for x in a["trBase"]):
        notes.append(f"기준 구간 안의 가장 큰 따라가는 오차는 YAW {fx(a['trBase'][0], 2, '°')} · PITCH {fx(a['trBase'][1], 2, '°')} 다. "
                     "사인파에 들어가는 첫 몇 초가 들어 있어 한계와 견주지 않는다.")
    base_rtt = f"({fx(jd.rtt_base, 1, ' ms')})" if jd and have else ""
    notes.append(f"마지막 두 칸은 그런 일이 있던 시간(초)이다. '통신 지연 탓'은 통신이 밀려서 기준을 벗어난 시간으로, 위치 신호 수는 "
                 "모자랐지만 다음 초에 몰려 온 때를, 따라가는 오차는 한계를 넘었지만 늘린 한계 안에 든 때를, 통신 왕복 시간은 왕복이 "
                 f"{rl:g} ms 를 넘은 때를 센다. '진짜 이상'은 통신 지연 몫을 빼고도 남은 이상으로, 따라가는 오차는 늘린 한계마저 넘은 "
                 "때를, 통신 왕복 시간은 확인 신호에 응답이 없던 때를, 위치 신호 수는 모자라고 다음 초에도 채워지지 않은 때를, 카메라 "
                 "초당 프레임은 한계를 벗어난 때를 센다. 따라가는 오차가 큰 것과 프레임이 적은 것은 기준 구간 뒤부터 세고, 나머지는 "
                 "처음부터 센다. 한계를 늘릴 때 쓰는 늦어진 시간은 바로 앞 초 · 그 초 · 바로 다음 초의 가장 긴 왕복 가운데 가장 큰 "
                 f"것에서 평소 왕복{base_rtt}을 뺀 값이다. 평소 왕복은 기준 구간 왕복의 가운데값이고, 왕복을 못 잰 초에는 직전 "
                 f"{RTT_FILL_S}초 안의 마지막 왕복을 쓴다. 신호 사이 공백은 위치 신호가 한 번 오고 다음 신호가 오기까지 가장 오래 "
                 "빈 시간이다(평소 10 ms 쯤, 판정에는 안 씀).")
    t.note = " ".join(notes)
    return t


def primary_counts(s: Soak) -> list[tuple[str, int]]:
    """기준을 벗어난 1초를 까닭별로 — 연결이 끊긴 초는 '연결 끊김' 하나로 센다(그 초의 위치 신호 0 · 왕복 없음은 끊김 탓)."""
    n: dict[str, int] = {}
    for _, _, why in s.out_rows:
        cs = cats(why)
        for c in (("link",) if "link" in cs else cs):
            n[c] = n.get(c, 0) + 1
    order = list(CAT_NAME)
    return sorted(n.items(), key=lambda x: (-x[1], order.index(x[0]) if x[0] in order else 99))


def _nv2(s: Soak, key: str, cls: str = "bad", suffix: str = ""):
    """항목별 표의 판정 v2 칸 — 그 표지가 걸린 1초 수. soak.csv 가 없으면 못 센다."""
    if s.rows is None or not s.jd:
        return ("—", "muted")
    n = s.jd.n.get(key, 0)
    return (f"{n}초{suffix}", cls) if n else "0초"


def diag_table(s: Soak) -> Table | None:
    ch = _diag_changes(s.j)
    if not ch or _diag_read(s.j) != "both":
        return None
    t = Table(["항목", "뜻", "YAW", "PITCH", "공회전 판정에서"], num=(2, 3), caption="진단 카운터 — 공회전 동안 바뀐 것")
    fault = DIAG[DIAG_FAULT_IDX][0]
    state = {DIAG[i][0] for i in DIAG_STATE_IDX}
    for x in ch:
        cells = []
        for ax in AX_NAME:
            se = x.get(ax)
            if not se or se[0] is None or se[1] is None:
                cells.append(("—", "muted"))
                continue
            d = se[1] - se[0]
            txt = f"{se[0]} → {se[1]}" + (f" ({d:+d})" if d else "")
            bad = (x["name"] == fault and d > 0) or (d < 0 and x["name"] not in state)
            cells.append((txt, "bad") if bad else txt)
        rule = ("늘거나 줄면 FAIL" if x["name"] == fault else "상태값이라 판정에 안 씀" if x["name"] in state
                else "줄면 FAIL — 도중에 모터가 다시 켜진 것")
        t.rows.append([x["name"], x["mean"], *cells, (rule, "muted")])
    t.note = ("모터 보드가 켜진 뒤로 센 수라, 시작보다 줄었으면 공회전 도중에 모터가 다시 켜진 것이다. STATUS · nFAULT · 복구 I2C 는 "
              "센 수가 아니라 마지막 상태라서 판정에 쓰지 않는다.")
    return t


def real_table(s: Soak) -> Table | None:
    """진짜 이상이 있던 1초 — 통신 지연 몫을 빼고도 남은 것. 옛 목록(outliers_table)은 시간 순이라 앞 50개에 안 들 수 있어 따로 모은다."""
    jd, r = s.jd, s.rows
    if not jd or r is None:
        return None
    secs = [sec for sec in sorted(jd.marks) if jd.marks[sec] & set(REAL)]
    if not secs:
        return None
    base = s.j.get("baseline") or {}
    tl = _pair(base.get("trackLimit"))
    sin = s.j.get("sin") or {}
    pk = [_num((sin.get(k) or {}).get("peakDps"), 0.0) for k in ("yaw", "pitch")]
    want = set(secs[:OUTLIER_SHOW])
    at = {sec: i for i, sec in enumerate(r["sec"]) if sec in want}
    t = Table(["시작부터 지난 시간", "시각", "무엇이"], num=(0,),
              caption=f"진짜 이상이 있던 때 — 통신 지연 몫을 빼고도 남은 것, 모두 {len(secs)}초"
              + (f", 앞 {OUTLIER_SHOW}개" if len(secs) > OUTLIER_SHOW else ""))
    for sec in secs[:OUTLIER_SHOW]:
        i, m = at.get(sec), jd.marks[sec]
        what = []
        for a in (0, 1):
            if f"hard{a}" in m and i is not None:
                eff = jd.eff.get(sec, jd.rtt_base)
                lim_a = (tl[a] or 0.0) + pk[a] * max(0.0, eff - jd.rtt_base) / 1000.0
                what.append(f"{AX_NAME[a]} 따라가는 오차 {fx(r[f'tr{a}'][i], 2, '°')} — 늘린 한계 {fx(lim_a, 2, '°')} 넘음"
                            f"(왕복 {fx(eff, 1, ' ms')})")
        if "lost" in m and i is not None:
            what.append(f"확인 신호 응답 없음 {r['lost'][i]}번")
        for a in (0, 1):
            if f"fb{a}" in m and i is not None:
                what.append(f"{AX_NAME[a]} 위치 신호 {r[f'fb{a}'][i]}번")
        for c in (0, 1):
            if f"cam{c}" in m and i is not None:
                what.append(f"{CAM_NAME[c]} {r[f'fps{c}'][i]}장/초")
        if "link" in m:
            what.append("게이트웨이 연결 끊김")
        t.rows.append([dur(sec), (r["t"][i] if i is not None else "") or "—", (" · ".join(what), "bad")])
    if any(jd.marks[sec] & {"hard0", "hard1"} for sec in secs[:OUTLIER_SHOW]):
        t.note = ("따라가는 오차의 늘린 한계는 기준 구간에서 정한 한계에 통신이 늦어진 만큼을 더한 값이다. 괄호 안 왕복은 그 초에 쓴 "
                  "왕복으로, 바로 앞 초 · 그 초 · 바로 다음 초의 왕복 가운데 가장 큰 것이다(못 쟀으면 직전 값).")
    if len(secs) > 20:
        t.fold = f"진짜 이상 목록 보기 ({min(len(secs), OUTLIER_SHOW)}개)"
    return t


def outliers_table(s: Soak) -> Table | None:
    rows = s.out_rows
    if s.rows is None:
        rows = [(o.get("s"), o.get("t"), o.get("why")) for o in s.j.get("outliers") or []]
    if not rows:
        return None
    total = len(rows) if s.rows is not None else (s.j.get("stats") or {}).get("outlierRows", len(rows))
    t = Table(["시작부터 지난 시간", "시각", "까닭", "새 기준으로 가르면"], num=(0,),
              caption=f"기준을 벗어난 때(옛 기준으로 센 것) — 모두 {total}초" + (f", 앞 {OUTLIER_SHOW}개" if total > OUTLIER_SHOW else ""))
    jd = s.jd if s.rows is not None else None
    for sec, when, why in rows[:OUTLIER_SHOW]:
        m = jd.marks.get(sec, set()) if jd else set()
        if not jd:
            cls = ("—", "muted")
        elif m & set(REAL):
            cls = ("진짜 이상", "bad")
        elif m & {"lat", "soft0", "soft1", "late0", "late1"}:
            cls = ("통신 지연 탓", "warn")
        else:
            cls = ("—", "muted")
        t.rows.append([dur(sec), when or "—", plain_why(why), cls])
    t.note = ("까닭의 숫자는 그 초에 잰 값이다. 위치 신호는 받은 수, 따라가는 오차는 가장 큰 차이, 왕복은 가장 긴 왕복이고, "
              "'확인 신호 응답 없음'은 1초 안에 응답이 안 온 수다. 기준을 벗어난 때는 모두 soak.csv 의 '이상' 칸에 있다. '새 기준으로 "
              "가르면' 칸은 통신이 밀려서 벗어난 것(통신 지연 탓)인지, 그 몫을 빼고도 남은 것(진짜 이상)인지 가른 것이다.")
    if total > 20:
        t.fold = f"기준을 벗어난 때 목록 보기 ({min(total, OUTLIER_SHOW)}개)"
    return t


def events_table(s: Soak) -> Table | None:
    ev = [str(x) for x in s.j.get("events") or [] if x]
    if not ev:
        return None
    t = Table(["시작부터 지난 시간", "있었던 일"], caption=f"사건 — {len(ev)}건" + (f", 앞 {EVENTS_SHOW}건" if len(ev) > EVENTS_SHOW else ""))
    for x in ev[:EVENTS_SHOW]:
        m = re.match(r"^(\d+):(\d\d):(\d\d)\s+(.*)$", x)          # soaktab.cpp hms — 공회전을 시작하고 지난 시간
        t.rows.append([dur(int(m.group(1)) * 3600 + int(m.group(2)) * 60 + int(m.group(3))), m.group(4)] if m else ["—", x])
    t.note = "공회전 탭이 남긴 일 — 게이트웨이 연결 끊김과 다시 붙음, 카메라 다시 연결, PC 절전 막기 풀림, 기록 실패."
    if len(ev) > 20:
        t.fold = f"사건 목록 보기 ({min(len(ev), EVENTS_SHOW)}건)"
    return t


# ---------------------------------------------------------------- 19 절 — 채우기
def crit_v2(j: dict | None) -> list[tuple[str, str]]:
    """19 절 기준 — 판정 v2 를 soak.json limits 의 숫자로(없으면 공회전 탭 기본값, soakcore.h SoakLimit)."""
    lim = (j or {}).get("limits") or {}
    base = (j or {}).get("baseline") or {}
    sin = (j or {}).get("sin") or {}
    fb, rtt = _num(lim.get("fbMinPerS"), 90.0), _num(lim.get("rttMaxMs"), 100.0)
    to = _num(lim.get("pingTimeoutMs"), 1000.0) / 1000
    f, add = _num(lim.get("trackFactor"), 1.5), _num(lim.get("trackAddDeg"), 0.5)
    ratio = _num(lim.get("fpsMinRatio"), 0.8)
    bs = base.get("seconds") or lim.get("baselineS")
    bt = f"처음 {dur(bs)}" if bs else "공회전 시간의 10%, 30초~10분"
    pk = _num((sin.get("yaw") or {}).get("peakDps"), 0.0)
    ex = (f" 예를 들어 왕복이 평소보다 0.1초 늦으면 YAW 는 가장 빠를 때 {pk:g} °/s 로 움직이니 한계가 {pk * 0.1:.1f}° 늘어난다."
          if pk else "")
    n = RUN_FAIL_S
    lat = ("통신 지연(확인 신호 왕복이 {rtt:g} ms 를 넘은 때)이 있었다. 제품 고장이 아니라 PC 와 게이트웨이 사이 통신이 "
           "밀린 것이다. ").format(rtt=rtt) if LATENCY_IS_WARN else ""
    return [
        ("PASS", "끝까지 돌았고, 아래 FAIL 과 WARN 어디에도 걸리지 않았다."),
        ("FAIL", f"다음 가운데 하나라도 있었다. 게이트웨이 연결이 끊겼다. 위치 신호가 1초에 {fb:g}번보다 적었고 다음 1초에도 채워지지 "
                 f"않았다(두 초 합이 {2 * fb:g}번보다 적음). 카메라 프레임이 "
                 f"0장이거나 기준 구간 가운데 값의 {ratio * 100:.0f}% 보다 적었다. 따라가는 오차가 늘린 한계를 넘은 상태가 {n}초 이상 "
                 f"이어졌다. 확인 신호에 응답이 없는 상태가 {LOST_FAIL_RUN_S}초 이상 이어졌거나, 응답 없음이 24시간에 "
                 f"{LOST_FAIL_PER_DAY:.0f}번(공회전 시간에 비례)을 넘었다. 드라이버 폴트가 났다. 도중에 모터가 다시 켜졌다"
                 "(진단 카운터가 시작보다 줄어듦). PC 가 잠들었다."),
        ("중지", "FAIL 에는 안 걸렸지만 끝까지 못 돌고 멈췄다. 통신 지연이 있었어도 중지로 본다."),
        ("WARN", f"끝까지 돌았고 FAIL 은 없지만 걸린 것이 있다. {lat}위치 신호가 1초에 {fb:g}번보다 적었지만 다음 1초에 몰려 와 "
                 f"두 초 합이 {2 * fb:g}번 이상이면 신호를 잃은 게 아니라 늦게 온 것이라(위치 신호 밀림) WARN 이다. "
                 f"따라가는 오차가 늘린 한계를 넘은 때가 있었지만 {n}초 이상 "
                 f"이어지지 않은 것도 WARN 이다. 확인 신호의 응답이 {to:g}초 안에 오지 않은 것은 {LOST_WARN_RUN_S}초 이상 이어졌거나 "
                 f"FAIL 한도 횟수의 2/3 이상이면 WARN 이고, 그보다 적게 따로 빠진 것은 기록만 한다."),
        ("", f"한계는 이렇게 정한다. 따라가는 오차와 카메라 초당 프레임은 제품마다 크기가 달라서 기준 구간({bt})에 잰 값으로 "
             f"한계를 정한다. 그래서 기준 구간에서는 따라가는 오차가 큰 것과 프레임이 기준보다 적은 것만 판정하지 않는다. 연결 "
             "끊김, 위치 신호 부족, 프레임 0장, 응답 없음, 통신 지연은 처음부터 본다. 따라가는 오차의 한계는 기준 구간 p99(100번 가운데 "
             f"99번이 이보다 작은 값)의 {f:g}배에 {add:g}° 를 더한 값이다. 통신이 밀리면 위치 소식이 늦게 와서 PC 가 조금 전 "
             "각도를 지금 명령과 견주게 된다. 그래서 바로 앞 초 · 그 초 · 바로 다음 초의 왕복 가운데 가장 큰 것이 평소보다 늦은 시간에, 축이 가장 "
             f"빠를 때 속도를 곱해 한계에 더한다(늘린 한계).{ex} 평소 왕복은 기준 구간 왕복의 가운데값이다. 카메라 초당 프레임의 "
             f"한계는 기준 구간 가운데 값의 {ratio * 100:.0f}% 다."),
    ]


def line_tail(s: Soak) -> str:
    """요약표 19 줄에 덧붙일 까닭 — 기록 멈춤 · 도는 중이면 지금까지의 두 수 · 끝났으면 판정 v2 까닭(멈춘 까닭 포함)."""
    jd = s.jd
    if s.state == "stale":
        return f"기록이 {s.stale_s / 60:.0f}분째 멈춤"
    if not jd:
        return ""
    if s.state == "running":
        if s.rows is None:
            return ""
        late = jd.n.get("late", 0)
        return (f"지금까지 통신 지연 {jd.n.get('lat', 0)}초" + (f" · 위치 신호 밀림 {late}초" if late else "")
                + f" · 진짜 이상 {jd.n.get('real', 0)}초")
    return " · ".join(jd.fail + ([jd.stop_why] if jd.stop_why else []) + jd.warn)


def soak_brief(s: Soak | None) -> str:
    """대시보드 공회전 칸 옆 한 마디 — '통신 지연 191초 · 진짜 이상 1초'. 끝난 것만. 1초 표지 없이 FAIL 이면(폴트 · 잠듦 ·
    진단 카운터) 그 까닭 앞부분."""
    if not s or not s.jd or s.state != "done" or s.rows is None:
        return ""
    parts = []
    lat, real = s.jd.n.get("lat", 0), s.jd.n.get("real", 0)
    if s.jd.verdict == "FAIL" and s.jd.fail and not real:
        parts.append(re.split(r"\(| — ", s.jd.fail[0])[0].strip())
    if lat:
        parts.append(f"통신 지연 {lat}초")
    if s.jd.n.get("late"):
        parts.append(f"위치 신호 밀림 {s.jd.n['late']}초")
    if real:
        parts.append(f"진짜 이상 {real}초")
    return " · ".join(parts)


def fill_step19(v: StepView, mdir: Path | None, stage: str | None = None) -> Soak | None:
    """19 절에 공회전 자료를 더한다 — qc_steps.build 가 _s19(글 · 판정 · 한 줄 · 사유) 바로 뒤에 부른다.

    기준 칸은 판정 v2 글(crit_v2 — soak.json limits 의 숫자)로 바꾼다. soak.json 이 있으면 판정 · 한 줄을 판정 v2 로
    다시 쓰고(judge — 옛 기록은 soak.csv 로 다시 셈), _s19 의 작은 표를 한눈에 표(같은 칸에 더 자세히)로 바꾸고 항목별 표 ·
    진단 카운터 · 시간 그래프 · 진짜 이상 · 기준을 벗어난 1초 · 사건을 붙인다. v.tables 에는 Table 과 HTML 조각(str)이 섞인다
    — qc_steps.table_html 이 둘 다 그린다. stage 는 쓰지 않는다(공회전 기록이 없을 때의 글은 _s19 가 stage 로 가른다)."""
    s = load(mdir)
    v.crit = crit_v2(s.j if s else None)
    if not s:
        return None
    j = s.j
    v.verdict = s.verdict
    ran, plan = j.get("ranS"), j.get("plannedS")
    if s.state in ("running", "stale"):
        head = f"{dur(plan)} 가운데 {dur(max(float(ran or 0), s.last + 1))} 돌림"
    elif j.get("completed") or (ran is not None and plan is not None and float(ran) >= float(plan) - 0.5):
        head = f"계획한 {dur(plan)} 다 돌림"
    else:
        head = f"{dur(plan)} 가운데 {dur(ran)} 돌고 멈춤"
    tail = line_tail(s) or ("기준을 벗어난 때 없음" if s.state == "done" and s.verdict == "PASS" else "")
    v.line = head + (f" · {tail}" if tail else "")
    # _s19 가 soak.json reasons 로 쓴 '판정 사유'는 옛 기준 글이다 — 판정 v2 까닭은 한눈에 표에 있다
    if s.jd and s.jd.source == "soak.json v1":
        v.notes = [n.replace("이상값 ", "기준을 벗어난 시간 ") for n in v.notes]
    else:
        v.notes = [n for n in v.notes if not n.startswith("판정 사유 — ")]
    t = [f"<style>{SOAK_CSS}</style>"]
    if s.state == "running":
        t.append(f'<p class="sk-live">공회전 중 — 지금까지 {esc(dur(s.last + 1))} 돌았다(계획 {esc(dur(plan))}). 아래 표와 '
                 "그래프는 지금까지 모은 것이다. 이 페이지는 공회전이 끝나면 다시 만들어진다.</p>")
    elif s.state == "stale":
        t.append(f'<p class="sk-live bad">기록이 {s.stale_s / 60:.0f}분째 멈춰 있다 — 공회전 프로그램이 꺼졌거나 PC 가 멈췄을 수 '
                 f"있다. 아래는 멈추기 전까지 모은 것이다({esc(dur(s.last + 1))} / 계획 {esc(dur(plan))}).</p>")
    t.append(overview_table(s))
    t.append(stack_html(metrics_table(s)))
    dt = diag_table(s)
    if dt:
        t.append(stack_html(dt))
    ch = charts_html(s)
    if ch:
        t.append(ch + f"<script>{SOAK_JS}</script>")
    t += [x for x in (real_table(s), outliers_table(s), events_table(s)) if x]
    v.tables = t
    return s


# ---------------------------------------------------------------- '단계별 자세히' 바로 뒤 — 끝 영상 · 20 절
_END: dict = {}          # 웹 내보내기 한 번 안에서 end_clips 가 만든 끝 영상 — section_html 이 찾아 쓴다(검사 폴더 → 결과)


def _pill_web(v) -> str:
    return f'<span class="pill v-{v.vclass or "none"}">{esc(v.verdict or "—")}</span>'


def _pill_pdf(v) -> str:
    from qc_report import VERDICT_COLOR            # PDF 판정 딱지 — qc_report 와 같은 색
    return f'<span class="badge" style="background:{VERDICT_COLOR.get(v.vclass or "none", "#555")}">{esc(v.verdict or "—")}</span>'


def section_html(mdir: Path | None, web: bool = True, stage: str | None = None) -> str:
    """qc_steps.soak_section 이 '단계별 자세히' 바로 뒤에 끼운다 — 공회전 끝 영상(웹, 19 절 이어서)과 20 공회전 뒤 재검사 절.
    재검사가 없고(after.json 없음) 끝 영상도 없으면 빈 글. stage 는 쓰지 않는다(20 은 after.json 이 있을 때만 붙는다)."""
    if not mdir:
        return ""
    mdir = Path(mdir)
    out = []
    clips = _END.get(str(mdir.resolve())) if web else None
    ends = end_clips_html(clips) if clips else ""
    if ends:
        out.append(f'<section class="st sk-cont" aria-label="12시간 공회전 끝 영상">{ends}</section>')
    csv1 = mdir.parent / f"{mdir.name}.csv"
    if read_after(mdir) and csv1.is_file():
        from qc_report import read_qc_csv
        from qc_steps import sections_html
        v20 = recheck_view(read_qc_csv(csv1), mdir, csv1)
        if v20:
            pill = _pill_web if web else _pill_pdf
            if web:
                v20.tables.append(recheck_details_html(v20, pill))
            out.append(sections_html([v20], pill, web))
    return (f"<style>{SOAK_CSS}</style>" + "\n".join(out)) if out else ""


def end_clips(mdir: Path, dest: Path, clip_fn, seconds: int = 60) -> dict:
    """soak_clip/zoom.mkv · thermal.mkv → dest/soak_end_<cam>.mp4 (clip_fn = qc_web.clip). 녹화가 그대로면 다시 안 만든다.
    결과는 section_html 이 쓰게 _END 에도 둔다 — 웹 내보내기가 product_html 보다 먼저 부른다."""
    src_dir = Path(mdir) / "soak_clip"
    info_p = Path(dest) / "soak_clips.json"
    old = read_json(info_p) or {}
    out: dict = {}
    for key in CAM_KEY:
        src = next((src_dir / f"{key}{e}" for e in (".mkv", ".mp4") if (src_dir / f"{key}{e}").is_file()
                    and (src_dir / f"{key}{e}").stat().st_size > 0), None)
        if not src:
            continue
        name = f"soak_end_{key}.mp4"
        dst = Path(dest) / name
        prev = old.get(key) or {}
        if prev.get("ok") and dst.exists() and dst.stat().st_mtime >= src.stat().st_mtime:
            out[key] = prev
            continue
        try:
            r = clip_fn(src, dst, seconds, 0.0)
        except Exception as e:                       # ffmpeg 가 없거나 녹화가 깨짐 — 영상만 빠진다
            r = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        r.update({"file": name, "source": f"soak_clip/{src.name}"})
        out[key] = r
    if out:
        info_p.write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    _END[str(Path(mdir).resolve())] = out
    return out


def end_clips_html(clips: dict) -> str:
    figs = []
    for key, label in zip(CAM_KEY, CAM_NAME):
        c = clips.get(key)
        if not c:
            continue
        if c.get("ok"):
            figs.append(f'<figure><video src="{esc(c["file"])}" controls muted playsinline preload="metadata"></video>'
                        f'<figcaption>{label} — 공회전 마지막 {fx(c.get("seconds"), 0)}초 · 720p 로 다시 압축(원본은 QC PC 의 '
                        f'{esc(c.get("source", "soak_clip/"))})</figcaption></figure>')
        else:
            figs.append(f'<figure><figcaption>{label} — 끝 영상을 못 만듦 ({esc(c.get("error", ""))[:120]})</figcaption></figure>')
    if not figs:
        return ""
    return ('<p class="cap">12시간 공회전 끝 영상</p><p class="sk-guide">공회전이 끝나기 직전 모습이다. 사인파로 움직이던 축이 '
            f'끝까지 매끄럽게 도는지 눈으로 본다.</p><div class="videos">{"".join(figs)}</div>')


# ---------------------------------------------------------------- 20 절 — 재검사(전·후 비교)
@dataclass
class Compare:
    after: dict
    csv2: Path | None
    overall2: str
    started2: str
    steps: list = field(default_factory=list)      # [(no, 이름, 처음, 재검사, 변함 표지, 재검사 한 줄)]
    metrics: list = field(default_factory=list)    # [(이름, 단위, 처음, 재검사, 차이, 한계 글, 변함 표지, 재검사가 한계 대비)]
    views2: list = field(default_factory=list)
    changed: list = field(default_factory=list)    # 크게 변한 것 — 사람 말 한 마디씩


def _vword(v: str) -> str:
    return v if v in RANK else ("안 함" if v in ("", "SKIP", "구현 전", "대기") else v)


def compare(qc1: dict, mdir1: Path | None, csv1: Path | None = None) -> Compare | None:
    """after.json 이 있으면 재검사 CSV 를 읽어 처음과 견준다. 파싱은 qc_steps.build · qc_metrics.key_metrics 그대로."""
    after = read_after(mdir1)
    if not after:
        return None
    from qc_metrics import fmt, key_metrics, rows_for   # qc_metrics → qc_report → (이 모듈) 순환을 피해 여기서
    from qc_report import read_qc_csv
    from qc_steps import build, get
    csv1 = Path(csv1) if csv1 else (Path(mdir1).parent / f"{Path(mdir1).name}.csv")
    csv2 = recheck_csv(csv1, after)
    c = Compare(after=after, csv2=csv2, overall2=str(after.get("overall") or ""), started2=str(after.get("startedAt") or ""))
    if not csv2:
        return c
    qc2 = read_qc_csv(csv2)
    c.overall2 = qc2["head"].get("종합") or c.overall2
    c.started2 = qc2["head"].get("시작") or c.started2
    v1 = {v.no: v for v in build(qc1, None) if v.no <= 18}
    c.views2 = [v for v in build(qc2, csv2.with_suffix("")) if v.no <= 18]
    v2 = {v.no: v for v in c.views2}
    for no in sorted(set(v1) | set(v2)):
        a, b = v1.get(no), v2.get(no)
        va, vb = (a.verdict if a else ""), (b.verdict if b else "")
        mark = ""
        if va in RANK and vb in RANK:
            if RANK[vb] > RANK[va]:
                mark = "나빠짐"
                c.changed.append(f"{no} {(b or a).name} {va}→{vb}")
            elif RANK[vb] < RANK[va]:
                mark = "나아짐"
        elif vb in ("FAIL", "중지") and va not in ("FAIL", "중지"):
            mark = "나빠짐"
            c.changed.append(f"{no} {(b or a).name} {_vword(va)}→{vb}")
        c.steps.append((no, (b or a).name, va, vb, mark, b.line if b else "—"))
    m1, m2 = key_metrics(qc1), key_metrics(qc2)
    rows = rows_for(m2)
    if m1.get("pitchRule") != m2.get("pitchRule"):  # 두 검사가 PITCH 를 다른 기준으로 갈랐다 — 견주지 않는다
        rows = [r for r in rows if not r[0].startswith("pitchCur")]
    for key, label, unit, lim, ratio in rows:
        x1, x2 = m1.get(key), m2.get(key)
        diff = None if x1 is None or x2 is None else x2 - x1
        mark, r2 = "", None
        if ratio and x1 is not None and x2 is not None:
            r1, r2 = ratio(x1), ratio(x2)
            if r2 - r1 >= CHANGE_FRAC - 1e-9:
                mark = "크게 변함"
                u = _unit(unit)
                c.changed.append(f"{label} {fmt(key, x1)}{u} → {fmt(key, x2)}{u}")
            elif r1 - r2 >= CHANGE_FRAC - 1e-9:
                mark = "나아짐"
        elif not ratio:
            mark = "기록만"
        f1, f2 = fmt(key, x1), fmt(key, x2)
        c.metrics.append((label, unit, f1, f2, _signed(diff, f1, f2), lim, "" if mark == "기록만" else mark,
                          None if r2 is None else f"{r2 * 100:.0f}%"))
    # 홈잉 끝~끝 거리(펄스) — 기록만. 차이는 각도로도(끝~끝을 YAW 345° · PITCH 114° 로 본다)
    d1, d2 = dict(qc1["details"]), dict(qc2["details"])
    for ax in AX_NAME:
        keys = (f"{ax}.홈 스트로크[count]", f"{ax}.홈 끝~끝 거리[펄스]", f"{ax}.홈 끝~끝[펄스]")
        s1, s2 = num(get(d1, *keys, default=None)), num(get(d2, *keys, default=None))
        if s1 is None and s2 is None:
            continue
        dd = None if s1 is None or s2 is None else s2 - s1
        if dd is None:
            dtxt = "—"
        elif dd == 0:
            dtxt = "0"
        else:
            dtxt = f"{dd:+,.0f} ({fx(dd / (s1 / STROKE_DEG[ax]), 2, '°', sign=True)})".replace("-", "−")
        c.metrics.append((f"{ax} 홈잉 끝에서 끝까지 거리", "펄스", comma(s1), comma(s2), dtxt, "기록만", "", None))
    return c


def _signed(d, f1: str, f2: str) -> str:
    """차이 — 보이는 두 값(반올림한 글자)의 차이. 소수 자리는 많은 쪽에 맞춘다(88 · 83 이면 −5, 날값 차이가 아니라)."""
    if d is None:
        return "—"
    try:
        d = float(f2.replace("−", "-")) - float(f1.replace("−", "-"))
    except ValueError:
        pass
    nd = max((len(x.split(".")[1]) if "." in x else 0) for x in (f1, f2))
    s = f"{abs(d):.{nd}f}"
    if float(s) == 0:
        return "0"
    return ("+" if d > 0 else "−") + s


def _unit(unit: str) -> str:
    return {"% Cur MAX": "%", "%": "%", "°": "°", "": ""}.get(unit, f" {unit}")


def recheck_view(qc1: dict, mdir1: Path | None, csv1: Path | None = None) -> StepView | None:
    """20 절 — 공회전 뒤 재검사. after.json 이 없으면 None(절을 안 만든다)."""
    c = compare(qc1, mdir1, csv1)
    if c is None:
        return None
    ov = c.overall2 if c.overall2 in RANK or c.overall2 == "중지" else (c.overall2 or "—")
    verdict = "FAIL" if ov in ("FAIL", "중지") else ("WARN" if ov == "WARN" or c.changed else ov)
    v = StepView(20, "공회전 뒤 재검사", verdict, "")
    v.what = "12시간 공회전을 끝까지 돈 제품에 QC 1~18단계를 한 번 더 돌려, 처음 검사와 견줘 나빠진 곳이 없는지 본다."
    v.how = ("공회전이 끝나면 QC 를 다시 돌리며 녹화한다. 펌웨어는 굽지 않고(3단계는 CRC 만 견준다) IP 도 쓰지 않는다. "
             "단계마다 처음 판정과 재검사 판정을, 주요 수치마다 처음 값과 재검사 값을 나란히 놓고 차이를 잰다.")
    v.why = ("공회전 동안에는 위치 신호, 따라가는 오차, 통신 왕복 시간, 카메라 초당 프레임만 판정한다. 12시간 움직인 뒤 기구가 "
             "헐거워지거나 뻑뻑해졌는지, 모터와 통신, 카메라, 소리가 처음처럼 제대로 되는지는 같은 검사를 한 번 더 해 봐야 안다.")
    v.crit = [("PASS", "재검사가 PASS 이고, 처음 검사와 견줘 크게 변한 곳이 없다."),
              ("WARN", "재검사가 WARN 이거나 크게 변한 곳이 있다. 크게 변함은 두 가지다. 단계 판정이 나빠졌다(PASS 가 WARN 이나 "
                       f"FAIL 로, WARN 이 FAIL 로). 한계가 있는 주요 수치가 한계 쪽으로 한계의 {CHANGE_FRAC * 100:.0f}% 이상 "
                       "움직였다(예: YAW 이동 중 최대 전류가 Cur MAX 의 40% 에서 65% 로)."),
              ("FAIL", "재검사가 FAIL 이거나 도중에 멈췄다."),
              ("기록만", "한계가 없는 수치(유지전류, 홈잉 끝에서 끝까지 거리 등)와 한계에서 멀어진 쪽의 변화는 적기만 한다.")]
    who = c.after.get("stem") or (c.csv2.stem if c.csv2 else "")
    if not c.csv2:
        v.line = f"재검사 {ov} — 재검사 기록({who}.csv)을 못 찾아 견주지 못함"
        v.notes.append(f"after.json 이 가리키는 재검사 CSV 가 없다 — {c.after.get('csv') or '—'}")
        return v
    head = f"재검사 {ov}"
    if c.changed:
        v.line = f"{head} · 크게 변한 곳 {len(c.changed)}곳 — " + " · ".join(c.changed[:3]) + (" …" if len(c.changed) > 3 else "")
    else:
        v.line = f"{head} · 처음 검사와 견줘 크게 변한 곳 없음"
    t = Table(["검사", "처음 → 재검사", "변함", "재검사 결과"], caption="단계 판정 — 처음과 재검사")
    for no, name, va, vb, mark, line in c.steps:
        cls = {"나빠짐": "bad" if vb in ("FAIL", "중지") else "warn", "나아짐": "ok"}.get(mark, "")
        show = line if (mark or vb not in ("PASS", "")) else ""
        arrow = f"{_vword(va) or '—'} → {_vword(vb) or '—'}"
        t.rows.append([f"{no} {name}", (arrow, _vcell(vb)[1]), (mark, cls) if mark else "",
                       (show, "muted") if show and not mark else show])
    t.note = "재검사 결과 칸은 재검사가 PASS 가 아니거나 판정이 바뀐 단계만 적는다. 단계마다 자세한 값은 아래 '재검사 단계별 자세히'에 있다."
    v.tables.append(stack_html(t))
    m = Table(["항목", "처음", "재검사", "차이", "한계", "한계에 다가간 정도 (재검사)", "변함"], num=(1, 2, 3, 5),
              caption="주요 수치 — 처음과 재검사")
    for label, unit, a, b, d, lim, mark, r2 in c.metrics:
        cls = {"크게 변함": "warn", "나아짐": "ok"}.get(mark, "")
        m.rows.append([f"{label}" + (f" [{unit}]" if unit else ""), a, b, d, (lim, "muted"), r2 or "",
                       (mark, cls) if mark else ""])
    m.note = ("차이는 재검사 값에서 처음 값을 뺀 것이다. 한계에 다가간 정도는 재검사 값이 한계에 얼마나 가까운지로, 100% 면 한계에 닿은 "
              "것이다(소리처럼 클수록 좋은 수치는 한계를 값으로 나눈다). 크게 변함은 한계 쪽으로 한계의 "
              f"{CHANGE_FRAC * 100:.0f}% 이상 움직인 것이다. 홈잉 끝에서 끝까지 거리의 괄호는 그 차이를 출력축 각도로 바꾼 "
              "값이다(끝에서 끝까지를 YAW 345°, PITCH 114° 로 본다).")
    v.tables.append(stack_html(m))
    v.notes.append(f"재검사 기록 — {c.csv2.name} · 시작 {c.started2 or '—'}")
    v._compare = c                                   # noqa: SLF001  웹이 '재검사 단계별 자세히'를 그릴 때 쓴다
    return v


def summary_rows(qc1: dict, mdir1: Path | None) -> list[StepView]:
    """요약표(검사 결과)에 붙일 20 줄 — 재검사가 없거나 읽다 틀리면 빈 목록(절은 section_html 이 따로 그린다)."""
    try:
        v = recheck_view(qc1, Path(mdir1)) if mdir1 else None
    except Exception as e:                           # 재검사 기록이 깨져도 보고서는 나와야 한다
        print(f"재검사 요약 줄을 못 만듦 — {type(e).__name__}: {e}", file=sys.stderr)
        return []
    return [v] if v else []


def _vcell(v: str):
    if v in RANK:
        return cell_v(v)
    return (_vword(v) or "—", "bad" if v == "중지" else "muted")


def recheck_details_html(v20: StepView, pill, web: bool = True) -> str:
    """재검사 1~18 단계를 처음 검사와 같은 모양으로 — 웹에서 접어 둔다. id 는 처음 검사와 겹치지 않게 바꾼다."""
    c = getattr(v20, "_compare", None)
    if not c or not c.views2:
        return ""
    from qc_steps import sections_html
    body = sections_html(c.views2, pill, web).replace('id="s-', 'id="r2-s-')
    if not web:
        return '<div class="sk-recheck"><p class="cap">재검사 단계별 자세히</p>' + body + "</div>"
    return ('<details class="fold sk-recheck"><summary>재검사 단계별 자세히 — 처음 검사와 같은 모양</summary>'
            + body + "</details>")


# ---------------------------------------------------------------- 재검사 사진
def recheck_photos(csv1: Path, ensure: bool = True, log=print) -> list[dict]:
    """재검사 녹화의 스냅샷 · 밀착 인화 — {src(절대 경로), file('recheck_…'), caption, sheet, group}.
    웹은 src 를 페이지 폴더에 file 이름으로 복사하고, PDF 는 처음 검사 폴더 기준 상대 경로로 쓴다."""
    csv1 = Path(csv1)
    csv2 = recheck_csv(csv1, read_after(qc_frames.media_dir(csv1)))
    if not csv2:
        return []
    if ensure:
        try:
            qc_frames.ensure(csv2, log=log)
        except Exception as e:                       # 사진이 없어도 보고서는 나와야 한다
            log(f"재검사 프레임 — 못 만듦 ({type(e).__name__}: {e})")
    mdir2 = qc_frames.media_dir(csv2)
    out = []
    for p in qc_frames.photo_list(mdir2):
        src = mdir2 / p["path"]
        if src.exists():
            out.append({"src": src, "path": str(src), "file": "recheck_" + p["file"], "sheet": p["sheet"],
                        "caption": "재검사(공회전 뒤) · " + p["caption"], "group": "재검사"})
    return out


# ---------------------------------------------------------------- 출하 판정 — 대시보드
def final_verdict(qc: str, soak: str | None, recheck: str | None, changed: int = 0,
                  soak_done: bool = True, soak_note: str = "") -> tuple[str, list[str]]:
    """처음 QC · 공회전 · 재검사 · 크게 변함 → 출하 판정(PASS · WARN · FAIL). 공회전이 도는 중이면 '' (아직).

    FAIL — QC 가 PASS · WARN 이 아님 · 공회전 FAIL 이나 중지 · 공회전을 끝까지 못 돎(12시간을 못 채움 — 판정 v2 개정 1 은
           이것을 중지로 내지만, 기록이 WARN 이어도 soak_done 으로 한 번 더 막는다) · 재검사 FAIL 이나 중지.
    WARN — QC WARN · 공회전 WARN(판정 v2 — 통신 지연 등) · 재검사 WARN · 전·후가 크게 변함.
    공회전 · 재검사를 안 했으면 그것은 빼고 본다. soak_note 는 공회전 판정 옆에 붙일 한 마디(soak_brief)."""
    why = []
    qc = qc or ""
    if qc not in ("PASS", "WARN"):
        return "FAIL", [f"QC {qc or '결과 없음'}"]
    out = qc
    why.append(f"QC {qc}")
    if soak in ("공회전 중", "도는 중", "기록 멈춤", "running", "stale"):
        return "", why + ["공회전 중" if soak in ("공회전 중", "도는 중", "running") else f"공회전 {soak}"]
    if soak:
        why.append(f"공회전 {soak}" + (f"({soak_note})" if soak_note else ""))
        if soak == "WARN" and soak_done:
            if out == "PASS":
                out = "WARN"
        elif soak != "PASS":
            out = "FAIL"
            if soak == "WARN":
                why.append("공회전을 끝까지 못 돌았음")
    elif recheck:                                    # 재검사는 공회전을 끝까지 돈 뒤에만 한다 — 공회전 기록을 못 읽었으면 확인하게
        why.append("공회전 기록을 못 읽음")
        if out == "PASS":
            out = "WARN"
    if recheck:
        why.append(f"재검사 {recheck}")
        if recheck in ("FAIL", "중지"):
            out = "FAIL"
        elif recheck == "WARN" and out == "PASS":
            out = "WARN"
    if changed:
        why.append(f"전·후 크게 변함 {changed}곳")
        if out == "PASS":
            out = "WARN"
    return out, why


CATEGORY_NAME = {"product": "제품 탓", "comm": "통신 탓", "env": "검사 쪽 사정"}


def run_fields(mdir: Path | None, final: str) -> dict:
    """검사 엔진(PTZ_QC)이 남긴 run.json → 갈래(category) · 검사 무효(void) · 그 까닭.

    갈래는 출하 판정이 PASS 가 아닐 때 그 까닭이 어디에 있나다 — product 제품 · comm 통신 · env 검사 환경.
    검사 환경 탓(PC 잠듦 · 사람이 멈춤 · 소리 검사 차례를 못 받음 · 주소 옮기기 실패)이면 제품을 본 것이 아니라
    검사 무효다 — 출하 판정 자리에 '무효'를 적고 PASS · FAIL 수에 넣지 않는다.
    run.json 이 없으면(창에서 사람이 돌린 검사 · 옛 기록) 빈 dict — 지금까지처럼 본다."""
    run = read_json(Path(mdir) / "run.json") if mdir else None
    if not run:
        return {}
    cat = run.get("category") or ""
    void = cat == "env"
    if not void and final and final != "PASS" and cat in ("", "none"):
        cat = "product"              # 재검사에서 크게 변한 것은 엔진이 모른다 — 보고서가 본 것이면 제품 쪽이다
    return {"category": cat if (void or final != "PASS") else "", "void": void,
            "voidWhy": str(run.get("reason") or "") if void else "",
            "recipe": run.get("recipe") or "", "criteria": (run.get("criteria") or {}).get("version") or ""}


def row_fields(qc: dict, mdir: Path | None, stage: str, csv1: Path | None = None) -> dict:
    """대시보드 한 줄에 더할 칸 — final(출하 판정) · soak(판정 v2) · soakNote · recheck · changed · finalWhy
    · category · void(검사 무효 — run_fields)."""
    s = load(mdir)
    sw = s.verdict if s else ""
    c = compare(qc, mdir, csv1)
    rw = ""
    changed = 0
    if c is not None:
        rw = c.overall2 or "—"
        changed = len(c.changed)
    qcv = qc["head"].get("종합", "")
    if stage == "interim" and not s:
        return {"final": "", "soak": "도는 중" if qcv in ("PASS", "WARN") else "", "recheck": "", "changed": 0,
                "finalWhy": f"QC {qcv} · 공회전 중"}
    note = soak_brief(s)
    fv, why = final_verdict(qcv, sw or None, rw or None, changed, soak_done=bool(s.j.get("completed")) if s else True,
                            soak_note=note)
    sw = {"공회전 중": "도는 중"}.get(sw, sw)      # 대시보드는 '공회전 도는 중'으로 적는다(낱말이 겹치지 않게)
    rf = run_fields(mdir, fv) if stage != "interim" else {}
    if rf.get("void"):                             # 검사 환경 탓 — 제품 판정이 아니다
        fv, why = "무효", [rf["voidWhy"] or "검사 쪽 사정"]
    return {"final": fv, "soak": sw, "soakNote": note,
            "soakDone": bool(s.j.get("completed")) if s else True,   # 판정 고침(qc_fix.recount)이 같은 셈을 다시 돌릴 때
            "soakOutliers": (len(s.out_rows) if s and s.rows is not None else None),
            "recheck": rw, "changed": changed, "finalWhy": " · ".join(why),
            "category": rf.get("category", ""), "void": bool(rf.get("void")),
            "recipe": rf.get("recipe", ""), "criteria": rf.get("criteria", "")}


# ---------------------------------------------------------------- 모양
SOAK_CSS = """
.sk-live { margin: 0 0 10px; padding: 8px 12px; border: 1px dashed var(--accent, #0f6b74); border-radius: 6px;
           color: var(--accent, #0f6b74); font-size: 13px; max-width: 86ch; }
.sk-live.bad { border-color: var(--fail, #c0392b); color: var(--fail, #c0392b); }
.sk-charts { margin-top: 14px; }
.sk-guide, .sk-gd, .sk-hint { margin: 0 0 8px; font-size: 13px; color: var(--muted, #5b676b); max-width: 100ch; }
.sk-key { display: flex; flex-wrap: wrap; gap: 4px 16px; margin: 0 0 10px; font-size: 12.5px; color: var(--muted, #5b676b); }
.sk-key span { display: inline-flex; align-items: center; gap: 6px; }
.sk-key i { display: inline-block; width: 18px; height: 10px; }
.sk-key .k-line { height: 0; border-top: 2px solid var(--accent, #0f6b74); }
.sk-key .k-band { background: var(--accent, #0f6b74); opacity: .18; }
.sk-key .k-lim { height: 0; border-top: 1.5px dashed var(--ink, #1c2326); }
.sk-key .k-out { width: 3px; height: 13px; background: var(--fail, #c0392b); }
.sk-key .k-lat { width: 3px; height: 7px; background: var(--warn, #a86400); }
.sk-key .k-base { background: var(--muted, #5b676b); opacity: .14; }
.sk-group { margin: 14px 0 6px; }
.sk-gt { margin: 0 0 2px; font-size: 14px; }
.sk-grid-1 { display: grid; grid-template-columns: minmax(0, 1fr); }
.sk-grid-2 { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px 22px; }
.sk-panel { min-width: 0; }
.sk-ph { margin: 4px 0 4px; font-size: 13px; }
.sk-ph span { color: var(--muted, #5b676b); }
.sk-plot { display: grid; grid-template-columns: 3.4em minmax(0, 1fr); grid-template-rows: auto auto; outline-offset: 3px; }
.sk-yax { position: relative; height: 130px; font: 11px/1 var(--mono, monospace); color: var(--muted, #5b676b); }
.sk-yax span { position: absolute; right: 6px; transform: translateY(-50%); white-space: nowrap; }
.sk-area { position: relative; height: 130px; border-left: 1px solid var(--line, #d6dddb); border-bottom: 1px solid var(--line, #d6dddb); }
.sk-svg { position: absolute; inset: 0; width: 100%; height: 100%; overflow: visible; }
.sk-grid { stroke: var(--line, #d6dddb); stroke-width: 1; vector-effect: non-scaling-stroke; }
.sk-band { fill: var(--accent, #0f6b74); fill-opacity: .18; stroke: none; }
.sk-line { fill: none; stroke: var(--accent, #0f6b74); stroke-width: 2; vector-effect: non-scaling-stroke; stroke-linejoin: round; stroke-linecap: round; }
.sk-lim { stroke: var(--ink, #1c2326); stroke-width: 1.5; stroke-dasharray: 5 4; vector-effect: non-scaling-stroke; opacity: .75; }
.sk-out { stroke: var(--fail, #c0392b); stroke-width: 2; vector-effect: non-scaling-stroke; }
.sk-lat { stroke: var(--warn, #a86400); stroke-width: 2; vector-effect: non-scaling-stroke; }
.sk-base { fill: var(--muted, #5b676b); fill-opacity: .12; }
.sk-rest { fill: var(--line, #d6dddb); fill-opacity: .45; }
.sk-now { stroke: var(--accent, #0f6b74); stroke-width: 1; stroke-dasharray: 2 3; vector-effect: non-scaling-stroke; }
.sk-limlab, .sk-baselab, .sk-nowlab { position: absolute; font: 11px/1.2 var(--font, sans-serif); color: var(--ink, #1c2326);
           background: var(--surface, #fff); padding: 0 3px; border-radius: 3px; white-space: nowrap; pointer-events: none; }
.sk-limlab { right: 2px; transform: translateY(-115%); }
.sk-limlab.lo { transform: translateY(15%); }
.sk-baselab { left: 3px; top: 3px; color: var(--muted, #5b676b); background: transparent; }
.sk-nowlab { top: 3px; transform: translateX(-50%); color: var(--accent, #0f6b74); }
.sk-xax { grid-column: 2; position: relative; height: 18px; font: 11px/1 var(--font, sans-serif); color: var(--muted, #5b676b); }
.sk-xax span { position: absolute; top: 5px; transform: translateX(-50%); white-space: nowrap; }
.sk-xax span:first-child { transform: none; }
.sk-xax span:last-child { transform: translateX(-100%); }
.sk-over { margin: 2px 0 0; font-size: 12px; color: var(--muted, #5b676b); }
.sk-cross { position: absolute; top: 0; bottom: 0; width: 0; border-left: 1px solid var(--ink, #1c2326); opacity: .55; pointer-events: none; }
.sk-tip { position: fixed; z-index: 20; pointer-events: none; background: var(--surface, #fff); color: var(--ink, #1c2326);
          border: 1px solid var(--line, #d6dddb); border-radius: 6px; padding: 7px 10px; font-size: 12.5px; line-height: 1.45;
          box-shadow: 0 4px 14px rgba(0, 0, 0, .14); max-width: 260px; }
.sk-tip .t { color: var(--muted, #5b676b); font-size: 12px; }
.sk-tip .v { font: 600 14px/1.3 var(--mono, monospace); }
.sk-tip .o { color: var(--fail, #c0392b); font-weight: 600; }
.sk-tip .w { color: var(--warn, #a86400); font-weight: 600; }
.sk-tip i { display: inline-block; width: 12px; border-top: 2px solid var(--accent, #0f6b74); vertical-align: middle; margin-right: 5px; }
.sk-recheck { margin-top: 12px; }
section.st.sk-cont { border-top: 0; padding-top: 0; margin-top: 6px; }
section.st table.mt td.sk-nw { white-space: nowrap; }
.sk-recheck > summary { cursor: pointer; font-weight: 600; font-size: 13px; }
@media (max-width: 640px) {
  .sk-grid-2 { grid-template-columns: minmax(0, 1fr); }
  .sk-yax, .sk-area { height: 118px; }
  .sk-plot { grid-template-columns: 3em minmax(0, 1fr); }
  .sk-xax span:nth-child(even) { display: none; }
  section.st table.mt.sk-stack { min-width: 0; width: 100%; }
  section.st table.mt.sk-stack thead { display: none; }
  section.st table.mt.sk-stack, section.st table.mt.sk-stack tbody { display: block; }
  section.st table.mt.sk-stack tr { display: block; padding: 8px 0; border-bottom: 1px solid var(--line, #d6dddb); }
  section.st table.mt.sk-stack td { display: grid; grid-template-columns: 7.5em minmax(0, 1fr); gap: 0 10px; border: 0;
                                    padding: 1px 0; text-align: left; white-space: normal; }
  section.st table.mt.sk-stack td::before { content: attr(data-l); color: var(--muted, #5b676b); font-size: 12px; font-weight: 400; }
  section.st table.mt.sk-stack td:first-child { display: block; font-weight: 600; padding-bottom: 3px; }
  section.st table.mt.sk-stack td:first-child::before { content: none; }
  section.st table.mt.sk-stack td:empty { display: none; }
}
@media print {
  .sk-hint { display: none; }
  .sk-xax span:nth-child(even) { display: none; }
  section.st p.cap { break-after: avoid; }
  .sk-yax, .sk-area { height: 110px; }
  .sk-group, .sk-panel { break-inside: avoid; }
}
"""

# 웹만 — 그래프 위 십자선과 풍선 도움말. 값은 페이지에 넣어 둔 JSON(.sk-data)에서 읽고 글자는 textContent 로만 넣는다
SOAK_JS = r"""
(function () {
  var boxes = document.querySelectorAll('script.sk-data');
  if (!boxes.length) return;
  var tip = document.createElement('div'); tip.className = 'sk-tip'; tip.hidden = true; document.body.appendChild(tip);
  function two(x) { return (x < 10 ? '0' : '') + x; }
  function el(s) {                                  // 초 → '3시간 12분 5초'(qc_steps.dur 와 같은 모양)
    s = Math.max(0, Math.round(s));
    var h = Math.floor(s / 3600), m = Math.floor(s % 3600 / 60), x = s % 60, o = [];
    if (h) o.push(h + '시간'); if (m) o.push(m + '분'); if (x || !o.length) o.push(x + '초');
    return o.join(' ');
  }
  function row(cls, text) { var p = document.createElement('div'); p.className = cls; p.textContent = text; return p; }
  Array.prototype.forEach.call(boxes, function (box) {
    var D; try { D = JSON.parse(box.textContent); } catch (e) { return; }
    var t0 = D.t0 ? Date.parse(D.t0.replace(' ', 'T')) : NaN;
    function clock(sec) {
      if (isNaN(t0)) return '';
      var d = new Date(t0 + sec * 1000);
      return two(d.getHours()) + ':' + two(d.getMinutes()) + (D.b < 60 ? ':' + two(d.getSeconds()) : '');
    }
    var root = box.parentNode;
    Array.prototype.forEach.call(root.querySelectorAll('.sk-plot'), function (plot) {
      var P = D.p[plot.getAttribute('data-k')];
      if (!P) return;
      var area = plot.querySelector('.sk-area'), cross = plot.querySelector('.sk-cross'), cur = -1;
      function show(k, x, y) {
        k = Math.max(0, Math.min(D.n - 1, k)); cur = k;
        cross.hidden = false; cross.style.left = (100 * (k + 0.5) / D.n) + '%';
        tip.textContent = '';
        var a = k * D.b, b = a + D.b;
        var when = clock(a) ? clock(a) + (D.b > 1 ? ' ~ ' + clock(b) : '') + ' · ' : '';
        tip.appendChild(row('t', when + '시작하고 ' + el(a)));
        var name = document.createElement('div'); var key = document.createElement('i'); name.appendChild(key);
        name.appendChild(document.createTextNode(P.l)); tip.appendChild(name);
        var av = P.av[k], mn = P.mn[k], mx = P.mx[k];
        if (av == null) tip.appendChild(row('t', '이 칸은 기록이 없다'));
        else if (D.b === 1 || mn === mx) tip.appendChild(row('v', mn.toFixed(P.d) + ' ' + P.u));
        else {
          tip.appendChild(row('v', av.toFixed(P.d + 1) + ' ' + P.u + ' 평균'));
          tip.appendChild(row('t', '가장 작은 값 ' + mn.toFixed(P.d) + ' · 가장 큰 값 ' + mx.toFixed(P.d)));
        }
        var o = P.o[String(k)];                     // [통신 지연 탓, 진짜 이상] 1초 수
        if (o && o[0]) tip.appendChild(row('w', '통신 지연 탓 ' + o[0] + '초'));
        if (o && o[1]) tip.appendChild(row('o', '진짜 이상 ' + o[1] + '초'));
        tip.hidden = false;
        var w = tip.offsetWidth, h = tip.offsetHeight;
        var left = x + 14, top = y - h - 10;
        if (left + w > window.innerWidth - 8) left = x - w - 14;
        if (left < 8) left = 8;
        if (top < 8) top = y + 16;
        tip.style.left = left + 'px'; tip.style.top = top + 'px';
      }
      function hide() { cross.hidden = true; tip.hidden = true; }
      function fromX(e) { var r = area.getBoundingClientRect(); return Math.floor((e.clientX - r.left) / r.width * D.n); }
      area.addEventListener('pointermove', function (e) { show(fromX(e), e.clientX, e.clientY); });
      area.addEventListener('pointerdown', function (e) { show(fromX(e), e.clientX, e.clientY); });
      area.addEventListener('pointerleave', hide);
      plot.addEventListener('blur', hide);
      plot.addEventListener('keydown', function (e) {
        var step = e.shiftKey ? 10 : 1, k = cur < 0 ? D.n - 1 : cur;
        if (e.key === 'ArrowLeft') k -= step; else if (e.key === 'ArrowRight') k += step;
        else if (e.key === 'Home') k = 0; else if (e.key === 'End') k = D.n - 1;
        else if (e.key === 'Escape') { hide(); return; } else return;
        e.preventDefault();
        var r = area.getBoundingClientRect();
        k = Math.max(0, Math.min(D.n - 1, k));
        show(k, r.left + r.width * (k + 0.5) / D.n, r.top + 10);
      });
      plot.addEventListener('focus', function () {
        var r = area.getBoundingClientRect(), k = cur < 0 ? 0 : cur;
        show(k, r.left + r.width * (k + 0.5) / D.n, r.top + 10);
      });
    });
  });
})();
"""


# ---------------------------------------------------------------- 확인용
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="처음 QC CSV (<시리얼>_<시각>.csv)")
    ap.add_argument("--stage", choices=("interim", "final"), default=None, help="보고서 단계(모르면 비움 — PDF 처럼)")
    ap.add_argument("--html", type=Path, help="19 · 20 절만 그린 미리보기 페이지")
    a = ap.parse_args()
    from qc_report import read_qc_csv
    from qc_steps import STEPS_CSS, build, sections_html
    csv1 = a.csv.resolve()
    qc = read_qc_csv(csv1)
    mdir = csv1.with_suffix("")
    t0 = time.time()
    v19 = next(v for v in build(qc, mdir, a.stage) if v.no == 19)
    s = load(mdir)
    print(f"19 {v19.verdict:<6} {v19.line}")
    if s:
        print(f"   기준 구간 {s.base_s} s · 줄 {s.after['rows']} · 기준을 벗어난 1초(옛 셈) {len(s.out_rows)} · 계획 {s.extent} s")
        if s.jd:
            jd = s.jd
            print(f"   판정 v2 {jd.verdict or '—'} ({jd.source}) · 옛 기준 {jd.old or '—'} · 평소 왕복 {jd.rtt_base:.2f} ms · "
                  + " · ".join(f"{k} {v}" for k, v in jd.st.items()))
            if any(jd.n.values()):
                print("   1초 수 " + " · ".join(f"{k} {v}" for k, v in jd.n.items() if v))
            if jd.diff:
                print("   soak.json 과 다름 — " + " · ".join(jd.diff))
    c = compare(qc, mdir, csv1)
    v20 = recheck_view(qc, mdir, csv1) if c else None
    if v20:
        print(f"20 {v20.verdict:<6} {v20.line}")
    rf = row_fields(qc, mdir, a.stage or "final", csv1)
    print(f"출하 판정 {rf['final'] or '(아직)'} — {rf['finalWhy']}")
    if a.html:
        import qc_web
        body = '<main class="wrap">' + sections_html([v19], _pill_web, web=True) + section_html(mdir, True, a.stage) + "</main>"
        a.html.write_text(qc_web.page("공회전 미리보기", qc_web.PRODUCT_CSS + STEPS_CSS, body, "", True), encoding="utf-8")
        print(f"HTML {a.html} ({a.html.stat().st_size / 1024:.0f} KB)")
    print(f"걸린 시간 {time.time() - t0:.2f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
