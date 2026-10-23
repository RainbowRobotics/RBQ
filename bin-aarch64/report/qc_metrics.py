#!/usr/bin/env python3
"""QC 결과 한 건에서 제품끼리 견줄 주요 수치를 뽑는다 — PDF · 제품 페이지 · 대시보드가 같이 쓴다.

ui2 사본 — 셈은 원래 tools/qc_report/qc_metrics.py 그대로, 사람이 읽는 이름표만 다듬었다(09-29 "스트로크 같은 말 어색").

    python3 qc_metrics.py <QC 보고서 CSV> [...]     # 여러 건을 표 하나로(추세 보기)

판정(PASS/WARN/FAIL)은 QC 탭이 이미 내렸다. 여기 수치는 판정을 바꾸지 않고, 한계에 얼마나
가까웠는지를 남겨 제품끼리 · 시간에 따라 견주게 한다(09-29 사용자: "그런 기록들은 대시보드나
pdf 만들 때 참고하는 걸로"). 한계값은 QC 탭 기준(qctab.cpp QcLimit · qcgwload.h QcGwLimit)을 옮겼다.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc_report import read_qc_csv  # noqa: E402
from qc_steps import get, parse_move  # noqa: E402  이동 한 줄 · 이름이 바뀐 키 — 보고서 절과 같은 읽기

CUR_SAT_RATIO = 0.95        # YAW 이동 전류 / Cur MAX — 넘으면 QC 가 WARN(걸림 의심)
PITCH_CUR_WARN_A = 5.0      # PITCH 이동 전류 [A] — 넘으면 QC 가 WARN(10-08 결정, qc/03 §7 — Cur MAX 와 상관없이)
MOVE_ERR_PASS = 0.25        # 정착 오차 [deg]
RTT_P99_WARN_MS = 100.0     # 부하 시험 L1 왕복 p99 [ms]
MIC_PASS_DB = 20.0          # 스피커→마이크 두 톤 상승 [dB]
_MOVE = re.compile(r"^(\d+) (YAW|PITCH)→")      # 세부 키 "9 YAW→-90.0°"


def _num(pattern: str, text: str) -> float | None:
    m = re.search(pattern, text or "")
    return float(m.group(1)) if m else None


def key_metrics(qc: dict) -> dict:
    """CSV 한 건 → {이름: 값}. 값이 없으면(단계를 안 돌았음) 그 칸은 None."""
    d = dict(qc["details"])
    out: dict = {}
    for ax in ("YAW", "PITCH"):
        cur_max = _num(r"([0-9.]+)", d.get(f"{ax}.CFG4 Cur MAX", ""))
        peak, peak_at, err = None, "", None
        for k, v in qc["details"]:
            m = _MOVE.match(k)
            if not m or m.group(2) != ax:
                continue
            mv = parse_move(v) or {}
            i, e = mv.get("i"), mv.get("err")
            # 소프트 리밋(10)의 끝까지 밀기는 일부러 막히는 이동이라 멈춘 위치 오차에서 뺀다
            if e is not None and m.group(1) != "10":
                err = e if err is None else max(err, e)
            if i is not None and (peak is None or i > peak):
                peak, peak_at = i, k.split(" ", 1)[1]
        out[f"{ax.lower()}CurA"] = peak
        out[f"{ax.lower()}CurAt"] = peak_at
        out[f"{ax.lower()}CurPct"] = round(100.0 * peak / cur_max, 1) if peak is not None and cur_max else None
        out[f"{ax.lower()}HoldA"] = _num(r"([0-9.]+)", d.get(f"{ax}.유지전류[A]", ""))
        out[f"{ax.lower()}SettleDeg"] = err
        st = _num(r"([0-9.]+)", get(d, f"{ax}.홈 스트로크[count]", f"{ax}.홈 끝~끝 거리[펄스]", f"{ax}.홈 끝~끝[펄스]"))
        th = _num(r"([0-9.]+)", get(d, f"{ax}.홈 임계[count]", f"{ax}.홈 최소 기준[펄스]"))
        out[f"{ax.lower()}HomeX"] = round(st / th, 3) if st and th else None
    out["gwL1p99"] = _num(r"왕복 p50/p99/최대 [0-9.]+/([0-9.]+)/", d.get("게이트웨이부하.L1 +100/s", ""))
    out["gwLossless"] = _num(r"([0-9.]+)", get(d, "게이트웨이부하.무손실 상한", "게이트웨이부하.손실 없이 버틴 최대 부하"))
    out["monP99"] = _num(r"^[0-9.]+/([0-9.]+)/", d.get("통신감시.왕복 게이트웨이 p50/p99/최대[ms]", ""))
    rises = [_num(r"/ ([+-]?[0-9.]+) \(", d.get(f"스피커→마이크.{hz} Hz 배경/톤/상승[dB]", "")) for hz in (1000, 2000)]
    rises = [r for r in rises if r is not None]
    out["micMinDb"] = min(rises) if rises else None
    crit = " ".join(st.get("criterion") or "" for st in qc["steps"])
    out["pitchRule"] = "A" if re.search(r"PITCH\s*<\s*[0-9.]+\s*A", crit) else "pct"
    return out


# 사람이 보는 표 — (이름, 칸 이름, 단위, 한계 설명, 한계에 다가간 정도: 값 → 1 이면 한계). 비율이 None 인 줄은 셈하지 않는다.
# 홈잉 끝~끝 거리 배수(최소 기준이 설계 거리의 95% 라 늘 1.04~1.08)와 손실 없이 버틴 부하(단계 폭 때문에 늘 500)는
# 한계와 붙어 있는 구조라 "가까움"을 붙이면 늘 뜬다 — 값만 남겨 제품끼리 견준다(판정은 7 · 16단계가 한다).
# '≥' 한계(소리)는 한계 ÷ 값으로 셈해 다른 줄과 눈금 방향을 맞춘다 — 100% 를 넘으면 한계 밖.
ROWS = [
    ("pitchCurA",   "PITCH 이동 중 최대 전류", "A", f"< {PITCH_CUR_WARN_A:g} A",
     lambda v: v / PITCH_CUR_WARN_A),
    ("yawCurPct",   "YAW 이동 중 최대 전류 (Cur MAX 대비)",   "%", f"< {CUR_SAT_RATIO * 100:.0f}%",
     lambda v: v / (CUR_SAT_RATIO * 100)),
    ("pitchHoldA",  "PITCH 유지전류 (멈춰 있을 때)", "A",   "기록만",                          None),
    ("pitchHomeX",  "PITCH 홈잉 끝에서 끝까지 거리 (최소 기준 대비)", "배", "1배 넘어야 홈잉 성공", None),
    ("yawHomeX",    "YAW 홈잉 끝에서 끝까지 거리 (최소 기준 대비)",   "배", "1배 넘어야 홈잉 성공", None),
    ("pitchSettleDeg", "PITCH 멈춘 위치 오차 (가장 큰 것)", "°", f"≤ {MOVE_ERR_PASS}°",       lambda v: v / MOVE_ERR_PASS),
    ("yawSettleDeg",   "YAW 멈춘 위치 오차 (가장 큰 것)",   "°", f"≤ {MOVE_ERR_PASS}°",       lambda v: v / MOVE_ERR_PASS),
    ("gwL1p99",     "게이트웨이 왕복 시간 p99 (평소 부하)", "ms", f"≤ {RTT_P99_WARN_MS:.0f} ms", lambda v: v / RTT_P99_WARN_MS),
    ("gwLossless",  "게이트웨이가 손실 없이 버틴 추가 요청", "번/초", "≥ 500 번/초 (못 미치면 WARN)", None),
    ("monP99",      "검사 내내 게이트웨이 왕복 시간 p99", "ms", "기록만",                         None),
    ("micMinDb",    "스피커 → 마이크 배경보다 커진 소리 (두 소리 중 약한 쪽)", "dB",  f"≥ {MIC_PASS_DB:.0f} dB",
     lambda v: MIC_PASS_DB / v if v and v > 0 else 9.99),
]
# 10-08 전 검사는 PITCH 도 YAW 처럼 Cur MAX 비율로 갈랐다 — 그 검사의 표는 그때 기준으로 그린다
PITCH_PCT_ROW = ("pitchCurPct", "PITCH 이동 중 최대 전류 (Cur MAX 대비)", "%", f"< {CUR_SAT_RATIO * 100:.0f}%",
                 lambda v: v / (CUR_SAT_RATIO * 100))
NEAR = 0.85                 # 한계의 85% 이상이면 "가까움", 한계에 닿거나 넘으면 "한계 밖"
NOTE = ("이동 중 최대 전류는 9 · 10 · 15단계 이동 전부에서, 멈춘 위치 오차는 9 · 15단계에서 가장 큰 값이다. "
        "PITCH 전류는 Cur MAX 와 상관없이 5 A 를 한계로 본다. 10-08 에 바뀐 기준이라, 그 전에 돈 검사는 그때처럼 Cur MAX "
        "비율로 적는다. 왕복 시간은 PC 가 확인 신호를 보내고 응답을 받기까지 걸린 시간이고, p99 는 100번 가운데 99번이 이보다 "
        "빠른 값이다. 한계에 다가간 정도가 85% 이상이면 '가까움', 한계에 닿거나 넘으면 '한계 밖'이 붙는다.")

# 대시보드 한 줄에 싣는 것(runs 문서의 metrics 칸)
DASH_KEYS = ("pitchCurA", "pitchCurPct", "yawCurPct", "pitchHomeX", "yawHomeX", "gwL1p99", "micMinDb")


def rows_for(m: dict) -> list:
    """이 검사에 맞는 표 줄 — 10-08 전 검사(pitchRule 'pct')는 PITCH 전류를 그때 기준(Cur MAX 비율)으로."""
    if m.get("pitchRule") == "pct":
        return [PITCH_PCT_ROW if r[0] == "pitchCurA" else r for r in ROWS]
    return list(ROWS)


def out_of_limit(lim: str, r: float) -> bool:
    """한계 밖인가 — '<' 한계는 닿기만 해도 밖(QC 탭은 그 값부터 WARN), '≤' · '≥' 한계는 넘어야 밖."""
    return r >= 1.0 if lim.lstrip().startswith("<") else r > 1.0


def table_rows(m: dict) -> list[tuple[str, str, str, str, str]]:
    """(칸 이름, 값, 단위, 한계, '가까움'·'한계 밖'·'') — PDF 와 제품 페이지가 같은 표를 그린다. 값이 없으면 단위도 비운다."""
    rows = []
    for key, label, unit, lim, ratio in rows_for(m):
        v = m.get(key)
        r = ratio(v) if (ratio and v is not None) else None
        word = "" if r is None else ("한계 밖" if out_of_limit(lim, r) else "가까움" if r >= NEAR else "")
        pct = "" if r is None else f"{r * 100:.0f}%"
        rows.append((label, fmt(key, v), unit if v is not None else "", lim, f"{pct} {word}".strip()))
    return rows


def flagged(word: str) -> bool:
    """한계 대비 칸에 '가까움'이나 '한계 밖'이 붙었나 — 페이지는 주황, PDF 는 굵게."""
    return "가까움" in word or "한계 밖" in word


def dash_metrics(m: dict) -> dict:
    return {k: m.get(k) for k in DASH_KEYS}


def fmt(key: str, v) -> str:
    if v is None:
        return "—"
    if key.endswith("Pct"):
        return f"{v:.0f}"
    if key.endswith("HomeX"):
        return f"{v:.2f}"
    if key in ("gwLossless",):
        return f"{v:.0f}"
    return f"{v:.2f}" if isinstance(v, float) and abs(v) < 10 else f"{v:.1f}" if isinstance(v, float) else str(v)


def main() -> int:
    paths = [Path(p) for p in sys.argv[1:]]
    if not paths:
        print(__doc__)
        return 2
    table = [(p.stem, key_metrics(read_qc_csv(p))) for p in paths]
    print("항목".ljust(26) + "".join(s[:22].rjust(24) for s, _ in table))
    for key, label, unit, lim, _ in ROWS:
        print(f"{label} [{unit}]".ljust(26) + "".join(fmt(key, m.get(key)).rjust(24) for _, m in table))
    return 0


if __name__ == "__main__":
    sys.exit(main())
