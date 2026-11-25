#!/usr/bin/env python3
"""로봇 앱 'PTZ 점검' 보고서 — 웹 제품 페이지(qc_web.product_html)와 같은 절 · 같은 순서를 JSON(한 · 영)으로 낸다.

    python3 qc_export.py <QC CSV> [--stage interim|final]        <검사 폴더>/report.json + <QC 폴더>/index.json
    python3 qc_export.py finalize --status status.json --why 글    끝 보고 없이 끊긴 로봇 검사를 '무효'로 마무리
    python3 qc_export.py prune --dir <QC 폴더> --keep 20           오래된 보고서부터 지워 N 건을 남긴다
    python3 qc_export.py index --dir <QC 폴더>                     index.json(이력 목록)만 다시 만든다

글 · 표 · 판정은 qc_steps.build · qc_soak · qc_metrics 그대로다(PDF · 웹과 같은 모듈). 여기는 HTML 대신 JSON 으로
옮기고, 로봇에만 있는 것(다시 돌린 기록 · 로봇 움직임 · 고객 문구 · 이력 목록 · 보관 20건)을 더한다. 영어는 qc_en.py.

로봇에는 웹 서버 · PDF · 클로드가 없다(PTZ docs/qc/03 §6). 앱이 command 통로로 이 JSON 과 사진 · 녹화를 받아 그린다.
판정 글자(PASS · WARN · FAIL · SKIP · 무효 …)는 한국어 그대로 두고 앱이 언어에 맞춰 보인다.

report.json 모양(v 1) — 앱(es gui src/lib/ptzCheck.ts)이 이 모양을 읽는다. 바꾸면 거기도.
  head      시리얼 · UID · 버전 · 시작 · 끝 · 레벨 · 종류 · 판정 도장(verdict · why · customer)
  clips     녹화 — 9 · 10 단계 30초(QC · 재검사) · 공회전 끝 30초. 단계마다 녹화 기준 초
  summary   단계 1~20 요약표 — 다시 돌린 단계는 시도 수와 마지막 판정
  metrics   주요 수치 표
  steps     단계별 자세히(무엇을 · 어떻게 · 왜 · 기준 · 표 · 건너뛴 까닭) — 표의 levelMin 은 그 표를 보일 최소 레벨
  soak      19 공회전 — 한눈에 표 · 항목별 표 · 그래프 데이터(1초를 묶은 칸) · 진짜 이상 · 사건
  recheck   20 재검사 — 전 · 후 비교 표
  reruns    단계 다시 돌린 기록 — 시도마다 단계 · 판정 · 측정값 · 조건(R1~R9)
  photos    사진 파일
  appendix  QC 탭이 남긴 세부 값(원본)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import qc_en  # noqa: E402
import qc_soak  # noqa: E402
from qc_metrics import NOTE as METRIC_NOTE  # noqa: E402
from qc_metrics import key_metrics, table_rows  # noqa: E402
from qc_report import read_qc_csv  # noqa: E402
from qc_steps import TITLE, StepView, Table, build, step_no  # noqa: E402

V = 1
KEEP_DEFAULT = 20
RANK = {"PASS": 1, "WARN": 2, "FAIL": 3}
# 고객 문구(D18) — 판정 도장 옆 한 줄
CUSTOMER = {
    "WARN": ("확인 필요 — 사용은 가능, 다음 점검 때 지원팀과 확인", "Needs a check — usable; review it with support at the next service"),
    "FAIL": ("지원팀 문의", "Contact support"),
    "무효": ("검사 무효 — 제품 판정이 아닙니다. 다시 검사하세요", "Test invalid — not a product result. Run the check again"),
}
# 17단계 진단 카운터 원시값 표만 레벨 2 이상(qc/03 §6)
LEVEL2_TABLES = {17}


def bi(ko, en=None) -> dict:
    ko = "" if ko is None else str(ko)
    return {"ko": ko, "en": en if en else None}


def now() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def read_json(p: Path) -> dict:
    try:
        j = json.loads(Path(p).read_text(encoding="utf-8"))
        return j if isinstance(j, dict) else {}
    except (OSError, ValueError):
        return {}


def write_json(p: Path, obj) -> None:
    """원자적으로 — 앱이 쓰는 도중의 파일을 읽지 않게."""
    p = Path(p)
    p.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".tmp_", dir=str(p.parent))
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, separators=(",", ":"))
    os.chmod(tmp, 0o644)                       # mkstemp 는 0600 — 보고서를 내주는 쪽(Streamer)이 읽게
    os.replace(tmp, p)


def media_of(csv_path: Path) -> Path:
    return csv_path.with_suffix("")


# ---------------------------------------------------------------- 칸 · 표 · 단계
def cell_json(c) -> dict:
    text, cls = (c if isinstance(c, tuple) else (c, ""))
    text = "" if text is None else str(text)
    return {"t": text, "en": qc_en.cell(text), "c": cls or ""}


def table_json(t, no: int = 0) -> dict | None:
    """Table → dict. HTML 조각(str — 그래프 · 스타일)은 앱이 못 그린다 — 데이터는 따로 낸다(soak.series)."""
    if not isinstance(t, Table):
        return None
    return {"cols": [bi(c, qc_en.col(c)) for c in t.cols],
            "rows": [[cell_json(c) for c in r] for r in t.rows],
            "num": list(t.num),
            "caption": bi(t.caption, qc_en.text(t.caption)) if t.caption else None,
            "note": bi(t.note, qc_en.text(t.note)) if t.note else None,
            "fold": bi(t.fold, qc_en.text(t.fold)) if t.fold else None,
            "levelMin": 2 if no in LEVEL2_TABLES else 1}


def tables_of(v: StepView) -> list:
    """v.tables 의 Table — stack_html 로 싼 것(qc_soak)은 안의 Table 을 꺼낸다."""
    out = []
    for t in v.tables:
        if isinstance(t, Table):
            out.append(t)
        elif hasattr(t, "_table"):
            out.append(t._table)                                   # noqa: SLF001
    return out


def step_json(v: StepView, qc: dict, L: dict, moving: set) -> dict:
    en = qc_en.step(v.no, L, dict(qc["details"]) if qc else {})
    tabs = [x for x in (table_json(t, v.no) for t in tables_of(v)) if x]
    return {"no": v.no, "name": bi(v.name, qc_en.title(v.no, v.name)), "verdict": v.verdict,
            "line": bi(v.line, qc_en.line(v.no, v.line)),
            "skip": bi(v.skip, qc_en.text(v.skip)) if v.skip else None,
            "what": bi(v.what, en.get("what")) if v.what else None,
            "how": bi(v.how, en.get("how")) if v.how else None,
            "why": bi(v.why, en.get("why")) if v.why else None,
            "crit": [{"tag": k, "text": bi(txt, (en.get("crit") or {}).get(k) or qc_en.text(txt))} for k, txt in v.crit],
            "tables": tabs,
            "notes": [bi(n, qc_en.text(n)) for n in v.notes if n],
            "moving": v.no in moving}


# ---------------------------------------------------------------- 공회전 그래프 데이터 — 앱이 react-native-svg 로 그린다
def soak_series(s) -> dict | None:
    if not s or not s.rows or not s.rows["sec"]:
        return None
    b = qc_soak.bucket_size(s.extent)
    n = int(math.ceil(s.extent / b))
    out = []
    for p in qc_soak.panels(s):
        r = s.cur if p.group == "cur" else s.rows
        mn, av, mx = qc_soak.aggregate(r["sec"], r[p.key], b, n)
        lat_keys, real_keys = qc_soak.PANEL_MARKS[p.key]
        lat, real = {}, {}
        for sec, m in (s.jd.marks.items() if s.jd else ()):
            k = sec // b
            if not 0 <= k < n:
                continue
            if m & real_keys:
                real[k] = real.get(k, 0) + 1
            elif m & lat_keys:
                lat[k] = lat.get(k, 0) + 1

        def rnd(xs):
            return [None if x is None else round(x, max(p.nd, 1) + 1) for x in xs]
        out.append({"key": p.key, "group": p.group, "who": bi(p.who, qc_en.cell(p.who)), "unit": p.unit,
                    "lo": p.lo, "hi": p.hi, "limFrom": p.lim_from,
                    "min": rnd(mn), "avg": rnd(av), "max": rnd(mx),
                    "lat": sorted(lat.items()), "real": sorted(real.items())})
    groups = [{"key": g, "title": bi(t, qc_en.text(t)), "unit": u, "desc": bi(d, qc_en.text(d))} for g, t, u, d in qc_soak.GROUPS]
    return {"bucketS": b, "n": n, "extentS": s.extent, "lastS": s.last, "baseS": s.base_s,
            "startedAt": s.j.get("startedAt") or "", "groups": groups, "panels": out}


def soak_json(v19: StepView | None, mdir: Path, qc: dict, L: dict) -> dict | None:
    s = qc_soak.load(mdir)
    if v19 is None:
        return None
    base = step_json(v19, qc, L, set())
    en = qc_en.soak_crit(s.j if s else None)              # 19 기준은 soak.json 의 숫자로(qc_soak.crit_v2 와 같다)
    for c in base["crit"]:
        c["text"]["en"] = en.get(c["tag"]) or c["text"]["en"]
    base["series"] = soak_series(s)
    base["state"] = s.state if s else ""
    cap = qc_soak.cur_cap(s.j) if s else None
    base["curCap"] = cap
    clip = (s.j.get("clip") if s else None) or {}
    base["endClip"] = {"dir": clip.get("dir") or "soak_clip", "ok": bool(clip.get("ok")), "seconds": clip.get("seconds") or 0}
    return base


# ---------------------------------------------------------------- 녹화 · 사진
CAMS = (("zoom", "줌", "Zoom"), ("thermal", "열화상", "Thermal"))


def remux(src: Path) -> Path:
    """mkv → mp4(다시 인코딩하지 않고 그릇만 바꾼다 — 앱 · 브라우저가 바로 튼다). ffmpeg 가 없거나 실패하면 mkv 그대로."""
    dst = src.with_suffix(".mp4")
    if dst.exists() and dst.stat().st_mtime >= src.stat().st_mtime:
        return dst
    ff = shutil.which("ffmpeg")
    if not ff:
        return src
    try:
        r = subprocess.run([ff, "-v", "error", "-y", "-i", str(src), "-c", "copy", "-an", "-movflags", "+faststart", str(dst)],
                           capture_output=True, timeout=120)
        return dst if r.returncode == 0 and dst.exists() and dst.stat().st_size > 0 else src
    except (OSError, subprocess.SubprocessError):
        return src


def clip_file(mkv: Path) -> Path | None:
    """녹화 한 개 — mp4 로 바꿔 두고 그 파일. 바꿨으면 mkv 는 지운다(같은 영상을 두 벌 두지 않게 — 로봇 SSD, 20건 보관).
    mkv 가 이미 지워졌으면 남은 mp4. 둘 다 없으면 None"""
    mp4 = mkv.with_suffix(".mp4")
    if mkv.exists() and mkv.stat().st_size > 0:
        f = remux(mkv)
        if f == mp4:
            try:
                mkv.unlink()
            except OSError:
                pass
        return f
    return mp4 if mp4.exists() and mp4.stat().st_size > 0 else None


def clips_json(stem: str, mdir: Path, qc: dict, kind: str) -> list[dict]:
    """녹화 — 그 폴더의 zoom · thermal(.mkv). 단계 시작이 녹화 기준 몇 초였나(CSV 녹화[s] 칸)."""
    marks = {}
    for st in qc["steps"]:
        try:
            r = float(st.get("rec") or "")
        except ValueError:
            continue
        marks[str(step_no(st["name"]))] = round(r, 1)
    out = []
    for key, ko, en in CAMS:
        f = clip_file(mdir / f"{key}.mkv")
        if not f:
            continue
        out.append({"cam": key, "label": bi(ko, en), "file": f"{stem}/{f.name}", "bytes": f.stat().st_size,
                    "part": kind, "steps": marks})
    clipdir = mdir / "soak_clip"
    for key, ko, en in CAMS:
        f = clip_file(clipdir / f"{key}.mkv")
        if f:
            out.append({"cam": key, "label": bi(ko + " — 공회전 끝", en + " — end of soak"), "file": f"{stem}/soak_clip/{f.name}",
                        "bytes": f.stat().st_size, "part": "soakEnd", "steps": {}})
    return out


def photos_json(stem: str, mdir: Path) -> list[dict]:
    out = []
    for f, ko, en in (("snap_zoom.jpg", "줌 카메라", "Zoom camera"), ("snap_thermal.jpg", "열화상 카메라", "Thermal camera")):
        if (mdir / f).exists():
            out.append({"file": f"{stem}/{f}", "caption": bi(ko, en)})
    return out


# ---------------------------------------------------------------- 단계 다시 돌린 기록(qc/03 §8)
def rerun_attempts(base_csv: Path, qc1: dict) -> list[dict]:
    """원래 보고서 폴더 reruns.json(엔진이 남김) → 시도마다 고른 단계 · 같이 돈 단계 · 판정 · 측정값 · 조건."""
    mdir = media_of(base_csv)
    runs = read_json(mdir / "reruns.json").get("runs") or []
    d1 = dict(qc1["details"])
    started1 = qc1["head"].get("시작") or ""
    out = []
    # 멈춘 시도(사람 · 배터리 · 다른 PTZ — 엔진 writeRerunLink 의 stopped)는 판정에도 기록에도 넣지 않는다. 목록에 남는 것은
    # 원래 보고서를 지울 때 그 파일까지 지우려고다(linked)
    runs = [r for r in runs if not r.get("stopped")]
    for i, r in enumerate(runs):
        csv2 = Path(str(r.get("csv") or ""))
        if not csv2.is_file():
            cand = base_csv.parent / f"{r.get('stem', '')}.csv"
            csv2 = cand if cand.is_file() else None
        if not csv2:
            out.append({"n": i + 1, "stem": r.get("stem") or "", "missing": True, "startedAt": r.get("startedAt") or ""})
            continue
        qc2 = read_qc_csv(csv2)
        d2 = dict(qc2["details"])
        picked = [int(x) for x in r.get("picked") or []]
        ran = [int(x) for x in r.get("ran") or []]
        views = {v.no: v for v in build(qc2, None) if v.no <= 18}
        steps = []
        for no in ran:
            v = views.get(no)
            if not v:
                continue
            raw = next((st for st in qc2["steps"] if step_no(st["name"]) == no), {})
            steps.append({"no": no, "name": bi(v.name, qc_en.title(no, v.name)), "verdict": v.verdict,
                          "line": bi(v.line, qc_en.line(no, v.line)), "measured": raw.get("measured", ""),
                          "prep": no not in picked})
        # R5 — 펌웨어를 구웠거나 영점이 바뀌었으면 그 전 결과가 낡았다
        fw = any(d2.get(f"최신화.{b} 구웠나") == "예" for b in ("GW", "YAW", "PITCH"))
        zero = any(zero_moved(d1.get(f"{ax}.영점 U/V/W"), d2.get(f"{ax}.영점 U/V/W")) for ax in ("YAW", "PITCH")) \
            and 6 in ran and d2.get("영점.방식") is None
        vers = " · ".join(f"{lab} {d2.get('버전.' + b)}" for b, lab in (("GW", "게이트웨이"), ("YAW", "YAW"), ("PITCH", "PITCH"))
                          if d2.get("버전." + b))
        since = ""
        try:
            a = datetime.strptime(started1, "%Y-%m-%d %H:%M:%S")
            b2 = datetime.strptime(qc2["head"].get("시작") or "", "%Y-%m-%d %H:%M:%S")
            mins = int((b2 - a).total_seconds() // 60)
            since = f"{mins // 60}시간 {mins % 60}분" if mins >= 60 else f"{mins}분"
        except ValueError:
            pass
        out.append({"n": i + 1, "stem": csv2.stem, "startedAt": qc2["head"].get("시작") or r.get("startedAt") or "",
                    "overall": qc2["head"].get("종합") or r.get("overall") or "", "picked": picked, "ran": ran,
                    "steps": steps, "level": r.get("level"),
                    "cond": {"versions": vers, "gait": r.get("gait") or "", "battery": r.get("battery"), "sinceFirst": since},
                    "outdated": {"fw": fw, "zero": zero}})
    return out


ZERO_MOVED_LSB = 10     # 다시 잰 영점이 상마다 이만큼 넘게 달라야 '영점이 바뀜' — 다시 재면 잡음으로 몇 LSB 는 늘 달라진다


def zero_moved(a: str | None, b: str | None) -> bool:
    """'2048/2051/2047' 두 개 — 상(U · V · W) 가운데 하나라도 ZERO_MOVED_LSB 넘게 달라졌나. 못 읽으면 글자가 다른지로"""
    if not a or not b:
        return False
    try:
        xa = [float(x) for x in str(a).split("/")]
        xb = [float(x) for x in str(b).split("/")]
    except ValueError:
        return a != b
    if len(xa) != len(xb):
        return True
    return any(abs(p - q) > ZERO_MOVED_LSB for p, q in zip(xa, xb))


def effective(first: str, attempts: list[str]) -> str:
    """R2 — 처음 결과는 지우지 않는다. 다시 돌린 마지막 판정과 '처음(FAIL 이면 WARN 으로 낮춤)' 가운데 나쁜 쪽."""
    tries = [a for a in attempts if a in RANK]
    if not tries:
        return first
    cap = "WARN" if first == "FAIL" else first
    last = tries[-1]
    if cap not in RANK:
        return last
    return last if RANK[last] >= RANK[cap] else cap


# ---------------------------------------------------------------- 판정 도장
def merge_rerun(v: str, eff_overall: str, non_qc: str) -> str:
    """도장 v 에 다시 돌린 QC 판정을 넣는다. 다시 돌리기는 QC 1~18 몫만 바꾼다 — 공회전 · 재검사(non_qc)에서 온 판정은 그대로.
    도장이 FAIL 이어도 그 까닭이 QC 단계뿐일 때만 WARN 으로 낮춰 보고(R2 — 처음 FAIL 은 다시 돌려도 WARN 까지),
    다시 돌린 QC 판정과 견줘 나쁜 쪽."""
    if v not in RANK or eff_overall not in RANK:
        return v
    capped = "WARN" if v == "FAIL" and RANK.get(non_qc, 0) < RANK["FAIL"] else v
    return eff_overall if RANK[eff_overall] >= RANK[capped] else capped


def stamp(qc: dict, mdir: Path, stage: str, csv_path: Path, eff_overall: str | None, non_qc: str = "") -> dict:
    """웹 제품 페이지의 판정 도장과 같은 셈(qc_web.product_html) + 다시 돌린 기록(R2).
    eff_overall — 다시 돌린 뒤 QC 1~18 의 판정. non_qc — 19 공회전 · 20 재검사 가운데 나쁜 판정(다시 돌리기로 바뀌지 않는 부분)."""
    run = read_json(mdir / "run.json")
    h = qc["head"]
    interim = stage == "interim"
    fields = qc_soak.row_fields(qc, mdir, stage, csv_path)
    if not interim and fields.get("void") or run.get("verdict") == "무효":
        v = "무효"
        why = run.get("reason") or fields.get("voidWhy") or fields.get("finalWhy") or ""
    elif not interim and fields.get("final") and (fields.get("soak") or fields.get("recheck")):
        v, why = fields["final"], fields.get("finalWhy", "")
    else:
        v = h.get("종합") or "—"
        why = "QC 판정" + (" — 공회전 전 중간 보고" if interim else "")
    why_en = qc_en.text(why)
    if eff_overall and v in RANK:
        # 다시 돌리기는 QC 1~18 몫만 바꾼다. 공회전 · 재검사에서 온 판정은 그대로 둔다 — 도장이 FAIL 이어도 그 까닭이 QC 단계뿐일
        # 때만 WARN 으로 낮춰 보고(R2 — 처음 FAIL 은 다시 돌려도 WARN 까지), 다시 돌린 QC 판정과 견줘 나쁜 쪽
        nv = merge_rerun(v, eff_overall, non_qc)
        if nv != v:
            why = f"{why} · 단계를 다시 돌림(처음 결과는 남김)"
            why_en = f"{why_en} · steps re-run (first results kept)" if why_en else None
            v = nv
    if v == "중지":
        v = "무효"
    c = CUSTOMER.get(v)
    return {"verdict": v, "why": bi(why, why_en), "customer": bi(c[0], c[1]) if c else None,
            "category": fields.get("category") or run.get("category") or ""}


# ---------------------------------------------------------------- 한 건
def export(csv_path: Path, stage: str | None = None) -> Path:
    csv_path = Path(csv_path).resolve()
    qc = read_qc_csv(csv_path)
    stem = csv_path.stem
    mdir = media_of(csv_path)
    mdir.mkdir(parents=True, exist_ok=True)
    run = read_json(mdir / "run.json")
    rb = run.get("robot") or {}
    rerun = read_json(mdir / "rerun.json")
    if rerun.get("base"):
        # 다시 돌린 결과 — 제 report.json 은 짧게, 원래 보고서를 다시 내서 기록을 붙인다(R1)
        base = csv_path.parent / f"{rerun['base']}.csv"
        out = mdir / "report.json"
        write_json(out, {"v": V, "kind": "rerunAttempt", "stem": stem, "base": rerun["base"], "generatedAt": now()})
        if base.is_file():
            bstage = "final" if read_json(media_of(base) / "run.json").get("endedAt") else None
            export(base, bstage)
        return out
    if stage is None:
        stage = "final" if run.get("endedAt") else ("interim" if (mdir / "soak.json").exists() else "final")
    views = build(qc, mdir, stage)
    from qc_steps import limits  # noqa: PLC0415
    L = limits(qc)
    d = dict(qc["details"])
    moving = set(int(x) for x in ((rb.get("movingSteps") or {}).get("qc") or []))

    # 다시 돌린 기록 — 단계마다 시도 판정
    attempts = rerun_attempts(csv_path, qc)
    tries: dict[int, list[str]] = {}
    for a in attempts:
        for st in a.get("steps") or []:
            if st["prep"] and st["verdict"] not in ("FAIL",):
                continue                     # 준비로 돈 단계는 FAIL 만 그 단계 판정에 더한다(17 이 EEPROM 드롭을 잡으면 FAIL)
            tries.setdefault(st["no"], []).append(st["verdict"])
    summary, effs = [], []
    v20 = qc_soak.recheck_view(qc, mdir, csv_path)
    if rb.get("kind") == "basic":                    # 로봇 기본 검사는 1~18 — 공회전 줄을 싣지 않는다(돌리지 않는 것이 정상)
        views = [v for v in views if v.no != 19]
    for v in views + ([v20] if v20 else []):
        row = {"no": v.no, "name": bi(v.name, qc_en.title(v.no, v.name)), "verdict": v.verdict,
               "line": bi(v.line, qc_en.line(v.no, v.line)), "moving": v.no in moving}
        if v.no in tries:
            row["attempts"] = len(tries[v.no])
            row["lastAttempt"] = tries[v.no][-1]
            row["effective"] = effective(v.verdict, tries[v.no])
        summary.append(row)
        if v.no <= 18:
            effs.append(row.get("effective", v.verdict))
    eff_overall = None
    if tries:
        worst = max((RANK[x] for x in effs if x in RANK), default=0)
        eff_overall = {1: "PASS", 2: "WARN", 3: "FAIL"}.get(worst)
    non_qc = max((r["verdict"] for r in summary if r["no"] > 18 and r["verdict"] in RANK), key=lambda x: RANK[x], default="")
    head_st = stamp(qc, mdir, stage, csv_path, eff_overall, non_qc)
    outdated = {"fw": any(a.get("outdated", {}).get("fw") for a in attempts),
                "zero": any(a.get("outdated", {}).get("zero") for a in attempts)}
    notice = None
    if outdated["fw"] or outdated["zero"]:
        what = "펌웨어" if outdated["fw"] and not outdated["zero"] else "영점" if outdated["zero"] and not outdated["fw"] else "펌웨어 · 영점"
        what_en = {"펌웨어": "firmware", "영점": "current zero", "펌웨어 · 영점": "firmware and current zero"}[what]
        notice = bi(f"{what}이(가) 바뀐 뒤의 결과가 섞여 있다 — 처음부터 다시 검사 권장",
                    f"Results after the {what_en} changed are mixed in — a full check from the start is recommended")

    metrics = [{"label": bi(a, qc_en.text(a)), "value": b, "unit": c, "limit": bi(lim, qc_en.cell(lim)), "near": e,
                "nearEn": qc_en.cell(e) if e else None}
               for a, b, c, lim, e in table_rows(key_metrics(qc))]
    v19 = next((v for v in views if v.no == 19), None)
    steps = [step_json(v, qc, L, moving) for v in views if v.no <= 18]
    rep = {
        "v": V, "stem": stem, "stage": stage, "generatedAt": now(),
        "kind": rb.get("kind") or ("aging" if (mdir / "soak.json").exists() else "basic"),
        "level": int(rb.get("level") or 2),
        "head": {"serial": h_serial(qc, rb), "uid": d.get("UID.GW", "") if d.get("UID.GW") != "무응답" else "",
                 "startedAt": qc["head"].get("시작") or "", "endedAt": run.get("endedAt") or "",
                 "seconds": qc["head"].get("소요[s]") or "",
                 "versions": {"gw": d.get("버전.GW", ""), "yaw": d.get("버전.YAW", ""), "pitch": d.get("버전.PITCH", "")},
                 "presets": rb.get("presets"), "robotMoving": bool(moving) or bool(rb.get("movingSoakS")),
                 "batteryMin": rb.get("batteryMin"), "notice": notice, **head_st},
        "clips": clips_json(stem, mdir, qc, "qc"),
        "summary": summary,
        "metrics": metrics, "metricsNote": bi(METRIC_NOTE, qc_en.text(METRIC_NOTE)),
        "steps": steps,
        "soak": soak_json(v19, mdir, qc, L),
        "recheck": recheck_json(v20, qc, L, csv_path, mdir),
        "reruns": attempts,
        "photos": photos_json(stem, mdir),
        "appendix": [[k, v] for k, v in qc["details"]],
    }
    out = mdir / "report.json"
    write_json(out, rep)
    write_index(csv_path.parent)
    return out


def h_serial(qc: dict, rb: dict) -> str:
    sn = rb.get("serialProduct") or qc["head"].get("시리얼") or ""
    return sn if sn and sn != "미상" else ""


def recheck_json(v20: StepView | None, qc: dict, L: dict, csv_path: Path, mdir: Path) -> dict | None:
    if not v20:
        return None
    out = step_json(v20, qc, L, set())
    c = getattr(v20, "_compare", None)
    if c and c.csv2:
        out["stem2"] = c.csv2.stem
        mdir2 = media_of(c.csv2)
        out["clips"] = clips_json(c.csv2.stem, mdir2, read_qc_csv(c.csv2), "recheck")
        rb = read_json(mdir / "run.json").get("robot") or {}
        moving2 = set(int(x) for x in ((rb.get("movingSteps") or {}).get("recheck") or []))
        out["steps2"] = [step_json(v, read_qc_csv(c.csv2), L, moving2) for v in c.views2]
    return out


# ---------------------------------------------------------------- 이력 목록 · 보관
def is_main(csv_path: Path) -> bool:
    """이력 한 줄이 되는 보고서 — 재검사(before.json) · 다시 돌린 시도(rerun.json)는 원래 보고서에 붙는다."""
    m = media_of(csv_path)
    return not (m / "before.json").exists() and not (m / "rerun.json").exists()


def started_of(csv_path: Path) -> str:
    m = re.search(r"_(\d{8})_(\d{6})$", csv_path.stem)
    return f"{m.group(1)}{m.group(2)}" if m else datetime.fromtimestamp(csv_path.stat().st_mtime).strftime("%Y%m%d%H%M%S")


def mains(qc_dir: Path) -> list[Path]:
    out = [p for p in Path(qc_dir).glob("*.csv") if p.name != "summary.csv" and is_main(p)]
    return sorted(out, key=started_of, reverse=True)


def write_index(qc_dir: Path) -> Path:
    """앱 [결과] 칸의 목록 — 최신부터. 로봇 검사(run.json robot)만 싣는다(QC PC 폴더를 가리켜도 섞이지 않게)."""
    rows = []
    for p in mains(qc_dir):
        m = media_of(p)
        run = read_json(m / "run.json")
        rb = run.get("robot")
        if not isinstance(rb, dict):
            continue
        rep = read_json(m / "report.json")
        head = rep.get("head") or {}
        reruns = len([r for r in read_json(m / "reruns.json").get("runs") or [] if not r.get("stopped")])
        # 걸린 단계 — 요약표에서 PASS · SKIP 이 아닌 단계(다시 돌린 단계는 마지막 판정까지)
        flagged = [{"no": r.get("no"), "name": r.get("name"), "verdict": r.get("verdict"),
                    "effective": r.get("effective"), "attempts": r.get("attempts")}
                   for r in rep.get("summary") or []
                   if r.get("verdict") not in ("PASS", "SKIP", "", None) or r.get("attempts")]
        rows.append({"stem": p.stem, "startedAt": run.get("startedAt") or "", "endedAt": run.get("endedAt") or "",
                     "serial": rb.get("serialProduct") or "", "uid": rb.get("uid") or "", "kind": rb.get("kind") or "",
                     "level": rb.get("level"), "soakMin": rb.get("soakMin") or 0,
                     "verdict": head.get("verdict") or run.get("verdict") or "",
                     "why": (head.get("why") or {}).get("ko") or run.get("reason") or "", "whyBi": head.get("why"),
                     "versions": head.get("versions") or {}, "flagged": flagged,
                     "stage": rep.get("stage") or "", "reruns": reruns, "running": not run.get("endedAt")})
    out = Path(qc_dir) / "index.json"
    write_json(out, {"v": V, "generatedAt": now(), "keep": KEEP_DEFAULT, "reports": rows})
    return out


def linked(csv_path: Path) -> list[Path]:
    """보고서 하나에 딸린 것 — 재검사 CSV(after.json) · 다시 돌린 시도(reruns.json)."""
    m = media_of(csv_path)
    out = []
    a = read_json(m / "after.json")
    if a.get("stem"):
        out.append(csv_path.parent / f"{a['stem']}.csv")
    for r in read_json(m / "reruns.json").get("runs") or []:
        if r.get("stem"):
            out.append(csv_path.parent / f"{r['stem']}.csv")
    return out


def remove_report(csv_path: Path) -> None:
    for p in [csv_path] + linked(csv_path):
        shutil.rmtree(media_of(p), ignore_errors=True)
        try:
            p.unlink()
        except OSError:
            pass


def prune(qc_dir: Path, keep: int) -> list[str]:
    """보고서가 새로 생기면 가장 오래된 것부터 지워 keep 건을 남긴다(qc/03 §6). 로봇 보고서만 센다 · 지운다."""
    gone = []
    robot = [p for p in mains(qc_dir) if isinstance(read_json(media_of(p) / "run.json").get("robot"), dict)]
    for p in robot[keep:]:
        if not read_json(media_of(p) / "run.json").get("endedAt"):
            continue                          # 도는 중이거나 끝 보고가 없다 — 마무리(finalize) 전에는 지우지 않는다
        remove_report(p)
        gone.append(p.stem)
    write_index(qc_dir)
    return gone


# ---------------------------------------------------------------- 끊긴 검사 마무리
def finalize(status_path: Path, why: str) -> int:
    """엔진이 끝 보고 없이 사라졌다(전원 · 프로그램 종료) — 남은 status.json 으로 CSV · run.json · report.json 을 마무리한다.
    이어서 다시 돌리지는 않는다(qc/03 §3). 이미 끝난 검사면 아무것도 안 한다."""
    st = read_json(status_path)
    if not st:
        print(f"status.json 을 못 읽음: {status_path}")
        return 1
    if st.get("state") == "done":
        print("이미 끝난 검사 — 할 일 없음")
        return 0
    qc_dir = Path(st.get("qcDir") or os.environ.get("RBQ_QC_DIR") or Path.home() / "PTZ_QC")
    qc_dir.mkdir(parents=True, exist_ok=True)
    stem = st.get("stem") or ""
    serial = st.get("serial") or ""
    uid = st.get("uid") or ""
    if not stem:
        base = serial or (("UID-" + uid.replace("-", "")) if uid else "PTZ")
        try:
            t = datetime.strptime(st.get("startedAt") or "", "%Y-%m-%d %H:%M:%S")
        except ValueError:
            t = datetime.now()
        stem = f"{re.sub(r'[^A-Za-z0-9_.-]', '_', base)}_{t.strftime('%Y%m%d_%H%M%S')}"
    csv_path = qc_dir / f"{stem}.csv"
    mdir = media_of(csv_path)
    mdir.mkdir(parents=True, exist_ok=True)
    if not csv_path.exists():
        # QC 탭이 결과 파일을 쓰기 전에 끊겼다 — status.json 의 단계표로 같은 모양의 CSV 를 만든다
        with csv_path.open("w", encoding="utf-8-sig", newline="") as f:
            w = csv.writer(f, quoting=csv.QUOTE_ALL)
            f.write("# PTZ QC\n")
            for k, v in (("시리얼", serial or "미상"), ("시작", st.get("startedAt") or ""), ("대상", "로봇 — 끊긴 검사를 마무리"),
                         ("종합", "중지"), ("소요[s]", "0")):
                f.write(f"{k},")
                w.writerow([v])
            f.write("\n검사,판정,측정값,기준,시간[s],녹화[s]\n")
            for s in st.get("steps") or []:
                vd = s.get("verdict") or "WAIT"
                vd = vd if vd in ("PASS", "WARN", "FAIL", "SKIP") else ("FAIL" if vd == "RUN" else "SKIP")
                meas = s.get("measured") or ""
                if vd == "FAIL" and (s.get("verdict") == "RUN"):
                    meas = (meas + " · " if meas else "") + "이 단계 도중에 검사가 끊김"
                elif vd == "SKIP" and not meas:
                    meas = "중지됨"
                w.writerow([s.get("name") or "", vd, meas, "", "0.0", ""])
            f.write("\n항목,값\n")
            w.writerow(["검사 조건", "로봇 점검 — 끝 보고 없이 끊긴 검사를 다음 기동 때 마무리"])
            if uid:
                w.writerow(["UID.GW", uid])
            w.writerow(["시리얼.제품", serial or "모름"])
    run = read_json(mdir / "run.json")
    if not run.get("endedAt"):
        run.update({"v": 1, "result": "stopped", "verdict": "무효", "category": "env", "reason": why, "endedAt": now(),
                    "csv": str(csv_path), "stem": stem})
        run.setdefault("startedAt", st.get("startedAt") or "")
        rb = run.get("robot") if isinstance(run.get("robot"), dict) else {}
        rb.setdefault("kind", st.get("kind") or "")
        rb.setdefault("level", st.get("level") or 2)
        rb.setdefault("presets", st.get("presets"))
        rb.setdefault("serialProduct", serial)
        rb.setdefault("uid", uid)
        rb.setdefault("stemBase", stem.rsplit("_", 2)[0])
        run["robot"] = rb
        write_json(mdir / "run.json", run)
    # 끊긴 단계 다시 돌리기 — 따로 이력 한 줄이 되지 않게 원래 보고서에 '멈춘 시도'로 붙인다(엔진 writeRerunLink 와 같은 틀).
    # 보고서 도구는 멈춘 시도를 판정에 넣지 않고, 원래 보고서를 지울 때 이 파일까지 지운다
    base_stem = str(st.get("base") or "")
    if st.get("kind") == "rerun" and base_stem and not (mdir / "rerun.json").exists():
        me = {"base": base_stem, "csv": str(csv_path), "stem": stem, "stopped": True, "why": why,
              "picked": st.get("picked") or [], "ran": st.get("ran") or [], "level": st.get("level"),
              "startedAt": st.get("startedAt") or "", "overall": "중지", "uid": uid}
        write_json(mdir / "rerun.json", me)
        bdir = media_of(csv_path.parent / f"{base_stem}.csv")
        if bdir.is_dir():
            lst = read_json(bdir / "reruns.json")
            runs = [r for r in lst.get("runs") or [] if r.get("stem") != stem]
            runs.append(me)
            lst["runs"] = runs
            write_json(bdir / "reruns.json", lst)
    st["state"] = "done"
    st["done"] = {"result": "stopped", "verdict": "무효", "category": "env", "reason": why, "stem": stem, "finalizedAt": now()}
    write_json(status_path, st)
    export(csv_path, "final")
    print(f"마무리 — {stem} · 무효 · {why}")
    return 0


# ---------------------------------------------------------------- 실행
def main() -> int:
    argv = sys.argv[1:]
    if argv[:1] == ["finalize"]:
        ap = argparse.ArgumentParser(prog="qc_export.py finalize")
        ap.add_argument("--status", required=True)
        ap.add_argument("--why", default="중지 — 검사 프로그램이 끝 보고 없이 끝남")
        a = ap.parse_args(argv[1:])
        return finalize(Path(a.status), a.why)
    if argv[:1] == ["prune"]:
        ap = argparse.ArgumentParser(prog="qc_export.py prune")
        ap.add_argument("--dir", required=True)
        ap.add_argument("--keep", type=int, default=KEEP_DEFAULT)
        a = ap.parse_args(argv[1:])
        gone = prune(Path(a.dir), max(1, a.keep))
        print("지움: " + (", ".join(gone) if gone else "없음"))
        return 0
    if argv[:1] == ["index"]:
        ap = argparse.ArgumentParser(prog="qc_export.py index")
        ap.add_argument("--dir", required=True)
        a = ap.parse_args(argv[1:])
        print(write_index(Path(a.dir)))
        return 0
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("csv")
    ap.add_argument("--stage", choices=("interim", "final"))
    a = ap.parse_args(argv)
    p = Path(a.csv)
    if not p.is_file():
        print(f"CSV 가 없다: {p}")
        return 2
    print(export(p, a.stage))
    return 0


if __name__ == "__main__":
    sys.exit(main())
