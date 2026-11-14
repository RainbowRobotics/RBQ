#!/usr/bin/env python3
"""QC 탭 CSV(+ Claude 참고 소견 + 사진) → 사람이 읽는 검사 보고서(HTML · PDF).

    python3 qc_report.py <QC 보고서 CSV> [--no-pdf]

입력 — CSV 옆의 같은 이름 폴더(<시리얼>_<시각>/)가 있으면 거기서 찾는다.
  findings.json          Claude 참고 소견(findings.schema.json). 없으면 "AI 소견 없음"
  *.jpg · *.png          스냅샷(폴더 바로 아래)
  frames/*_sheet_*.jpg   녹화 밀착 인화 — 녹화(zoom.mkv 등)가 있는데 없거나 낡았으면 여기서 만든다(qc_frames.ensure)
  soak.json              12시간 공회전 결과(있으면 19번 줄에)
출력 — <폴더>/report.html, 그리고 CSV 옆 <시리얼>_<시각>.pdf (Chrome 헤드리스 인쇄).
검사 결과 · 단계별 절은 qc_steps 가 만든다 — 웹 제품 페이지(qc_web.py)와 같은 글·표다.

판정은 CSV 에 있는 것을 그대로 옮긴다. 이 스크립트도 Claude 도 판정을 바꾸지 않는다 —
소견은 "참고"로만 붙는다. 틀(칸·순서)은 여기 코드가 고정한다.
"""
from __future__ import annotations

import argparse
import csv
import html
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import qc_frames  # noqa: E402  녹화 밀착 인화(사진 절)
try:
    import qc_soak  # noqa: E402  출하 판정(QC · 공회전 · 재검사) · 공회전 끝 영상 · 재검사 사진
except Exception as _e:        # 모듈이 없거나 깨져도 보고서는 나와야 한다(19 · 20 절 자리는 qc_steps 가 따로 막는다)
    print(f"qc_soak 을 못 불러 공회전 · 재검사 판정 없이 만든다 — {type(_e).__name__}: {_e}", file=sys.stderr)
    qc_soak = None
from qc_steps import STEPS_CSS, sections_html, soak_section, summary_html  # noqa: E402  단계 1~19 글·표(웹과 같은 모듈)
from qc_steps import build as build_steps  # noqa: E402
import qc_fix  # noqa: E402  판정 고침 — 웹과 같은 셈

VERDICT_COLOR = {"PASS": "#1e7d3c", "WARN": "#b36b00", "FAIL": "#c0392b", "SKIP": "#7f8c8d", "중지": "#7f8c8d",
                 "WAIT": "#8a969c", "none": "#8a969c"}
DEFAULT_PRODUCT = "PTZ"     # 기본 품명 — 웹 대시보드에서 바꾸면 <QC 폴더>/web/site.json 에 남는다


# ---------------------------------------------------------------- CSV 읽기
def read_qc_csv(path: Path) -> dict:
    """QcTab::finishRun 이 쓰는 모양: 머리(키,값) · 빈 줄 · 단계표 · 빈 줄 · 세부(항목,값)."""
    head, steps, details = {}, [], []
    part = "head"
    with path.open(encoding="utf-8-sig", newline="") as f:
        for row in csv.reader(f):
            if not row or not any(c.strip() for c in row):
                continue
            if row[0].startswith("#"):
                continue
            if row[0] == "검사" and len(row) >= 2 and row[1] == "판정":
                part = "steps"
                continue
            if row[0] == "항목" and len(row) >= 2 and row[1] == "값":
                part = "details"
                continue
            if part == "head" and len(row) >= 2:
                head[row[0]] = row[1]
            elif part == "steps" and len(row) >= 5:
                # 6번째 칸 '녹화[s]' — 단계가 시작한 시각(녹화 기준). 녹화 없이 돈 검사·옛 CSV 는 없다.
                steps.append({"name": row[0], "verdict": row[1], "measured": row[2],
                              "criterion": row[3], "seconds": row[4], "rec": row[5] if len(row) > 5 else ""})
            elif part == "details" and len(row) >= 2:
                details.append((row[0], row[1]))
    return {"head": head, "steps": steps, "details": details}


def record_meta(csv_path: Path) -> dict:
    """사람이 웹에서 고친 값 — <QC 폴더>/web/r/<stem>/meta.json 의 품명 · 시리얼.
    품명이 없으면 기본 품명(web/site.json, 없으면 PTZ). QC 원본 CSV 는 고치지 않는다."""
    web = csv_path.parent / "web"
    try:
        meta = json.loads((web / "r" / csv_path.stem / "meta.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        meta = {}
    if not meta.get("product"):
        try:
            meta["product"] = json.loads((web / "site.json").read_text(encoding="utf-8")).get("product") or DEFAULT_PRODUCT
        except (OSError, ValueError):
            meta["product"] = DEFAULT_PRODUCT
    return meta


# ---------------------------------------------------------------- HTML
def esc(s) -> str:
    return html.escape("" if s is None else str(s))


def badge(v: str) -> str:
    c = VERDICT_COLOR.get(v, "#555")
    return f'<span class="badge" style="background:{c}">{esc(v or "—")}</span>'


def findings_html(fd: dict | None) -> str:
    if not fd:
        return '<p class="muted">AI 소견 없음 — 아직 분석하지 않았거나 실패했다.</p>'
    out = [f'<p class="note">판정에 반영하지 않는 참고 소견이다. 작성: {esc(fd.get("author"))}</p>',
           f'<p>{esc(fd.get("summary"))}</p>']
    t = fd.get("thermal") or {}
    if t.get("readable"):
        out.append(f'<h3>열화상 온도 — 화면 온도 표시(OSD)를 읽은 값</h3><p>최고 {esc(t.get("max_c"))} °C · 최저 {esc(t.get("min_c"))} °C · '
                   f'평균 {esc(t.get("avg_c"))} °C <span class="muted">— {esc(t.get("evidence"))}</span></p>')
    else:
        out.append(f'<h3>열화상 온도 — 화면 온도 표시(OSD)를 읽은 값</h3><p class="muted">온도 표시를 읽지 못함 — {esc(t.get("evidence"))}</p>')
    iq = fd.get("image_quality") or []
    if iq:
        rows = "".join(f"<tr><td class='call'>{esc(x['camera'])}</td><td class='call'>{esc(x['call'])}</td><td>{esc(x['observation'])}</td>"
                       f"<td class='muted'>{esc(x['evidence'])}</td></tr>" for x in iq)
        out.append(f'<h3>화질</h3><table><tr><th>카메라</th><th>판단</th><th>관찰</th><th>근거</th></tr>{rows}</table>')
    om = fd.get("output_motion") or []
    if om:
        rows = "".join(f"<tr><td>{esc(x['segment'])}</td><td>{esc(x['commanded'])}</td><td>{esc(x['seen'])}</td>"
                       f"<td class='call'>{esc(x['call'])}</td><td class='muted'>{esc(x['evidence'])}</td></tr>" for x in om)
        out.append('<h3>출력축 움직임 (영상)</h3><p class="muted">엔코더가 감속기 앞 모터축에 있어서, 감속기 뒤 출력축이 실제로 '
                   '얼마나 돌았는지는 영상으로만 본다.</p>'
                   f'<table><tr><th>구간</th><th>명령</th><th>영상에서 본 움직임</th><th>판단</th><th>근거</th></tr>{rows}</table>')
    fu = fd.get("follow_up") or []
    if fu:
        out.append("<h3>사람이 확인할 것</h3><ul>" + "".join(f"<li>{esc(x)}</li>" for x in fu) + "</ul>")
    return "\n".join(out)


def build_html(qc: dict, fd: dict | None, photos: list[dict], base: Path, meta: dict | None = None) -> str:
    """photos = qc_frames.photo_list(검사 폴더) — {path(검사 폴더 기준), caption, sheet}."""
    h = qc["head"]
    meta = meta or {}
    d = dict(qc["details"])
    serial_qc = h.get("시리얼", "")
    serial = meta.get("serial") or serial_qc
    product = meta.get("product") or DEFAULT_PRODUCT
    uid = d.get("UID.GW", "")
    edited = f' <span class="muted">(웹에서 고침 — QC 가 남긴 시리얼은 {esc(serial_qc)})</span>' if serial != serial_qc else ""
    overall = h.get("종합", "")
    fw = " · ".join(f"{label} {d.get('버전.' + b)}" for b, label in (("GW", "게이트웨이"), ("YAW", "YAW"), ("PITCH", "PITCH"))
                    if d.get("버전." + b))
    # 공회전 · 재검사가 붙었으면 머리 판정은 출하 판정(QC · 공회전 · 재검사). PDF 는 단계를 몰라 soak.json · after.json 으로 본다
    fin = {}
    if qc_soak:
        try:
            fin = qc_soak.row_fields(qc, base, "final")
        except Exception as e:                     # 기록이 깨져도 PDF 는 나와야 한다
            print(f"공회전 · 재검사 판정을 못 읽음 — {type(e).__name__}: {e}", file=sys.stderr)
    ship = fin.get("final") if fin.get("soak") or fin.get("recheck") else ""
    # 다시 돌린 기록(R2) · 판정 고침 — 웹 도장과 같은 셈(qc_web.rerun_view · qc_fix.recount). 10-14 전에는 PDF 만 이것을 몰라
    # 다시하기로 WARN 이 된 검사가 PDF 에서는 FAIL 로 남았다
    import qc_web  # noqa: PLC0415  qc_web 이 이 파일을 읽으므로 쓸 때 읽는다
    rv = qc_web.rerun_view(base, qc, fin.get("soak", ""), fin.get("recheck", ""))
    head_v, head_why = ship or overall, fin.get("finalWhy", "") if ship else ""
    if rv and rv["eff"] and head_v in ("PASS", "WARN", "FAIL"):
        nv = rv["merge"](head_v, rv["eff"], rv["nonQc"])
        if nv != head_v:
            head_v, head_why = nv, " · ".join(x for x in (head_why, "단계를 다시 돌림(처음 결과는 남김)") if x)
    rc = qc_fix.recount(qc, fin, rv, meta, "final")
    if rc:
        head_v, head_why = rc["overall"], " · ".join(rc["why"])
    # 10-26 전 모터 판이 전류를 2배로 읽어 숫자를 고쳐 적은 검사 — CSV 머리 '전류 고침'(ui 58 · tools/ptz_qc_cur2x_fix.py)
    cur_fix = f'<tr><td>전류 숫자</td><td>{esc(h["전류 고침"])}</td></tr>' if h.get("전류 고침") else ""
    views = build_steps(qc, base)              # 19 는 보고서 단계를 모른다(PDF 는 웹 내보내기보다 먼저 만든다) — soak.json 이 없으면 '기록 없음'
    from qc_metrics import NOTE as METRIC_NOTE

    def pill(v) -> str:
        c = VERDICT_COLOR.get(v.vclass or "none", "#555")
        return f'<span class="badge" style="background:{c}">{esc(v.verdict or "—")}</span>'
    details = "".join(f"<tr><td>{esc(k)}</td><td>{esc(v)}</td></tr>" for k, v in qc["details"])
    # 스냅샷은 반 폭 둘씩, 밀착 인화는 칸마다 시각·단계 글자가 있어 한 줄을 다 쓴다
    def figs_of(ps: list[dict]) -> str:
        return "".join(
            f'<figure class="{"sheet" if p.get("sheet") else ""}"><img src="{esc(p["path"])}">'
            f'<figcaption>{esc(p["caption"])}</figcaption></figure>'
            for p in ps)
    later = [p for p in photos if p.get("group") == "재검사"]    # 공회전 뒤 재검사 녹화
    figs = figs_of([p for p in photos if not p.get("group")]) + (
        "<h3>재검사(공회전 뒤) 녹화</h3>" + figs_of(later) if later else "")
    return f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<title>{esc(product)} QC {esc(serial)}</title>
<style>
  @page {{ size: A4; margin: 14mm 12mm; }}
  body {{ font-family: 'Noto Sans CJK KR', 'Noto Sans KR', sans-serif; font-size: 10pt; color: #222;
          word-break: keep-all; overflow-wrap: break-word; }}   /* 한글은 낱말 사이에서만 줄을 바꾼다 */
  h1 {{ font-size: 16pt; margin: 0 0 4px; }}
  h2 {{ font-size: 12pt; border-bottom: 2px solid #333; padding-bottom: 2px; margin: 16px 0 6px; break-after: avoid; }}
  h3 {{ font-size: 10.5pt; margin: 10px 0 4px; }}
  table {{ border-collapse: collapse; width: 100%; margin: 4px 0; }}
  th, td {{ border: 1px solid #bbb; padding: 3px 5px; vertical-align: top; text-align: left; }}
  th {{ background: #f0f0f0; }}
  td.num, th.num {{ text-align: right; white-space: nowrap; }}
  td.call {{ white-space: nowrap; }}   /* 소견 표의 카메라·판단 칸 — "정상"이 한 글자씩 꺾이지 않게 */
  .muted {{ color: #666; font-size: 9pt; }}
  .note {{ background: #fff6e0; border-left: 4px solid #d68910; padding: 4px 8px; }}
  .badge {{ color: #fff; border-radius: 3px; padding: 1px 6px; font-weight: bold; white-space: nowrap; font-size: 9pt; }}
  .overall {{ font-size: 20pt; padding: 2px 14px; }}
  .head td {{ border: none; padding: 1px 8px 1px 0; }}
  figure {{ display: inline-block; width: 48%; margin: 4px 1% 8px 0; page-break-inside: avoid; vertical-align: top; }}
  figure img {{ width: 100%; border: 1px solid #999; }}
  figure.sheet {{ width: 100%; margin-right: 0; }}
  figcaption {{ font-size: 8pt; color: #666; }}
  tr {{ page-break-inside: avoid; }}
  {STEPS_CSS}
  /* 인쇄 — 웹 모양을 A4 에 맞게 줄인다 */
  table.sum td, table.sum th {{ padding: 2px 5px; }}
  table.sum td.ln {{ font-size: 9pt; }}
  section.st {{ margin-top: 10px; padding-top: 8px; border-top: 1px solid #999; }}
  section.st h3 {{ font-size: 11pt; margin: 0 0 3px; break-after: avoid; }}
  section.st dl.wh {{ gap: 2px 10px; margin-bottom: 6px; max-width: none; break-inside: avoid; }}
  section.st dl.wh dt, section.st .crit .lbl {{ color: #0f6b74; font-size: 9pt; }}
  section.st .crit {{ break-inside: avoid; margin-bottom: 6px; }}
  section.st .crit ul {{ max-width: none; gap: 2px; }}
  section.st .crit li {{ grid-template-columns: 5.5em 1fr; }}
  section.st table.mt {{ font-size: 9pt; }}
  section.st p.tnote {{ font-size: 8pt; max-width: none; }}
  section.st p.line {{ margin-bottom: 6px; }}
  table.apx {{ font-size: 7.5pt; }}
  table.apx td, table.apx th {{ padding: 1px 4px; }}
</style></head><body>
<h1>{esc(product)} 출하 검사 보고서</h1>
<table class="head">
  <tr><td>품명</td><td>{esc(product)}</td><td rowspan="6" style="text-align:right">{badge(head_v).replace('class="badge"', 'class="badge overall"')}</td></tr>
  <tr><td>시리얼</td><td><b>{esc(serial)}</b>{edited}</td></tr>
  <tr><td>게이트웨이 칩 고유 번호 (UID)</td><td>{esc(uid or '— (이 검사에서는 안 읽음)')}</td></tr>
  <tr><td>펌웨어</td><td>{esc(fw or '—')}</td></tr>
  <tr><td>검사 일시</td><td>{esc(h.get('시작'))} · 걸린 시간 {esc(h.get('소요[s]'))} 초</td></tr>
  {cur_fix}
  <tr><td>결과 파일</td><td class="muted">{esc(qc.get('source'))}</td></tr>
  {f'<tr><td>출하 판정</td><td>{esc(head_v)} — {esc(head_why)}</td></tr>' if head_why else ''}
</table>

{qc_web.precheck_html(base)}
<h2>검사 결과</h2>
{summary_html(views + (qc_soak.summary_rows(qc, base) if qc_soak else []), pill, link=False)}
{qc_web.rerun_html(rv, badge)}
{qc_web.holds_html(base)}
{qc_web.fix_html(qc, fin, meta, False, base.name, badge)}

<h2>주요 수치 — 한계에 얼마나 가까웠나</h2>
<p class="muted">판정에는 쓰지 않는다. 단계 기준을 한계로 두고 값이 얼마나 다가갔는지 적어 제품끼리 견준다. 한계는 지금 기준이라,
옛 검사는 단계별 자세히의 기준과 다를 수 있다. 한계의 85% 이상이면 굵게 적는다.</p>
<table><tr><th>항목</th><th>값</th><th>한계</th><th>한계에 다가간 정도 (100% = 한계)</th></tr>{metrics_rows(qc)}</table>
<p class="muted">{esc(METRIC_NOTE)}</p>

{f'<h2>AI 참고 소견</h2>{findings_html(fd)}' if fd else ''}

<h2>단계별 자세히</h2>
<p class="muted">단계마다 무엇을 어떻게 왜 재는지와 판정 기준, 측정값.</p>
{sections_html(views, pill, web=False)}
{soak_section(base, False)}

<h2>사진</h2>
{f'<p class="muted">{esc(qc_frames.sheet_note(photos))}</p>' if qc_frames.sheet_note(photos) else ''}
{figs or '<p class="muted">사진 없음</p>'}

<h2>부록 — QC 탭이 남긴 세부 값 (원본 그대로)</h2>
<table class="apx"><tr><th>항목</th><th>값</th></tr>{details}</table>
</body></html>
"""


def metrics_rows(qc: dict) -> str:
    # qc_metrics 가 이 파일을 불러 쓰므로 여기서 부른다
    from qc_metrics import flagged, key_metrics, table_rows
    return "".join(
        f"<tr><td>{esc(a)}</td><td class='num'>{esc(b)}{'' if c in ('%', '°') else ' '}{esc(c)}</td><td class='muted'>{esc(d)}</td>"
        f"<td class='num'>{'<b>' + esc(e) + '</b>' if flagged(e) else esc(e)}</td></tr>"
        for a, b, c, d, e in table_rows(key_metrics(qc)))


# ---------------------------------------------------------------- 소견 검사
def validate_findings(fd: dict) -> list[str]:
    """findings.schema.json 의 필수 칸·선택지만 본다(jsonschema 없이). 틀리면 이유 목록."""
    schema = json.loads((Path(__file__).with_name("findings.schema.json")).read_text(encoding="utf-8"))
    errs = []
    for key in schema["required"]:
        if key not in fd:
            errs.append(f"'{key}' 없음")
    for key in fd:
        if key not in schema["properties"]:
            errs.append(f"모르는 칸 '{key}'")
    t = fd.get("thermal", {})
    for key in schema["properties"]["thermal"]["required"]:
        if key not in t:
            errs.append(f"thermal.'{key}' 없음")
    for arr in ("image_quality", "output_motion"):
        item = schema["properties"][arr]["items"]
        for i, x in enumerate(fd.get(arr, [])):
            for key in item["required"]:
                if key not in x:
                    errs.append(f"{arr}[{i}].'{key}' 없음")
            for key, spec in item["properties"].items():
                if "enum" in spec and key in x and x[key] not in spec["enum"]:
                    errs.append(f"{arr}[{i}].{key} = '{x[key]}' — {spec['enum']} 중 하나여야 함")
    return errs


# ---------------------------------------------------------------- 실행
def find_chrome() -> str | None:
    for name in ("google-chrome", "chromium", "chromium-browser"):
        p = shutil.which(name)
        if p:
            return p
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="QC 탭이 남긴 <시리얼>_<시각>.csv")
    ap.add_argument("--no-pdf", action="store_true", help="HTML 만 만든다")
    a = ap.parse_args()

    csv_path = a.csv.resolve()
    qc = read_qc_csv(csv_path)
    qc["source"] = csv_path.name
    mdir = csv_path.with_suffix("")
    mdir.mkdir(exist_ok=True)

    fd = None
    fpath = mdir / "findings.json"
    if fpath.exists():
        fd = json.loads(fpath.read_text(encoding="utf-8"))
        errs = validate_findings(fd)
        if errs:
            print("findings.json 이 스키마와 다릅니다 — 보고서를 만들지 않습니다:", file=sys.stderr)
            for e in errs:
                print("  - " + e, file=sys.stderr)
            return 1

    qc_frames.ensure(csv_path)                     # 녹화 밀착 인화 — 없거나 낡았으면 만든다(한 건에 10 초 안팎)
    photos = qc_frames.photo_list(mdir)
    if qc_soak:                                    # 공회전 뒤 재검사 녹화 — 처음 검사 폴더 기준 경로로 붙인다
        try:
            photos += [dict(p, path=os.path.relpath(p["src"], mdir)) for p in qc_soak.recheck_photos(csv_path)]
        except Exception as e:
            print(f"재검사 사진을 못 붙임 — {type(e).__name__}: {e}", file=sys.stderr)

    out_html = mdir / "report.html"
    out_html.write_text(build_html(qc, fd, photos, mdir, record_meta(csv_path)), encoding="utf-8")
    print(f"HTML  {out_html}")

    if a.no_pdf:
        return 0
    chrome = find_chrome()
    if not chrome:
        print("Chrome/Chromium 이 없어 PDF 는 건너뜀 — HTML 을 브라우저에서 인쇄하세요", file=sys.stderr)
        return 0
    out_pdf = csv_path.with_suffix(".pdf")
    subprocess.run([chrome, "--headless", "--disable-gpu", "--no-pdf-header-footer",
                    f"--print-to-pdf={out_pdf}", out_html.as_uri()],
                   check=True, capture_output=True)
    print(f"PDF   {out_pdf}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
