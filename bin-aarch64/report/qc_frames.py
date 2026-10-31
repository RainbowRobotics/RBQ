#!/usr/bin/env python3
"""QC 녹화에서 프레임을 뽑아 밀착 인화를 만든다 — 보고서 '사진' 절과 Claude 참고 소견이 같이 쓴다.

Claude 는 동영상을 받지 않는다. 그래서 녹화는 로컬에 증거로 두고, 일정 간격 프레임과
그걸 한 장에 모은 밀착 인화(contact sheet)만 넘긴다. 밀착 인화 한 장이면 "화면이 지령
방향으로 흘렀나" 같은 시간 흐름을 이미지 한 장으로 볼 수 있다. 사람도 보고서에서 이걸 본다.

    python3 qc_frames.py <QC 보고서 CSV> [--every 2.0] [--cols 4]

CSV 옆의 같은 이름 폴더(<시리얼>_<시각>/)에서 *.mkv(녹화 탭) · *.mp4 · *.avi 를 찾아
  <폴더>/frames/<영상이름>/t0000.0.jpg …   각 프레임(파일 이름 = 영상 안 시각 [s])
  <폴더>/frames/<영상이름>_sheet_NN.jpg    프레임 16장씩 모은 밀착 인화
  <폴더>/frames/sheets.json                밀착 인화 목록 — 장마다 영상 몇 초~몇 초인지
를 만든다. 칸마다 영상 안 시각과 그때 돌던 QC 단계(CSV 단계표의 녹화[s])를 박는다(09-29).
한글 글꼴(fc-list :lang=ko)이 없으면 단계 번호만 박는다. 영상이 없으면 아무것도 안 하고 알린다.
ffmpeg 만 쓴다(OpenCV 불필요).

보고서 쪽(qc_report.py · qc_web.py)은 ensure() 를 부른다 — 밀착 인화가 없거나 녹화보다 오래됐거나
단계 글자가 없는 옛 판이면 그 영상만 다시 만든다(한 건에 10 초 안팎).
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

VIDEO_EXT = (".mp4", ".mkv", ".avi", ".mov")
SHEET_N = 16          # 밀착 인화 한 장의 프레임 수 — 4×4. 글자가 읽히는 크기를 지키는 선
LABELS = "step-1"     # sheets.json 표지 — 칸 글자 모양이 바뀌면 올려서 옛 밀착 인화를 다시 만들게 한다
CAM_NAME = {"zoom": "줌", "thermal": "열화상"}
SNAP_CAPTION = {
    "snap_zoom.jpg": "줌 카메라 — 12단계(카메라 영상)가 마지막으로 받은 화면",
    "snap_thermal.jpg": "열화상 카메라 — 12단계(카메라 영상)가 마지막으로 받은 화면",
}


def media_dir(csv_path: Path) -> Path:
    return csv_path.with_suffix("")


def duration_s(video: Path) -> float:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(video)],
        capture_output=True, text=True, check=True).stdout.strip()
    try:
        return float(out)
    except ValueError:
        # 녹화가 강제로 끊긴 mkv 는 머리에 길이가 없다(N/A) — 마지막 프레임 시각으로 잰다.
        pts = subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "packet=pts_time",
             "-of", "csv=p=0", str(video)], capture_output=True, text=True).stdout.split()
        vals = [float(x) for x in pts if x.replace(".", "", 1).isdigit()]
        return max(vals) if vals else 0.0


def extract(video: Path, out_dir: Path, every: float, until: float | None = None) -> list[Path]:
    """until — 이 녹화 시각[s]까지만(검사가 끝난 뒤까지 녹화가 이어졌을 때). None 이면 끝까지."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for old in out_dir.glob("t*.jpg"):
        old.unlink()
    dur = duration_s(video)
    if until is not None:
        dur = min(dur, until)
    frames = []
    t = 0.0
    while t < dur:
        dst = out_dir / f"t{t:06.1f}.jpg"
        # -ss 를 입력 앞에 두면 키프레임으로 빠르게 찾아간다. 초 단위 정밀도면 충분하다.
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{t:.2f}", "-i", str(video),
                        "-frames:v", "1", "-vf", "scale=640:-2", "-q:v", "3", str(dst)], check=True)
        if dst.exists():
            frames.append(dst)
        t += every
    return frames


# ---------------------------------------------------------------- 칸 글자 — 그 시각에 돌던 단계
def step_marks(csv_path: Path) -> tuple[list[tuple[float, str]], float | None]:
    """CSV 단계표의 녹화[s](단계가 시작한 시각, 녹화 기준) → [(시각, '7 홈잉')], 검사가 끝난 시각(대략)."""
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from qc_report import read_qc_csv
        qc = read_qc_csv(csv_path)
    except Exception:                              # CSV 가 없거나 모양이 다르면 시각만 박는다
        return [], None
    marks = []
    for s in qc["steps"]:
        try:
            marks.append((float(s.get("rec") or ""), s["name"]))
        except ValueError:
            continue
    marks.sort()
    end = None
    try:
        # 단계 시각은 녹화 기준, 소요는 검사 시작 기준이다 — 첫 단계 시각 + 소요 ≈ 검사가 끝난 녹화 시각
        end = marks[0][0] + float(qc["head"].get("소요[s]", "")) if marks else None
    except ValueError:
        end = None
    return marks, end


def step_at(t: float, marks: list[tuple[float, str]], end: float | None) -> str:
    if not marks:
        return ""
    if t + 0.05 < marks[0][0]:
        return "검사 전"
    if end is not None and t > end + 0.5:
        return "검사 끝"
    name = ""
    for at, n in marks:
        if at <= t + 0.05:
            name = n
    return name


def korean_font() -> str | None:
    """한글이 되는 글꼴 파일 — 없으면 None(단계 번호만 박는다)."""
    if not shutil.which("fc-list"):
        return None
    out = subprocess.run(["fc-list", ":lang=ko", "file"], capture_output=True, text=True).stdout.split("\n")
    files = [x.split(":")[0].strip() for x in out if x.strip()]
    for pref in ("Sans", ""):
        for f in files:
            if pref in Path(f).name and Path(f).exists():
                return f
    return None


def _esc_filter(p: str) -> str:
    return p.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")


def sheets(frames: list[Path], out_prefix: Path, cols: int, marks=(), end=None) -> list[Path]:
    """프레임 SHEET_N 장씩 한 장에 모은다. 칸마다 영상 안 시각과 그때 돌던 단계를 박는다 —
    사람과 Claude 가 "몇 초에 무엇"을 말할 수 있게."""
    out = []
    for old in out_prefix.parent.glob(f"{out_prefix.name}_sheet_*.jpg"):
        old.unlink()
    font = korean_font() if marks else None
    tmp = Path(tempfile.mkdtemp(prefix="qcframes_"))
    try:
        for i in range(0, len(frames), SHEET_N):
            chunk = frames[i:i + SHEET_N]
            n = len(chunk)
            dst = out_prefix.parent / f"{out_prefix.name}_sheet_{i // SHEET_N:02d}.jpg"
            inputs, filters = [], []
            for k, f in enumerate(chunk):
                inputs += ["-i", str(f)]
                t = float(f.stem[1:])                   # 파일 이름 t0012.0 → 12.0
                step = step_at(t, list(marks), end)
                if step and not font:                   # 한글 글꼴이 없으면 번호만
                    step = step.split(" ")[0] if step[0].isdigit() else ""
                label = tmp / f"l{i + k:04d}.txt"
                label.write_text(f"{t:.1f} s" + (f"\n{step}" if step else ""), encoding="utf-8")
                ff = f":fontfile='{_esc_filter(font)}'" if font else ""
                filters.append(f"[{k}:v]scale=480:-2,drawtext=textfile='{_esc_filter(str(label))}'{ff}:expansion=none:"
                               f"x=8:y=8:fontsize=22:line_spacing=4:fontcolor=yellow:box=1:boxcolor=black@0.6:boxborderw=4[v{k}]")
            # xstack 레이아웃: 칸 (c, r) 의 x 는 왼쪽 칸 너비의 합, y 는 위쪽 칸 높이의 합
            layout = []
            for k in range(n):
                c, r = k % cols, k // cols
                x = "+".join(f"w{j}" for j in range(c)) or "0"
                y = "+".join(f"h{j * cols}" for j in range(r)) or "0"
                layout.append(f"{x}_{y}")
            tail = (f"xstack=inputs={n}:layout={'|'.join(layout)}:fill=black[out]" if n > 1 else "null[out]")
            filters.append("".join(f"[v{k}]" for k in range(n)) + tail)
            subprocess.run(["ffmpeg", "-v", "error", "-y", *inputs, "-filter_complex", ";".join(filters),
                            "-map", "[out]", "-q:v", "3", str(dst)], check=True)
            out.append(dst)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return out


# ---------------------------------------------------------------- 목록
def videos_in(mdir: Path) -> list[Path]:
    return sorted(p for p in mdir.glob("*") if p.suffix.lower() in VIDEO_EXT and p.stat().st_size > 0) if mdir.is_dir() else []


def _index_path(mdir: Path) -> Path:
    return mdir / "frames" / "sheets.json"


def read_index(mdir: Path) -> dict:
    try:
        j = json.loads(_index_path(mdir).read_text(encoding="utf-8"))
        return j if isinstance(j, dict) else {}
    except (OSError, ValueError):
        return {}


def make(csv_path: Path, video: Path, every: float = 2.0, cols: int = 4) -> list[dict]:
    """영상 하나 → 프레임 · 밀착 인화 · 목록 한 칸. 목록 칸을 돌려준다."""
    mdir = media_dir(csv_path)
    marks, end = step_marks(csv_path)
    # 검사가 끝나고 몇 초까지만 — 녹화가 멈추지 않고 이어진 판(09-29 재검사 녹화가 24분)에서도 검사 구간만 뽑는다
    frames = extract(video, mdir / "frames" / video.stem, every, None if end is None else end + 4.0)
    made = sheets(frames, mdir / "frames" / video.stem, cols, marks, end)
    items = []
    for n, sh in enumerate(made):
        chunk = frames[n * SHEET_N:(n + 1) * SHEET_N]
        items.append({"file": sh.name, "video": video.name, "cam": video.stem, "part": n + 1, "parts": len(made),
                      "from": float(chunk[0].stem[1:]) if chunk else 0.0,
                      "to": float(chunk[-1].stem[1:]) if chunk else 0.0})
    idx = read_index(mdir)
    vids = idx.get("videos") if isinstance(idx.get("videos"), dict) else {}
    vids[video.name] = {"mtime": video.stat().st_mtime, "every": every, "labels": LABELS, "sheets": items}
    _index_path(mdir).parent.mkdir(parents=True, exist_ok=True)
    _index_path(mdir).write_text(json.dumps({"videos": vids}, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return items


def fresh(mdir: Path, video: Path, every: float) -> bool:
    """이 영상의 밀착 인화가 녹화보다 새것이고 지금 글자 모양으로 만든 것인가."""
    e = (read_index(mdir).get("videos") or {}).get(video.name) or {}
    if e.get("labels") != LABELS or e.get("every") != every or not e.get("sheets"):
        return False
    try:
        return all((mdir / "frames" / s["file"]).stat().st_mtime >= video.stat().st_mtime for s in e["sheets"])
    except (OSError, KeyError, TypeError):
        return False


def ensure(csv_path: Path, every: float = 2.0, cols: int = 4, log=print) -> list[dict]:
    """보고서가 부른다 — 필요한 영상만 다시 만들고 밀착 인화 목록을 돌려준다. ffmpeg 가 없으면 빈 목록."""
    mdir = media_dir(csv_path)
    vids = videos_in(mdir)
    if not vids:
        return []
    if not shutil.which("ffmpeg"):
        log("프레임 — ffmpeg 가 없어 밀착 인화를 못 만듦(sudo apt install ffmpeg)")
        return sheet_list(mdir)
    for v in vids:
        if fresh(mdir, v, every):
            continue
        try:
            items = make(csv_path, v, every, cols)
            log(f"프레임 — {v.name}: 밀착 인화 {len(items)}장")
        except (subprocess.CalledProcessError, OSError) as e:
            log(f"프레임 — {v.name}: 못 만듦 ({e})")
    return sheet_list(mdir)


def sheet_list(mdir: Path) -> list[dict]:
    """밀착 인화 목록(카메라 순서: 줌 → 열화상 → 나머지). 목록 파일이 없으면 파일 이름에서 짐작한다."""
    out = []
    vids = read_index(mdir).get("videos") or {}
    for name, e in vids.items():
        for s in e.get("sheets") or []:
            if (mdir / "frames" / s.get("file", "")).exists():
                out.append(dict(s, every=e.get("every")))
    if not out and (mdir / "frames").is_dir():               # 옛 판(목록 없음) — 파일 이름만으로
        for p in sorted((mdir / "frames").glob("*_sheet_*.jpg")):
            cam = p.name.rsplit("_sheet_", 1)[0]
            out.append({"file": p.name, "cam": cam, "video": "", "part": int(p.stem.rsplit("_", 1)[1]) + 1,
                        "parts": 0, "from": None, "to": None})
    order = {"zoom": 0, "thermal": 1}
    out.sort(key=lambda s: (order.get(s.get("cam"), 9), s.get("cam", ""), s.get("part", 0)))
    return out


def sheet_caption(s: dict) -> str:
    """밀착 인화 한 장의 설명 — 어느 카메라 몇 초~몇 초. 칸 글자 풀이는 보고서 사진 절 머리가 한 번 한다(sheet_note)."""
    cam = CAM_NAME.get(s.get("cam"), s.get("cam") or "녹화")
    rng = (f" {s['from']:.0f}~{s['to']:.0f} 초" if s.get("from") is not None and s.get("to") is not None else "")
    part = f" ({s['part']}/{s['parts']})" if s.get("parts") and s["parts"] > 1 else ""
    return f"{cam} 녹화{rng}{part}"


def sheet_note(photos: list[dict]) -> str:
    """사진 절 머리 한 줄 — 밀착 인화가 있을 때만. 간격은 만든 값(sheets.json every)을 쓴다."""
    ev = next((p.get("every") for p in photos if p.get("sheet") and p.get("every")), None)
    if not any(p.get("sheet") for p in photos):
        return ""
    gap = f"{ev:g}초마다" if ev else "일정한 간격으로"
    return f"녹화 사진은 영상에서 {gap} 뽑은 화면을 모은 것이고, 칸마다 녹화 시각과 그때 돌던 단계를 적었다."


def photo_list(mdir: Path) -> list[dict]:
    """보고서 '사진' 절 — 스냅샷(폴더 바로 아래 *.jpg) 뒤에 밀착 인화 전부. path 는 mdir 기준."""
    out = []
    if mdir.is_dir():
        order = list(SNAP_CAPTION)                     # 줌 → 열화상 → 나머지(이름순) — 밀착 인화와 같은 순서
        snaps = sorted(list(mdir.glob("*.jpg")) + list(mdir.glob("*.png")),
                       key=lambda p: (order.index(p.name) if p.name in order else len(order), p.name))
        for p in snaps:
            out.append({"path": p.name, "file": p.name, "caption": SNAP_CAPTION.get(p.name, p.name), "sheet": False})
    for s in sheet_list(mdir):
        out.append({"path": f"frames/{s['file']}", "file": s["file"], "caption": sheet_caption(s), "sheet": True,
                    "every": s.get("every")})
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="QC 탭이 남긴 <시리얼>_<시각>.csv")
    ap.add_argument("--every", type=float, default=2.0, help="프레임 간격 [s] (기본 2)")
    ap.add_argument("--cols", type=int, default=4, help="밀착 인화 열 수 (기본 4)")
    a = ap.parse_args()

    if not shutil.which("ffmpeg"):
        print("ffmpeg 가 없습니다 — sudo apt install ffmpeg", file=sys.stderr)
        return 2
    mdir = media_dir(a.csv)
    videos = videos_in(mdir)
    if not videos:
        print(f"영상 없음 — {mdir}/ 에 *.mp4 등을 두면 프레임을 뽑습니다")
        return 0
    for v in videos:
        items = make(a.csv, v, a.every, a.cols)
        print(f"{v.name}: 프레임 → {mdir / 'frames' / v.stem}/, 밀착 인화 {len(items)}장")
    return 0


if __name__ == "__main__":
    sys.exit(main())
