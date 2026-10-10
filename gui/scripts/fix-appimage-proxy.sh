#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUNDLE="$ROOT/src-tauri/target/release/bundle/appimage"
APPDIR="$BUNDLE/RBQ.AppDir"
PRISTINE="$ROOT/src-tauri/binaries/rbq-proxy-x86_64-unknown-linux-gnu"
PLUGIN="${TAURI_TOOLS_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/tauri}/linuxdeploy-plugin-appimage.AppImage"

[ -d "$APPDIR" ] || { echo "FAIL: AppDir 없음 — 먼저 'npx tauri build --bundles appimage' 실행: $APPDIR" >&2; exit 1; }
[ -f "$PRISTINE" ] || { echo "FAIL: 원본 사이드카 없음: $PRISTINE" >&2; exit 1; }
[ -x "$PLUGIN" ] || { echo "FAIL: linuxdeploy-plugin-appimage 없음(tauri build가 캐시함): $PLUGIN" >&2; exit 1; }

cp -f "$PRISTINE" "$APPDIR/usr/bin/rbq-proxy"

WRTC_STAGE="$ROOT/src-tauri/binaries/wrtc-modules"
if [ -d "$WRTC_STAGE" ]; then
  rm -rf "$APPDIR/usr/bin/wrtc-modules"
  cp -r "$WRTC_STAGE" "$APPDIR/usr/bin/wrtc-modules"
  echo "wrtc-modules 주입: $APPDIR/usr/bin/wrtc-modules"
else
  echo "주의: wrtc-modules 없음($WRTC_STAGE) — 데스크탑 WebRTC 브리지 비활성. bundle-proxy.mjs 재실행 필요."
fi

FFMPEG_STAGE="$ROOT/src-tauri/binaries/ffmpeg"
if [ -f "$FFMPEG_STAGE" ]; then
  cp -f "$FFMPEG_STAGE" "$APPDIR/usr/bin/ffmpeg"
  chmod +x "$APPDIR/usr/bin/ffmpeg"
  echo "ffmpeg 주입: $APPDIR/usr/bin/ffmpeg"
else
  echo "주의: ffmpeg 없음($FFMPEG_STAGE) — 데스크탑 카메라 영상 비활성(타깃 시스템 ffmpeg 폴백). bundle-proxy.mjs 재실행 필요."
fi

GTK_HOOK="$APPDIR/apprun-hooks/linuxdeploy-plugin-gtk.sh"
if [ -f "$GTK_HOOK" ] && ! grep -q "WEBKIT_DISABLE_DMABUF_RENDERER" "$GTK_HOOK"; then
  printf '\nexport WEBKIT_DISABLE_DMABUF_RENDERER=1\n' >> "$GTK_HOOK"
  echo "DMABUF 비활성 주입: $GTK_HOOK"
fi

if [ -f "$GTK_HOOK" ] && ! grep -q "gameoverlayrenderer" "$GTK_HOOK"; then
  cat >> "$GTK_HOOK" <<'HOOK'

# Drop the Steam overlay preload (it breaks the bundled WebKitGTK under gamescope)
if [ -n "${LD_PRELOAD:-}" ]; then
    LD_PRELOAD="$(printf '%s\n' "$LD_PRELOAD" | tr ':' '\n' | grep -v gameoverlayrenderer | paste -sd: - || true)"
    export LD_PRELOAD
fi
HOOK
  echo "Steam 오버레이 프리로드 제거 주입: $GTK_HOOK"
fi

GST_PLUG_SRC=/usr/lib/x86_64-linux-gnu/gstreamer-1.0
mkdir -p "$APPDIR/usr/lib/gstreamer-1.0"
GST_PLUGS="libgstpulseaudio.so libgstcoreelements.so libgstapp.so libgstaudioconvert.so libgstaudioresample.so libgstvolume.so libgstautodetect.so libgstaudiorate.so libgstaudiotestsrc.so libgstinterleave.so libgstlevel.so libgstaudiomixer.so libgstaudiofx.so libgstpipewire.so libgstplayback.so libgsttypefindfunctions.so"
GST_MISS=""
for p in $GST_PLUGS; do
  if [ -f "$GST_PLUG_SRC/$p" ]; then cp -f "$GST_PLUG_SRC/$p" "$APPDIR/usr/lib/gstreamer-1.0/"; else GST_MISS="$GST_MISS $p"; fi
done
if [ -n "$GST_MISS" ]; then
  echo "FAIL: GStreamer 플러그인 누락 —$GST_MISS" >&2
  echo "      빌드 호스트에 설치 필요: apt install gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-pipewire (setup.bash가 설치)" >&2
  exit 1
fi
echo "GStreamer 마이크 플러그인 동봉: $(ls "$APPDIR/usr/lib/gstreamer-1.0" | wc -l)개"

GST_VIDEO_PLUGS="libgstisomp4.so libgstvideoparsersbad.so libgstopenh264.so libgstvideoconvert.so libgstvideoscale.so"
GST_VIDEO_LIBS="libgstriff-1.0.so.0 libgstrtp-1.0.so.0 libgstcodecparsers-1.0.so.0 libopenh264.so.6"
GST_LIB_SRC=/usr/lib/x86_64-linux-gnu
GST_MISS=""
for p in $GST_VIDEO_PLUGS; do
  if [ -f "$GST_PLUG_SRC/$p" ]; then cp -f "$GST_PLUG_SRC/$p" "$APPDIR/usr/lib/gstreamer-1.0/"; else GST_MISS="$GST_MISS $p"; fi
done
for l in $GST_VIDEO_LIBS; do
  if [ -e "$GST_LIB_SRC/$l" ]; then cp -fL "$GST_LIB_SRC/$l" "$APPDIR/usr/lib/"; else GST_MISS="$GST_MISS $l"; fi
done
if [ -n "$GST_MISS" ]; then
  echo "FAIL: 영상 재생 GStreamer 구성 누락 —$GST_MISS" >&2
  echo "      빌드 호스트에 설치 필요: apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad libopenh264-6" >&2
  exit 1
fi
echo "GStreamer 영상 재생 플러그인 동봉: $GST_VIDEO_PLUGS"

mapfile -t WL_FOUND < <(find "$APPDIR" -name 'libwayland-client.so*' 2>/dev/null)
if [ "${#WL_FOUND[@]}" -eq 0 ]; then
  echo "libwayland-client 번들 없음 — 제거 생략(linuxdeploy 가 애초에 안 넣은 빌드)"
else
  for f in "${WL_FOUND[@]}"; do rm -f "$f"; echo "제거: ${f#$APPDIR/}"; done
  if find "$APPDIR" -name 'libwayland-client.so*' | grep -q .; then
    echo "FAIL: libwayland-client 제거 실패 — 스팀덱에서 흰 화면이 된다" >&2; exit 1
  fi
fi

mapfile -t WK_FOUND < <(find "$APPDIR" -name 'libwebkit*gtk*.so*' -type f 2>/dev/null)
if [ "${#WK_FOUND[@]}" -eq 0 ]; then
  echo "FAIL: libwebkit*gtk*.so 를 AppDir 에서 못 찾았다 — 번들 구성이 바뀌었는지 확인할 것" >&2
  exit 1
fi
WK_DONE=0
for WK_LIB in "${WK_FOUND[@]}"; do
  REL="${WK_LIB#$APPDIR/}"
  if grep -aq '\./\./\+/lib' "$WK_LIB"; then
    echo "이미 치환됨(linuxdeploy 가 처리) — 생략: $REL"
    WK_DONE=$((WK_DONE + 1))
    continue
  fi
  if ! grep -aq '/usr/' "$WK_LIB"; then
    echo "치환 대상 아님(/usr/ 없음) — 생략: $REL"
    continue
  fi
  python3 - "$WK_LIB" "$REL" <<'PYEOF'
import sys
p, rel = sys.argv[1], sys.argv[2]
b = open(p, 'rb').read()
n = b.count(b'/usr/')
open(p, 'wb').write(b.replace(b'/usr/', b'././/'))
print(f"치환: {n} 곳 (/usr/ → ././/) — {rel}")
PYEOF
  if ! grep -aq '\./\./\+/lib' "$WK_LIB"; then
    echo "FAIL: 치환 실패 — 스팀덱에서 코어 덤프한다: $REL" >&2; exit 1
  fi
  WK_DONE=$((WK_DONE + 1))
done
if [ "$WK_DONE" -eq 0 ]; then
  echo "FAIL: 치환된 libwebkit 이 하나도 없다 — 번들 구성이 바뀌었는지 확인할 것" >&2; exit 1
fi

mapfile -t IMGS < <(find "$BUNDLE" -maxdepth 1 -name '*.AppImage' -printf '%T@ %p\n' | sort -rn | cut -d' ' -f2-)
[ "${#IMGS[@]}" -ge 1 ] || { echo "FAIL: 재팩 대상 AppImage 없음" >&2; exit 1; }
APPIMAGE="${IMGS[0]}"
[ "${#IMGS[@]}" -eq 1 ] || echo "주의: AppImage ${#IMGS[@]}개 발견 — mtime 최신 선택: $APPIMAGE (이전 빌드 잔여물은 정리 권장)"

if command -v fuser >/dev/null 2>&1 && fuser -s "$APPIMAGE" 2>/dev/null; then
  echo "FAIL: $APPIMAGE 가 실행 중 — 앱을 종료한 뒤 다시 실행할 것" >&2; exit 1
fi
cd "$BUNDLE"
LDAI_OUTPUT="$(basename "$APPIMAGE")" OUTPUT="$(basename "$APPIMAGE")" ARCH=x86_64 \
    APPIMAGE_EXTRACT_AND_RUN=1 "$PLUGIN" --appdir "$APPDIR" >/dev/null

rm -rf squashfs-root
"$APPIMAGE" --appimage-extract 'usr/bin/rbq-proxy' >/dev/null
if ! cmp -s "$PRISTINE" squashfs-root/usr/bin/rbq-proxy; then
  echo "FAIL: 재팩 후에도 사이드카 불일치" >&2
  rm -rf squashfs-root
  exit 1
fi
echo "OK: 사이드카 원본 복원 확인 — $APPIMAGE"
rm -rf squashfs-root
"$APPIMAGE" --appimage-extract 'usr/lib/libwebkit*gtk*.so*' >/dev/null 2>&1 || true
mapfile -t WK_PACKED < <(find squashfs-root -name 'libwebkit*gtk*.so*' -type f 2>/dev/null)
if [ "${#WK_PACKED[@]}" -eq 0 ]; then
  echo "FAIL: 재팩된 AppImage 에서 libwebkit 을 못 꺼냈다 — 검증 불가" >&2
  rm -rf squashfs-root; exit 1
fi
WK_OK=0
for f in "${WK_PACKED[@]}"; do
  if grep -aq '\./\./\+/lib' "$f"; then
    WK_OK=$((WK_OK + 1))
  elif grep -aq '/usr/lib/.*webkit2gtk' "$f"; then
    echo "FAIL: 재팩 산출물의 ${f#squashfs-root/} 가 미치환 — 덱에서 코어 덤프한다" >&2
    rm -rf squashfs-root; exit 1
  fi
done
if [ "$WK_OK" -eq 0 ]; then
  echo "FAIL: 재팩 산출물에서 치환된 libwebkit 을 확인하지 못했다" >&2
  rm -rf squashfs-root; exit 1
fi
echo "OK: libwebkit 치환 확인(최종 산출물) — ${WK_OK}개"
rm -rf squashfs-root
