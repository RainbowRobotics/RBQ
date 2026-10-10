import { useEffect, useState } from 'react';
import { View, Text, StyleSheet, Pressable } from 'react-native';
import { ptt, usePtt } from '@/lib/ptt';
import { useTelemetry } from '@/store/telemetry';
import { t } from '@/lib/i18n';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import type { ToolItem } from '@/components/control/LeftPanel';
import { media, useMedia, mmss } from '@/lib/media';
import { Popover } from '@/components/ui/overlays';

export const DOCK_BTN_W = 46;
export const DOCK_BTN_H = 42;
export const DOCK_GAP = 6;
export const TINY_BTN = 34;

export function ToolDock({ tools, left, bottom, compact, tiny }: { tools: ToolItem[]; left: number; bottom: number; compact?: boolean; tiny?: boolean }) {
  const { c, radius } = useTheme();
  if (!tools.length) return null;
  return (
    <View style={[styles.row, { left, bottom, gap: compact ? 4 : DOCK_GAP }]} pointerEvents="box-none">
      {tools.map((it) => (
        <Tappable key={it.key} onPress={it.onPress} accessibilityLabel={it.label}
          style={[styles.btn, compact && { width: 40, height: 40 }, tiny && { width: TINY_BTN, height: TINY_BTN }, {
            borderRadius: radius.md,
            backgroundColor: it.open ? 'rgba(77,156,245,0.22)' : c.glass,
            borderColor: it.open ? 'rgba(77,156,245,0.6)' : c.glassLine,
          }]}>
          <Icon name={it.icon} size={16} color={it.open ? c.accent2 : c.text} />
          {!tiny && <Text numberOfLines={1} style={{ color: it.open ? c.accent2 : c.text, fontSize: 8.5, fontWeight: '700' }}>{it.label}</Text>}
          {it.dot && <View style={[styles.dot, { backgroundColor: c.green }]} />}
        </Tappable>
      ))}
    </View>
  );
}

export function PttButton({ right, bottom, compact, tiny }: { right: number; bottom: number; compact?: boolean; tiny?: boolean }) {
  const { c, radius } = useTheme();
  const speaking = usePtt((s) => s.speaking);
  const error = usePtt((s) => s.error);
  const ampOff = useTelemetry((s) => s.pdu?.amp === false);
  const tint = speaking ? c.green : error ? c.amber : c.text;
  useEffect(() => () => ptt.stopSpeak(), []);
  return (
    <Pressable
      accessibilityLabel={t('워키토키')}
      disabled={ampOff}
      onPressIn={() => ptt.startSpeak()}
      onPressOut={() => ptt.stopSpeak()}
      style={[styles.btn, styles.ptt, compact && { width: 40, height: 40 }, tiny && { width: TINY_BTN, height: TINY_BTN },
        { touchAction: 'none', userSelect: 'none' } as any,
        { right, bottom, borderRadius: radius.md, opacity: ampOff ? 0.45 : 1,
          backgroundColor: speaking ? 'rgba(63,185,80,0.22)' : c.glass,
          borderColor: speaking ? 'rgba(63,185,80,0.7)' : error ? 'rgba(210,153,34,0.6)' : c.glassLine }]}>
      <Icon name="mic" size={16} color={tint} />
      {!tiny && <Text numberOfLines={1} style={{ color: tint, fontSize: 8.5, fontWeight: '700' }}>{speaking ? t('송출 중') : t('워키토키')}</Text>}
    </Pressable>
  );
}

function useVideoElapsed(since: number) {
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    if (!since) return;
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, [since]);
  return since ? Math.max(0, (now - since) / 1000) : 0;
}

export function MediaDock({ right, bottom, compact, tiny, soundOpen, onSound, vertical }: {
  right: number; bottom: number; compact?: boolean; tiny?: boolean; soundOpen: boolean; onSound: () => void;
  vertical?: boolean;
}) {
  const { c, radius } = useTheme();
  const w = tiny ? TINY_BTN : compact ? 40 : DOCK_BTN_W;
  const h = tiny ? TINY_BTN : compact ? 40 : DOCK_BTN_H;
  const gap = compact || tiny ? 4 : DOCK_GAP;
  const shooting = useMedia((s) => s.shooting);
  const videoSince = useMedia((s) => s.videoSince);
  const videoBusy = useMedia((s) => s.videoBusy);
  const playing = useMedia((s) => s.playingPath != null);
  const secs = useVideoElapsed(videoSince);
  const rec = videoSince !== 0;
  const [pick, setPick] = useState(false);
  const box = (on: boolean, tone: 'blue' | 'red' = 'blue') => [styles.btn, { width: w, height: h }, {
    borderRadius: radius.md,
    backgroundColor: on ? (tone === 'red' ? 'rgba(231,51,28,0.22)' : 'rgba(77,156,245,0.22)') : c.glass,
    borderColor: on ? (tone === 'red' ? 'rgba(231,51,28,0.7)' : 'rgba(77,156,245,0.6)') : c.glassLine,
  }];
  const popBottom = bottom + (vertical ? (h + gap) * 3 : h + gap) + 2;
  const onCapture = () => {
    if (videoBusy) return;
    if (rec) { void media.toggleVideo(); return; }
    setPick(!pick);
  };
  const choice = (icon: React.ReactNode, label: string, onPress: () => void) => (
    <Tappable onPress={() => { setPick(false); onPress(); }} accessibilityLabel={label}
      style={[styles.choice, { borderColor: c.line, backgroundColor: c.elev, borderRadius: radius.md }]}>
      {icon}
      <Text style={{ color: c.text, fontSize: 12, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
  return (
    <>
      <View style={[styles.row, vertical
        ? { right, bottom: bottom + h + gap, gap, flexDirection: 'column' }
        : { right: right + w + gap, bottom, gap }]} pointerEvents="box-none">
        <Tappable onPress={onSound} accessibilityLabel={t('사운드')} style={box(soundOpen)}>
          <Icon name="speaker" size={16} color={soundOpen ? c.accent2 : c.text} />
          {!tiny && <Text numberOfLines={1} style={{ color: soundOpen ? c.accent2 : c.text, fontSize: 8.5, fontWeight: '700' }}>{t('사운드')}</Text>}
          {playing && <View style={[styles.dot, { backgroundColor: c.green }]} />}
        </Tappable>
        <Tappable onPress={onCapture} accessibilityLabel={rec ? t('녹화 정지') : t('촬영')} style={box(rec || pick || shooting, rec ? 'red' : 'blue')}>
          {rec
            ? <View style={{ width: 9, height: 9, borderRadius: 5, backgroundColor: c.redbright, opacity: videoBusy ? 0.4 : 1 }} />
            : <Icon name="camera" size={16} color={pick || shooting ? c.accent2 : c.text} />}
          {(!tiny || rec) && (
            <Text numberOfLines={1} style={{ color: rec ? c.redbright : pick || shooting ? c.accent2 : c.text, fontSize: tiny ? 7.5 : 8.5, fontWeight: '700' }}>
              {rec ? mmss(secs) : t('촬영')}
            </Text>
          )}
        </Tappable>
      </View>
      {pick && (
        <Popover onClose={() => setPick(false)} scroll={false} style={{ right, bottom: popBottom, width: 200 }}>
          <View style={{ flexDirection: 'row', gap: 8 }}>
            {choice(<Icon name="camera" size={18} color={c.accent2} />, t('사진'), () => { void media.snapshot(); })}
            {choice(<View style={{ width: 12, height: 12, borderRadius: 6, backgroundColor: c.redbright }} />, t('녹화'), () => { void media.toggleVideo(); })}
          </View>
        </Popover>
      )}
    </>
  );
}

export const rightDockWidth = (compact?: boolean, tiny?: boolean) =>
  (tiny ? TINY_BTN * 3 + 4 * 2 : compact ? 40 * 3 + 4 * 2 : DOCK_BTN_W * 3 + DOCK_GAP * 2);

const styles = StyleSheet.create({
  ptt: { position: 'absolute', zIndex: 9 },
  row: { position: 'absolute', flexDirection: 'row', gap: DOCK_GAP, zIndex: 9 },
  btn: { width: DOCK_BTN_W, height: DOCK_BTN_H, borderWidth: 1, alignItems: 'center', justifyContent: 'center', gap: 2 },
  dot: { position: 'absolute', right: 5, top: 5, width: 6, height: 6, borderRadius: 3 },
  choice: { flex: 1, height: 64, borderWidth: 1, alignItems: 'center', justifyContent: 'center', gap: 6 },
});
