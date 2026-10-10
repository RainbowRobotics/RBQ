import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet, ScrollView } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { gamepadInput, type GamepadDeviceInfo } from '@/lib/gamepad/input';
import { keyLabel, type ButtonRole, type GamepadProfile } from '@/lib/gamepad/profiles';
import { refreshProfile } from '@/lib/gamepad/manager';
import { useGamepadProfiles } from '@/store/gamepadProfiles';
import { t } from '@/lib/i18n';

type AxisKey = 'lx' | 'ly' | 'rx' | 'ry';
type Step =
  | { kind: 'axis'; axis: AxisKey; label: string; hint: string; positive: boolean }
  | { kind: 'dpad'; role: 'DPAD_U' | 'DPAD_D' | 'DPAD_L' | 'DPAD_R'; hat: 'hatY' | 'hatX'; label: string; hint: string }
  | { kind: 'button'; role: ButtonRole; label: string; hint: string };

const STEPS: Step[] = [
  { kind: 'axis', axis: 'ly', label: '왼쪽 스틱 — 위로', hint: '왼쪽 스틱을 위로 끝까지 밀어주세요', positive: false },
  { kind: 'axis', axis: 'lx', label: '왼쪽 스틱 — 오른쪽으로', hint: '왼쪽 스틱을 오른쪽으로 끝까지', positive: true },
  { kind: 'axis', axis: 'ry', label: '오른쪽 스틱 — 위로', hint: '오른쪽 스틱을 위로 끝까지', positive: false },
  { kind: 'axis', axis: 'rx', label: '오른쪽 스틱 — 오른쪽으로', hint: '오른쪽 스틱을 오른쪽으로 끝까지', positive: true },
  { kind: 'dpad', role: 'DPAD_U', hat: 'hatY', label: '왼쪽 방향키 — 위', hint: '왼쪽 방향키(십자키)의 위쪽을 눌러주세요' },
  { kind: 'dpad', role: 'DPAD_D', hat: 'hatY', label: '왼쪽 방향키 — 아래', hint: '왼쪽 방향키의 아래쪽을 눌러주세요' },
  { kind: 'dpad', role: 'DPAD_L', hat: 'hatX', label: '왼쪽 방향키 — 왼쪽', hint: '왼쪽 방향키의 왼쪽을 눌러주세요' },
  { kind: 'dpad', role: 'DPAD_R', hat: 'hatX', label: '왼쪽 방향키 — 오른쪽', hint: '왼쪽 방향키의 오른쪽을 눌러주세요' },
  { kind: 'button', role: 'A', label: '오른쪽 버튼 — 아래 (A·확인)', hint: '오른쪽 버튼 무리의 아래쪽을 눌러주세요 (A 각인이 있으면 A)' },
  { kind: 'button', role: 'B', label: '오른쪽 버튼 — 오른쪽 (B·취소)', hint: '오른쪽 버튼 무리의 오른쪽을 눌러주세요 (B 각인이 있으면 B)' },
  { kind: 'button', role: 'L1', label: '왼쪽 숄더 (L1)', hint: '왼쪽 위 모서리의 숄더 버튼을 눌러주세요' },
  { kind: 'button', role: 'R1', label: '오른쪽 숄더 (R1)', hint: '오른쪽 위 모서리의 숄더 버튼을 눌러주세요' },
];

const AXIS_THRESHOLD = 0.7;
const COOLDOWN_MS = 600;

type Captured = {
  axes: Partial<Record<AxisKey, number>>;
  invert: Partial<Record<AxisKey, boolean>>;
  hatY?: number;
  hatX?: number;
  buttons: Record<number, ButtonRole>;
};

export function GamepadRemapWizard({ dev, onClose }: { dev: GamepadDeviceInfo; onClose: () => void }) {
  const { c, radius, fonts } = useTheme();
  const [idx, setIdx] = useState(0);
  const [skipped, setSkipped] = useState<Set<number>>(new Set());
  const [last, setLast] = useState(t('입력 대기 중…'));
  const cap = useRef<Captured>({ axes: {}, invert: {}, buttons: {} });
  const cooldownUntil = useRef(0);
  const idxRef = useRef(0);
  idxRef.current = idx;

  const advance = () => {
    cooldownUntil.current = Date.now() + COOLDOWN_MS;
    setIdx((i) => i + 1);
  };

  useEffect(() => {
    const offAxes = gamepadInput.onAxes((e) => {
      if (e.deviceId !== dev.id || Date.now() < cooldownUntil.current) return;
      const step = STEPS[idxRef.current];
      if (!step) return;
      const used = new Set(Object.values(cap.current.axes));
      if (cap.current.hatY != null) used.add(cap.current.hatY);
      if (cap.current.hatX != null) used.add(cap.current.hatX);
      let best: { code: number; v: number } | null = null;
      for (const [k, v] of Object.entries(e.axes)) {
        const code = Number(k);
        if (used.has(code) || Math.abs(v) < AXIS_THRESHOLD || code >= 100) continue;
        if (!best || Math.abs(v) > Math.abs(best.v)) best = { code, v };
      }
      if (!best) return;
      setLast(`AXIS ${best.code} = ${best.v.toFixed(2)}`);
      if (step.kind === 'axis') {
        cap.current.axes[step.axis] = best.code;
        const standardSign = step.positive ? 1 : -1;
        if (Math.sign(best.v) !== standardSign) cap.current.invert[step.axis] = true;
        advance();
      } else if (step.kind === 'dpad' && cap.current[step.hat] == null) {
        const hat = step.hat;
        cap.current[hat] = best.code;
        cooldownUntil.current = Date.now() + COOLDOWN_MS;
        setIdx((i) => {
          let n = i + 1;
          for (;;) {
            const s2 = STEPS[n];
            if (!s2 || s2.kind !== 'dpad' || s2.hat !== hat) break;
            n += 1;
          }
          return n;
        });
      }
    });
    const offBtn = gamepadInput.onButton((e) => {
      if (e.deviceId !== dev.id || !e.down || e.repeat !== 0 || Date.now() < cooldownUntil.current) return;
      const step = STEPS[idxRef.current];
      if (!step) return;
      setLast(`${e.label || 'KEYCODE'} (${e.keyCode})`);
      if (step.kind === 'button' || step.kind === 'dpad') {
        cap.current.buttons[e.keyCode] = step.role;
        advance();
      }
    });
    return () => { offAxes(); offBtn(); };
  }, [dev.id]);

  const done = idx >= STEPS.length;
  const save = () => {
    const a = cap.current.axes;
    const profile: GamepadProfile = {
      id: `custom-${dev.descriptor}`,
      label: `${dev.name} ${t('(마법사)')}`,
      axes: {
        lx: a.lx ?? 0, ly: a.ly ?? 1, rx: a.rx ?? 11, ry: a.ry ?? 14,
        ...(cap.current.hatY != null ? { hatY: cap.current.hatY } : {}),
        ...(cap.current.hatX != null ? { hatX: cap.current.hatX } : {}),
      },
      ...(Object.keys(cap.current.invert).length ? { invert: cap.current.invert } : {}),
      buttons: cap.current.buttons,
      presentKeys: Object.keys(cap.current.buttons).map(Number),
    };
    useGamepadProfiles.getState().setProfile(dev.descriptor, profile);
    refreshProfile();
    onClose();
  };
  const skip = () => { setSkipped((s) => new Set(s).add(idx)); advance(); };

  const stepIcon = (i: number) =>
    i < idx ? (skipped.has(i) ? { t: '—', col: c.dim } : { t: '✓', col: c.green })
    : i === idx ? { t: '▶', col: c.accent2 } : { t: '·', col: c.dim };

  return (
    <Modal onClose={onClose}>
      <View style={[styles.card, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
        <View style={{ flexDirection: 'row', alignItems: 'center', marginBottom: 10 }}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '700', flex: 1 }} numberOfLines={1}>
            {t('매핑 마법사 ')}<Text style={{ color: c.dim, fontSize: 10, fontWeight: '500' }}>· {dev.name}</Text>
          </Text>
          <Tappable onPress={onClose}><Icon name="x" size={16} color={c.dim} /></Tappable>
        </View>
        <View style={{ flexDirection: 'row', gap: 16, flex: 1, minHeight: 0 }}>
          <ScrollView style={[styles.steps, { borderRightColor: c.line }]} showsVerticalScrollIndicator={false}>
            {STEPS.map((s, i) => {
              const ic = stepIcon(i);
              return (
                <View key={s.label} style={{ flexDirection: 'row', gap: 8, paddingVertical: 4, alignItems: 'center' }}>
                  <Text style={{ color: ic.col, fontSize: 11, width: 12 }}>{ic.t}</Text>
                  <Text style={{ color: i === idx ? c.text : i < idx ? c.muted : c.dim, fontSize: 11.5, fontWeight: i === idx ? '700' : '400' }}>
                    {t(s.label)}
                  </Text>
                </View>
              );
            })}
          </ScrollView>
          <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', gap: 12 }}>
            {done ? (
              <>
                <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>{t('완료!')}</Text>
                <Text style={{ color: c.muted, fontSize: 11, textAlign: 'center' }}>
                  {t('버튼 ')}{Object.keys(cap.current.buttons).length}{t('개')}
                  {cap.current.hatY != null ? t(' · HAT 십자키') : ''} ·
                  {t('캡처된 키: ')}{Object.keys(cap.current.buttons).map((k) => keyLabel(Number(k))).join(' ') || t('없음')}
                </Text>
                <Tappable onPress={save} style={[styles.btn, { backgroundColor: c.accent, borderColor: c.accent, borderRadius: radius.md }]}>
                  <Icon name="save" size={14} color={c.onAccent} />
                  <Text style={{ color: c.onAccent, fontSize: 12, fontWeight: '600' }}>{t('이 기기 전용 프로파일로 저장')}</Text>
                </Tappable>
              </>
            ) : (
              <>
                <View style={[styles.iconRing, { borderColor: c.accent2 }]}>
                  <Icon name="gamepad" size={30} color={c.accent2} />
                </View>
                <Text style={{ color: c.text, fontSize: 15, fontWeight: '700', textAlign: 'center' }}>{t(STEPS[idx].hint)}</Text>
                <Text style={{ color: c.dim, fontSize: 10, fontFamily: fonts.mono }}>{t('마지막 감지: ')}{last}</Text>
                <View style={{ flexDirection: 'row', gap: 8 }}>
                  <Tappable onPress={() => setIdx((i) => Math.max(0, i - 1))}
                    style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, opacity: idx ? 1 : 0.4 }]}>
                    <Text style={{ color: c.text, fontSize: 12 }}>{t('뒤로')}</Text>
                  </Tappable>
                  <Tappable onPress={skip}
                    style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
                    <Text style={{ color: c.text, fontSize: 12 }}>{t('이 컨트롤 없음 — 건너뛰기')}</Text>
                  </Tappable>
                </View>
                <Text style={{ color: c.dim, fontSize: 9.5 }}>
                  {idx + 1}/{STEPS.length}{t(' 단계 · 완료 시 이 기기 전용 프로파일로 저장됩니다')}
                </Text>
              </>
            )}
          </View>
        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  card: { width: 640, height: 380, borderWidth: 1, padding: 16 },
  steps: { width: 210, flexGrow: 0, flexShrink: 0, borderRightWidth: 1, paddingRight: 12 },
  iconRing: { width: 70, height: 70, borderRadius: 35, borderWidth: 2, borderStyle: 'dashed', alignItems: 'center', justifyContent: 'center' },
  btn: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 34, paddingHorizontal: 14, borderWidth: 1 },
});
