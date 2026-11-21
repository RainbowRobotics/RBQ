import { useCallback, useEffect, useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Toggle, Slider, Select } from '@/components/ui/controls';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { H2, Desc, Group, SBtn } from '@/components/panels/settings/common';
import { TRow, SecGap } from '@/components/panels/PowerPanel';
import { notify } from '@/components/DoneToast';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { useFeatureCanFd } from '@/store/capability';
import { HttpError } from '@/lib/rest';
import { t } from '@/lib/i18n';
import {
  autoSit, fromSlider, toSlider, jointKey, jointPatch, jointLimitKeys, JOINT_INHERIT, JOINT_IGNORE,
  type AutoSitResp, type AutoSitPatch, type AutoSitJoint,
} from '@/lib/autoSit';

type Ask = { title: string; message: string; confirmLabel: string; run: () => void };

function failText(e: unknown): string {
  if (e instanceof HttpError && e.status === 403) return t('제어권을 가진 기기에서만 바꿀 수 있습니다');
  if (e instanceof HttpError && e.status === 404) return t('이 로봇 소프트웨어는 자동 앉기 설정을 지원하지 않습니다 — 업데이트해 주세요');
  return `${t('자동 앉기 설정 실패')}: ${(e as Error)?.message ?? e}`;
}

export function AutoSitPanel() {
  const { c } = useTheme();
  const conn = useRobot((s) => s.conn);
  const isMine = useRobot((s) => s.isMine);
  const canFd = useFeatureCanFd();
  const [resp, setResp] = useState<AutoSitResp | null>(null);
  const [loadErr, setLoadErr] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [ask, setAsk] = useState<Ask | null>(null);
  const [open, setOpen] = useState<number | null>(null);

  const tempKey = useTelemetry((s) => (s.robot?.joints ?? []).map((j) => `${j.statorTemp},${j.temperature}`).join('|'));
  const [drag, setDrag] = useState<{ key: string; v: number } | null>(null);

  const load = useCallback(() => {
    setLoadErr(null);
    autoSit.get().then(setResp, (e) => setLoadErr(failText(e)));
  }, []);
  useEffect(() => { if (conn === 'connected') load(); }, [conn, load]);

  const send = (patch: AutoSitPatch) => {
    setBusy(true);
    autoSit.put(patch)
      .then((r) => { setResp(r); if (r.pending) notify(t('로봇이 아직 반영하지 않았습니다 — 잠시 뒤 다시 확인해 주세요'), 'warn'); })
      .catch((e) => { notify(failText(e), 'warn'); load(); })
      .finally(() => { setBusy(false); setDrag(null); });
  };

  if (!resp) {
    return <Desc>{loadErr ?? (conn === 'connected' ? t('설정을 읽는 중…') : t('로봇에 연결되면 설정을 읽습니다.'))}</Desc>;
  }

  const p = resp.auto_sit;
  const d = resp.defaults;
  const lim = resp.limits;
  const blocked = isMine ? null : t('제어권을 가진 기기에서만 바꿀 수 있습니다 — 상단에서 제어권을 가져오세요.');
  const can = isMine && !busy;
  const dim = { opacity: isMine ? 1 : 0.45 } as const;

  const sliderRow = (key: string, nm: string, value: number, range: [number, number], step: number,
    subOf: (v: number) => string, commit: (v: number) => void) => {
    const shown = drag?.key === key ? drag.v : value;
    return (
      <TRow nm={nm} sub={subOf(shown)}
        right={
          <View pointerEvents={can ? 'auto' : 'none'} style={[{ width: 200 }, dim]}>
            <Slider value={toSlider(shown, range)} width="100%"
              onChange={(pos) => setDrag({ key, v: fromSlider(pos, range, step) })}
              onCommit={(pos) => { const v = fromSlider(pos, range, step); if (v !== value) commit(v); else setDrag(null); }} />
          </View>
        } />
    );
  };

  const offToggle = (on: boolean, title: string, message: string, set: (v: boolean) => void) => (
    <View pointerEvents={can ? 'auto' : 'none'} style={dim}>
      <Toggle value={on} onChange={(v) => (v ? set(true) : setAsk({ title, message, confirmLabel: t('끄기'), run: () => set(false) }))} />
    </View>
  );

  const temps = (j: number) => {
    const s = tempKey.split('|')[j];
    if (!s) return '';
    const [m, b] = s.split(',');
    if (m === '0' && b === '0') return '';
    return `${t('모터')} ${m}°C · ${t('보드')} ${b}°C`;
  };

  const jointSummary = (j: AutoSitJoint) => {
    const part = (kind: string, ign: boolean, cv: number | null) =>
      ign ? `${kind} ${t('센서 제외')}` : cv != null ? `${kind} ${cv}°C ${t('넘으면')}` : null;
    const parts = [part(t('모터'), j.motor_ignored, j.motor_c), part(t('보드'), j.board_ignored, j.board_c)].filter(Boolean);
    return parts.length ? parts.join(' · ') : t('전체 기준');
  };

  const limitOptions = (global: number) => [
    { key: JOINT_INHERIT, label: `${t('전체 기준')} (${global}°C)` },
    ...jointLimitKeys(lim.temp_c).map((k) => ({ key: k, label: `${k}°C` })),
    { key: JOINT_IGNORE, label: t('센서 제외 (고장)') },
  ];

  const setJoint = (j: AutoSitJoint, kind: 'motor' | 'board', key: string) => {
    const go = () => send({ temperature: { joints: [{ index: j.index, ...jointPatch(kind, key) }] } });
    if (key !== JOINT_IGNORE) return go();
    setAsk({
      title: t('{j} {k} 온도 센서를 뺄까요?').replace('{j}', j.name).replace('{k}', kind === 'motor' ? t('모터') : t('보드')),
      message: t('이 센서는 앉기·경고·상태 표시에서 모두 빠집니다. 센서가 고장 나 실제와 다른 값을 보일 때만 빼세요 — 실제로 뜨거워져도 이 센서로는 앉지 않습니다.'),
      confirmLabel: t('센서 제외'), run: go,
    });
  };

  return (
    <>
      <Desc>{t('로봇이 서 있을 때 배터리가 부족하거나 관절이 과열되면 스스로 앉습니다. 그 조건을 바꿉니다 — 바꾼 값은 바로 로봇에 저장되고 다시 켜도 유지됩니다.')}</Desc>
      {blocked && (
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6, marginBottom: 12 }}>
          <Icon name="warn" size={13} color={c.redTx} />
          <Text style={{ color: c.redTx, fontSize: 12 }}>{blocked}</Text>
        </View>
      )}

      <H2>{t('배터리 부족')} <Text style={{ color: c.muted, fontSize: 11 }}>Low Battery</Text></H2>
      <Desc>{t('배터리가 기준 아래로 떨어지면 서 있던 로봇이 앉습니다. 끄면 배터리가 바닥날 때까지 걷다가 전원이 꺼져 쓰러질 수 있습니다.')}</Desc>
      <Group>
        <TRow nm={t('배터리 부족 시 자동 앉기')}
          sub={p.battery.enabled ? t('켜짐') : t('꺼짐 — 경고만 남기고 앉지 않습니다')}
          subColor={p.battery.enabled ? undefined : c.redTx}
          right={offToggle(p.battery.enabled, t('배터리 부족 자동 앉기를 끌까요?'),
            t('배터리가 바닥날 때까지 걷다가 전원이 꺼져 로봇이 쓰러질 수 있습니다. 경고와 상태 표시는 그대로 남습니다.'),
            (v) => send({ battery: { enabled: v } }))} />
        {canFd
          ? sliderRow('soc', t('앉는 기준'), p.battery.soc_percent, lim.soc_percent, 0.5,
              (v) => t('배터리 {v}% 이하 (기본 {d}%)').replace('{v}', String(v)).replace('{d}', String(d.soc_percent)),
              (v) => send({ battery: { soc_percent: v } }))
          : sliderRow('volt', t('앉는 기준'), p.battery.voltage, lim.voltage, 0.5,
              (v) => t('배터리 {v} V 미만 (기본 {d} V)').replace('{v}', String(v)).replace('{d}', String(d.voltage)),
              (v) => send({ battery: { voltage: v } }))}
      </Group>
      <SecGap />

      <H2>{t('관절 과열')} <Text style={{ color: c.muted, fontSize: 11 }}>Joint Overheat</Text></H2>
      <Desc>{t('관절 모터나 보드 온도가 기준을 넘으면 서 있던 로봇이 앉습니다. 기준은 기본값보다 낮출 수만 있습니다.')}</Desc>
      <Group>
        <TRow nm={t('관절 과열 시 자동 앉기')}
          sub={p.temperature.enabled ? t('켜짐') : t('꺼짐 — 경고만 남기고 앉지 않습니다')}
          subColor={p.temperature.enabled ? undefined : c.redTx}
          right={offToggle(p.temperature.enabled, t('관절 과열 자동 앉기를 끌까요?'),
            t('관절이 과열돼도 로봇이 앉지 않습니다. 모터·보드가 손상될 수 있습니다. 경고와 상태 표시는 그대로 남습니다.'),
            (v) => send({ temperature: { enabled: v } }))} />
        {sliderRow('motor', t('모터 온도 기준'), p.temperature.motor_c, lim.temp_c, 1,
          (v) => t('{v}°C 를 넘으면 (기본 {d}°C)').replace('{v}', String(v)).replace('{d}', String(d.motor_c)),
          (v) => send({ temperature: { motor_c: v } }))}
        {sliderRow('board', t('보드 온도 기준'), p.temperature.board_c, lim.temp_c, 1,
          (v) => t('{v}°C 를 넘으면 (기본 {d}°C)').replace('{v}', String(v)).replace('{d}', String(d.board_c)),
          (v) => send({ temperature: { board_c: v } }))}
      </Group>
      <SecGap />

      <H2>{t('관절별 설정')} <Text style={{ color: c.muted, fontSize: 11 }}>Per Joint</Text></H2>
      <Desc>{t('온도 센서가 고장 나 실제와 다른 값을 보이는 관절은 그 센서를 빼거나, 그 관절만 기준을 낮춥니다. 관절을 눌러 바꿉니다.')}</Desc>
      <Group>
        {p.temperature.joints.map((j) => {
          const custom = j.motor_ignored || j.board_ignored || j.motor_c != null || j.board_c != null;
          const now = temps(j.index);
          return (
            <View key={j.index}>
              <Tappable onPress={() => setOpen(open === j.index ? null : j.index)} accessibilityLabel={j.name}>
                <TRow nm={j.name} sub={now ? `${now} · ${jointSummary(j)}` : jointSummary(j)}
                  subColor={j.motor_ignored || j.board_ignored ? c.redTx : custom ? c.accent2 : undefined}
                  right={<Icon name={open === j.index ? 'chevL' : 'chevR'} size={14} color={c.muted} />} />
              </Tappable>
              {open === j.index && (
                <View pointerEvents={can ? 'auto' : 'none'} style={[{ flexDirection: 'row', gap: 12, paddingBottom: 12 }, dim]}>
                  <View style={{ flex: 1, gap: 4 }}>
                    <Text style={{ color: c.muted, fontSize: 11 }}>{t('모터 온도')}</Text>
                    <Select options={limitOptions(p.temperature.motor_c)} value={jointKey(j.motor_ignored, j.motor_c)}
                      onChange={(k) => setJoint(j, 'motor', k)} />
                  </View>
                  <View style={{ flex: 1, gap: 4 }}>
                    <Text style={{ color: c.muted, fontSize: 11 }}>{t('보드 온도')}</Text>
                    <Select options={limitOptions(p.temperature.board_c)} value={jointKey(j.board_ignored, j.board_c)}
                      onChange={(k) => setJoint(j, 'board', k)} />
                  </View>
                </View>
              )}
            </View>
          );
        })}
      </Group>
      <SecGap />

      <View pointerEvents={can ? 'auto' : 'none'} style={[{ alignItems: 'flex-start' }, dim]}>
        <SBtn kind="ghost" icon="recover" label={t('모두 기본값으로')} disabled={!can}
          onPress={() => setAsk({
            title: t('자동 앉기 설정을 모두 기본값으로 되돌릴까요?'),
            message: t('배터리·관절 과열 자동 앉기가 켜지고, 기준과 관절별 설정이 모두 기본값으로 돌아갑니다.'),
            confirmLabel: t('기본값으로 되돌리기'), run: () => send({ reset: true }),
          })} />
      </View>

      {ask && (
        <ConfirmModal title={ask.title} message={ask.message} confirmLabel={ask.confirmLabel} danger
          onConfirm={() => { ask.run(); setAsk(null); }} onClose={() => setAsk(null)} />
      )}
    </>
  );
}
