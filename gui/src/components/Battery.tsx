import { View, Text, StyleSheet, Platform } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useDeviceBattery } from '@/lib/deviceBattery';
import { Popover } from '@/components/ui/overlays';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { useFeatureCanFd } from '@/store/capability';
import { chargeView, TAIL_MAX_MIN, type ChargeIssue } from '@/lib/charge';
import { EDGE } from '@/lib/layout';
import { isDesktop } from '@/lib/desktopBridge';
import { useRb } from '@/rb/theme';
import { useTheme } from '@/theme';
import { RBBattery } from '@/rb/components/RBBattery';
import { t } from '@/lib/i18n';

export function Battery({ pct, label, charging = false }: {
  pct: number | null; label?: string; charging?: boolean;
  dense?: boolean;
}) {
  const { c } = useRb();
  const { fonts } = useTheme();
  const has = pct != null;
  const v = has ? Math.max(0, Math.min(100, Math.round(pct))) : undefined;
  return (
    <View style={styles.row} accessibilityLabel={`${label ?? ''} ${has ? `${v}%` : '—'}`}>
      {label ? <Text style={[styles.label, { color: c('fg-subtler') }]}>{label}</Text> : null}
      <RBBattery size="sm" percent={v} status={has ? undefined : 'empty'} charging={charging && has && (v ?? 0) < 100} />
      <Text style={[styles.num, { fontFamily: fonts.mono, color: !has ? c('fg-subtlest') : (v ?? 0) < 25 ? c('fg-danger') : c('fg-default') }]}>{has ? `${v}%` : '—'}</Text>
    </View>
  );
}

export function DeviceBattery(_: { dense?: boolean } = {}) {
  const batt = useDeviceBattery();
  if (!batt) return null;
  const label = isDesktop() ? 'PC' : Platform.OS === 'web' ? t('기기') : t('패드');
  return <Battery pct={batt.pct} label={label} charging={batt.charging} />;
}

const dur = (min: number) => {
  const h = Math.floor(min / 60), m = min % 60;
  return h > 0 ? t('{h}시간 {m}분').replace('{h}', String(h)).replace('{m}', String(m)) : t('{m}분').replace('{m}', String(Math.max(1, m)));
};

const issueText = (k: ChargeIssue) =>
  k === 'no-current' ? t('충전기가 연결됐지만 충전 전류가 들어오지 않습니다. 충전 단자와 충전기를 확인하세요.')
    : k === 'one-pack' ? t('배터리 팩이 1개만 인식됩니다. 충전이 두 배 오래 걸립니다.')
    : t('배터리 팩이 인식되지 않습니다.');

export function BatteryPopover({ onClose }: { onClose: () => void }) {
  const { c: rc } = useRb();
  const { c, fonts } = useTheme();
  const insets = useSafeAreaInsets();
  const on = useRobot((st) => st.conn) === 'connected';
  const pct = useRobot((st) => st.battPct);
  const volt = useRobot((st) => st.battV);
  const fd = useFeatureCanFd();
  const pdu = useTelemetry((st) => st.pdu);
  const packAvg = useTelemetry((st) => st.packAvg);
  const railAvg = useTelemetry((st) => st.chargeAvg);
  const plugSince = useTelemetry((st) => st.plugSince);
  const v = chargeView({ fd, pdu, pct, packAvg, railAvg, plugSince, now: Date.now() });
  const status = !on ? t('연결 안 됨') : pct >= 100 && !v.charging ? t('완충') : !v.charging ? t('충전 안 함')
    : v.to80Min != null ? t('충전 중 · 80%까지 약 {t}').replace('{t}', dur(v.to80Min)) : t('충전 중');
  const tail = on && v.tail ? (pct < 80 ? t('80% 이후 완충까지는 충전기에 따라 최대 약 {t} 더 걸립니다.') : t('80%를 넘으면 느려집니다. 완충까지 충전기에 따라 최대 약 {t} 걸립니다.')).replace('{t}', dur(TAIL_MAX_MIN)) : null;
  const temps = on && fd && pdu ? `${pdu.tempPdu} · ${pdu.tempPs} °C` : null;
  const pos = Platform.OS === 'android' ? { right: EDGE + insets.right, top: 56 } : { right: 0, top: 52 };
  return (
    <Popover onClose={onClose} style={[pos, { width: 280 }]}>
      <View style={styles.block}>
        <Text style={{ color: c.muted, fontSize: 11.5, fontWeight: '700' }}>{t('로봇 배터리')}</Text>
        <View style={{ flexDirection: 'row', alignItems: 'baseline', gap: 8 }}>
          <Text style={{ color: on && pct < 25 ? rc('fg-danger') : c.text, fontSize: 22, fontWeight: '800', fontFamily: fonts.mono }}>{on ? `${Math.round(pct)}%` : '—'}</Text>
          {on && volt > 0 ? <Text style={{ color: c.muted, fontSize: 12, fontFamily: fonts.mono }}>{`${volt.toFixed(1)} V${v.amps != null ? ` · ${v.amps.toFixed(1)} A` : ''}`}</Text> : null}
        </View>
        <Text style={{ color: v.charging ? c.greenTx : c.muted, fontSize: 13, fontWeight: '600' }}>{status}</Text>
        {tail ? <Text style={{ color: c.muted, fontSize: 11.5, lineHeight: 16 }}>{tail}</Text> : null}
        {temps ? <Text style={{ color: c.dim, fontSize: 11, fontFamily: fonts.mono }}>{`${t('팩 온도')} ${temps}`}</Text> : null}
      </View>
      {on && v.issues.map((k) => (
        <Text key={k} style={[styles.issue, { color: c.amberTx, borderColor: c.line2 }]}>{issueText(k)}</Text>
      ))}
    </Popover>
  );
}

const styles = StyleSheet.create({
  block: { paddingHorizontal: 10, paddingVertical: 8, gap: 3 },
  issue: { fontSize: 12, fontWeight: '600', lineHeight: 17, paddingHorizontal: 10, paddingVertical: 8, borderTopWidth: 1 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 3 },
  label: { fontSize: 9, fontWeight: '700' },
  num: { fontSize: 10.5, fontWeight: '700', includeFontPadding: false },
});
