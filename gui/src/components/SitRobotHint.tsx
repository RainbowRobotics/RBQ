import { View, Text, Image, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { useSitImuCalibGuard } from '@/lib/commissioning';
import { t } from '@/lib/i18n';

const SIT_ROBOT = require('@/assets/images/sit-robot.png');

export function SitRobotHint({ children }: { children?: React.ReactNode }) {
  const { c, radius } = useTheme();
  const guard = useSitImuCalibGuard();
  return (
    <View style={{ alignItems: 'center', marginTop: 12, gap: 8 }}>
      <View style={[styles.plate, { backgroundColor: c.elev, borderRadius: radius.md }]}>
        <Image source={SIT_ROBOT} style={{ width: 210, height: 140 }} resizeMode="contain" />
      </View>
      <Text style={{ color: c.text, fontSize: 12, fontWeight: '600', textAlign: 'center' }}>
        {t('로봇을 앉힌 상태로 두십시오.')}
      </Text>
      {children}
      <Text style={{ color: guard.blocked ? c.redTx : c.greenTx, fontSize: 11.5, fontWeight: '600', textAlign: 'center', lineHeight: 16 }}>
        {guard.blocked ? `⛔ ${guard.reason}` : `✓ ${t('앉음 · IMU 연결됨 — 실행할 수 있습니다')}`}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  plate: { padding: 8 },
});
