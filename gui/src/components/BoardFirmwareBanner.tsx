import { View, Text, Pressable, StyleSheet } from 'react-native';
import { useRouter } from 'expo-router';
import { useTheme } from '@/theme';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useBoardFw } from '@/store/boardFirmware';
import { fwBanners, shortNames, isMotorSlot } from '@/lib/boardFirmware';
import { t } from '@/lib/i18n';

const TONE = '#FBBA16';

export function BoardFirmwareBanner() {
  const { radius } = useTheme();
  const router = useRouter();
  const status = useBoardFw((s) => s.status);
  const panelOpen = useBoardFw((s) => s.panelOpen);
  const dismissed = useBoardFw((s) => s.dismissed);
  if (panelOpen) return null;
  const { update, mixedMotors, hwRecord, noEeprom } = fwBanners(status);
  const motors = status ? status.boards.filter((b) => isMotorSlot(b.slot)) : [];
  const items: { key: string; icon: IconName; text: string; open?: boolean }[] = [
    ...(update.length ? [{ key: `update:${update.map((b) => b.name).join(',')}`, icon: 'cpu' as const, open: true,
      text: `${t('펌웨어 업데이트가 필요한 보드가 있습니다')} (${shortNames(update)})` }] : []),
    ...(mixedMotors ? [{ key: `mixed:${motors.map((m) => `${m.appVersion}:${m.verdict}`).join(',')}`, icon: 'cpu' as const, open: true,
      text: t('모터 펌웨어가 섞여 있습니다 — 펌웨어 업데이트를 하세요') }] : []),
    ...(hwRecord.length ? [{ key: `hw:${hwRecord.map((b) => b.name).join(',')}`, icon: 'warn' as const,
      text: t('hw 기록이 필요한 보드가 있습니다 — 관리자 문의') }] : []),
    ...(noEeprom.length ? [{ key: `eeprom:${noEeprom.map((b) => b.name).join(',')}`, icon: 'warn' as const,
      text: t('EEPROM이 응답하지 않아 대기 중인 보드가 있습니다 — 관리자 문의') }] : []),
  ].filter((it) => !dismissed[it.key]);
  if (!items.length) return null;
  return (
    <>
      {items.map((it) => (
        <View key={it.key} style={[styles.banner, { borderColor: TONE, borderRadius: radius.md }]}>
          <Icon name={it.icon} size={14} color={TONE} />
          <Text style={[styles.txt, { color: TONE }]}>{it.text}</Text>
          {it.open ? (
            <Tappable onPress={() => router.navigate('/maintenance?sec=fw')} accessibilityLabel={t('펌웨어 업데이트 열기')}
              style={[styles.open, { borderColor: TONE, borderRadius: radius.sm }]}>
              <Text style={{ color: TONE, fontSize: 11, fontWeight: '700' }}>{t('열기')}</Text>
            </Tappable>
          ) : null}
          <Pressable onPress={() => useBoardFw.getState().dismiss(it.key)} hitSlop={8} accessibilityLabel={t('닫기')} style={{ marginLeft: 2 }}>
            <Icon name="x" size={13} color={TONE} />
          </Pressable>
        </View>
      ))}
    </>
  );
}

const styles = StyleSheet.create({
  banner: {
    flexDirection: 'row', alignItems: 'center', gap: 8, maxWidth: '92%',
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1,
    backgroundColor: 'rgba(0,0,0,0.55)',
  },
  txt: { fontSize: 11.5, fontWeight: '700', flexShrink: 1 },
  open: { borderWidth: 1, paddingHorizontal: 10, paddingVertical: 3, marginLeft: 4 },
});
