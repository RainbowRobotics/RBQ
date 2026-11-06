import { useState } from 'react';
import { View, Text, StyleSheet, BackHandler } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { isDesktop, quitApp } from '@/lib/desktopBridge';
import { t } from '@/lib/i18n';

export function QuitButton() {
  const { c, radius } = useTheme();
  const [ask, setAsk] = useState(false);
  if (!isDesktop()) return null;

  return (
    <>
      <Tappable onPress={() => setAsk(true)} accessibilityLabel={t('앱 종료')}
        style={[styles.btn, { borderColor: c.glassLine, backgroundColor: c.glassHi, borderRadius: radius.md }]}>
        <Icon name="x" size={18} color={c.text} />
      </Tappable>
      {ask && <QuitConfirm onClose={() => setAsk(false)} />}
    </>
  );
}

export function QuitConfirm({ onClose }: { onClose: () => void }) {
  const { c } = useTheme();
  return (
    <Modal onClose={onClose}>
      <View style={[styles.box, { backgroundColor: c.modalA, borderColor: c.line }]}>
        <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('앱을 종료할까요?')}</Text>
        <Text style={{ color: c.dim, fontSize: 11.5, lineHeight: 17 }}>
          {t('로봇은 계속 동작합니다 — 앱만 닫힙니다.')}
        </Text>
        <View style={{ flexDirection: 'row', gap: 10, marginTop: 4 }}>
          <Tappable onPress={onClose} style={[styles.act, { backgroundColor: c.elev, borderColor: c.line }]}>
            <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{t('취소')}</Text>
          </Tappable>
          <Tappable onPress={() => { if (isDesktop()) void quitApp(); else BackHandler.exitApp(); }}
            style={[styles.act, { backgroundColor: c.dangerA, borderColor: c.dangerLine }]}>
            <Text style={{ color: c.onAccent, fontSize: 13, fontWeight: '700' }}>{t('종료')}</Text>
          </Tappable>
        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  btn: { alignItems: 'center', justifyContent: 'center', width: 44, height: 44, borderWidth: 1 },
  box: { width: 300, borderRadius: 16, borderWidth: 1, padding: 18, gap: 8 },
  act: { flex: 1, height: 40, borderRadius: 10, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
