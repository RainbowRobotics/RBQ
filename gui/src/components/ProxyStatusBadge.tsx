import { useEffect, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { isDesktop } from '@/lib/desktopBridge';
import { t } from '@/lib/i18n';

type ProxyStatus = { running: boolean; detail?: string };

export function ProxyStatusBadge() {
  const [status, setStatus] = useState<ProxyStatus>({ running: true });

  useEffect(() => {
    if (!isDesktop()) return;
    let unlisten: (() => void) | undefined;
    (async () => {
      const { listen, emit } = await import('@tauri-apps/api/event');
      unlisten = await listen<ProxyStatus>('proxy-status', (e) => setStatus(e.payload));
      await emit('request-proxy-status', {});
    })();
    return () => unlisten?.();
  }, []);

  if (!isDesktop() || status.running) return null;
  return (
    <View style={styles.bar} pointerEvents="none">
      <Text style={styles.text}>{t('⚠ 프록시 중단됨 — 로봇 연결 불가')} {status.detail ?? ''}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  bar: {
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    backgroundColor: '#b00020',
    paddingVertical: 6,
    paddingHorizontal: 12,
    zIndex: 9999,
    alignItems: 'center',
  },
  text: { color: '#fff', fontSize: 13, fontWeight: '600' },
});
