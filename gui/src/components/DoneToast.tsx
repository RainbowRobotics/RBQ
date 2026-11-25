import { useEffect } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { create } from 'zustand';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';

const SHOW_MS = 3200;
const TONE = { ok: '#00C853', warn: '#FBBA16' };

type Toast = { msg: string; tone: keyof typeof TONE; at: number } | null;
const useToast = create<{ toast: Toast }>(() => ({ toast: null }));

export function notify(msg: string, tone: keyof typeof TONE = 'ok') {
  useToast.setState({ toast: { msg, tone, at: Date.now() } });
}

export function DoneToast() {
  const { radius } = useTheme();
  const toast = useToast((s) => s.toast);
  useEffect(() => {
    if (!toast) return;
    const id = setTimeout(() => useToast.setState({ toast: null }), SHOW_MS);
    return () => clearTimeout(id);
  }, [toast]);
  if (!toast) return null;
  const color = TONE[toast.tone];
  return (
    <View pointerEvents="box-none" style={styles.wrap}>
      <Tappable onPress={() => useToast.setState({ toast: null })} style={[styles.card, { borderColor: color, borderRadius: radius.md }]}>
        <Icon name={toast.tone === 'ok' ? 'save' : 'warn'} size={14} color={color} />
        <Text style={{ color, fontSize: 11.5, fontWeight: '700', flexShrink: 1 }} numberOfLines={2}>{toast.msg}</Text>
      </Tappable>
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { position: 'absolute', bottom: 48, left: 16, right: 16, alignItems: 'center', zIndex: 1000 },
  card: {
    flexDirection: 'row', alignItems: 'center', gap: 8, maxWidth: 460,
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1,
    backgroundColor: 'rgba(0,0,0,0.75)',
  },
});
