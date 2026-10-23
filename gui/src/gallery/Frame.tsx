import { useState } from 'react';
import { View, Text, StyleSheet, Platform, Pressable } from 'react-native';
import { useTheme } from '@/theme';
import type { WidgetCase } from './types';

function Code({ src }: { src: string }) {
  const { c, fonts } = useTheme();
  const [open, setOpen] = useState(false);
  const [copied, setCopied] = useState(false);
  const copy = () => {
    navigator?.clipboard?.writeText(src).then(() => { setCopied(true); setTimeout(() => setCopied(false), 1200); }).catch(() => {});
  };
  return (
    <View style={{ marginTop: 10 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 10 }}>
        <Pressable onPress={() => setOpen((v) => !v)} hitSlop={6}>
          <Text style={{ color: c.accent2, fontSize: 11, fontWeight: '600' }}>{open ? '▾ 코드' : '▸ 코드'}</Text>
        </Pressable>
        {open && (
          <Pressable onPress={copy} hitSlop={6}>
            <Text style={{ color: copied ? c.green : c.dim, fontSize: 11 }}>{copied ? '복사됨' : '복사'}</Text>
          </Pressable>
        )}
      </View>
      {open && (
        <View style={{ marginTop: 6, padding: 10, borderRadius: 8, backgroundColor: c.bg, borderWidth: 1, borderColor: c.line2 }}>
          <Text selectable style={{ color: c.text, fontSize: 11, lineHeight: 17, fontFamily: fonts.mono }}>{src}</Text>
        </View>
      )}
    </View>
  );
}

export function Frame({ c: item }: { c: WidgetCase }) {
  const { c, fonts } = useTheme();
  const { Demo } = item;
  return (
    <View style={[styles.case, item.size === 'wide' && styles.wide, item.size === 'full' && styles.full,
      { backgroundColor: c.panel, borderColor: c.line, opacity: Demo ? 1 : 0.72 }]}>
      <Text style={{ color: c.text, fontSize: 13, fontWeight: '700' }}>{item.name}</Text>
      <Text style={{ color: c.dim, fontSize: 10, fontFamily: fonts.mono, marginTop: 2 }}>{item.from}</Text>
      <Text style={{ color: c.muted, fontSize: 11, marginTop: 6, lineHeight: 15 }}>{item.when}</Text>
      <View style={[styles.stage, { backgroundColor: c.bg, borderColor: c.line2 }]}>
        {Demo ? <Demo /> : <Text style={{ color: c.amber, fontSize: 11, lineHeight: 16 }}>⚠ {item.unavailable}</Text>}
      </View>
      {item.code && <Code src={item.code} />}
    </View>
  );
}

const styles = StyleSheet.create({
  case: { width: Platform.OS === 'web' ? 360 : '100%', maxWidth: 480, borderWidth: 1, borderRadius: 12, padding: 12 },
  wide: { width: Platform.OS === 'web' ? 760 : '100%', maxWidth: 880 },
  full: { width: '100%', maxWidth: 1600 },
  stage: { marginTop: 10, borderWidth: 1, borderRadius: 9, padding: 12, minHeight: 72, justifyContent: 'center' },
});

export const demoBtn = { paddingHorizontal: 14, paddingVertical: 8, borderRadius: 9, alignItems: 'center', justifyContent: 'center' } as const;
