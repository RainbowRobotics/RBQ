import { useEffect, useState } from 'react';
import { Text, StyleSheet, ScrollView, Pressable } from 'react-native';
import Animated, { useSharedValue, useAnimatedStyle, withTiming, interpolate, Easing } from 'react-native-reanimated';
import { useTheme } from '@/theme';
import { useSettings } from '@/store/settings';
import { Icon, type IconName } from '@/components/Icon';

export const FAB = 44;
const OPEN_MS = 240;
const PAD = 8;

export function MorphCard({ side, x, top, width, maxHeight, icon, label, overVideo, children }: {
  side: 'left' | 'right';
  x: number;
  top: number;
  width: number;
  maxHeight: number;
  icon: IconName;
  label: string;
  overVideo: boolean;
  children: React.ReactNode;
}) {
  const { c, radius } = useTheme();
  const open = useSettings((s) => s.cardsOpen[side]);
  const setOpen = (f: (v: boolean) => boolean) => useSettings.getState().setCardOpen(side, f(useSettings.getState().cardsOpen[side]));
  const p = useSharedValue(open ? 1 : 0);
  useEffect(() => { p.value = withTiming(open ? 1 : 0, { duration: OPEN_MS, easing: Easing.inOut(Easing.cubic) }); }, [open, p]);

  const [contentH, setContentH] = useState(0);
  const [viewH, setViewH] = useState(0);
  const [atEnd, setAtEnd] = useState(false);
  const HINT = 14;
  const overflow = contentH > maxHeight - FAB - PAD;
  const bodyMax = Math.max(60, maxHeight - FAB - PAD - (overflow ? HINT : 0));
  const openH = FAB + Math.min(contentH, bodyMax) + (overflow ? HINT : 0) + PAD;

  const box = useAnimatedStyle(() => ({
    width: interpolate(p.value, [0, 1], [FAB, width]),
    height: interpolate(p.value, [0, 1], [FAB, openH]),
    borderRadius: interpolate(p.value, [0, 1], [FAB / 2, radius.lg]),
  }));
  const iconStyle = useAnimatedStyle(() => ({ opacity: interpolate(p.value, [0, 0.45], [1, 0], 'clamp') }));
  const bodyStyle = useAnimatedStyle(() => ({ opacity: interpolate(p.value, [0.5, 1], [0, 1], 'clamp') }));

  const edge = side === 'left' ? { left: x } : { right: x };
  const align = side === 'left' ? { left: 0 } : { right: 0 };
  return (
    <Animated.View style={[styles.box, edge, { top, backgroundColor: overVideo ? c.glass : c.panel, borderColor: overVideo ? c.glassLine : c.line }, box]}>
      <Pressable onPress={() => setOpen((v) => !v)} accessibilityLabel={label} style={[styles.head, { width }, align]}>
        <Animated.View style={[styles.iconCell, align, iconStyle]} pointerEvents="none">
          <Icon name={icon} size={20} color={c.text} />
        </Animated.View>
        <Animated.View style={[styles.foldCell, bodyStyle]} pointerEvents="none">
          <Icon name="caret" size={14} color={c.dim} />
        </Animated.View>
      </Pressable>
      <Animated.View style={[styles.body, { width, top: FAB }, align, bodyStyle]} pointerEvents={open ? 'auto' : 'none'}>
        <ScrollView style={{ maxHeight: bodyMax }} showsVerticalScrollIndicator={false} contentContainerStyle={{ gap: 6, paddingBottom: 2 }}
          onContentSizeChange={(_w, h) => setContentH(h)}
          onLayout={(e) => setViewH(e.nativeEvent.layout.height)}
          scrollEventThrottle={16}
          onScroll={(e) => {
            const { contentOffset, layoutMeasurement, contentSize } = e.nativeEvent;
            setAtEnd(contentOffset.y + layoutMeasurement.height >= contentSize.height - 4);
          }}>
          {children}
        </ScrollView>
        {contentH > viewH + 4 && !atEnd && (
          <Text style={{ color: c.dim, fontSize: 10, textAlign: 'center', marginTop: 1 }}>▾</Text>
        )}
      </Animated.View>
    </Animated.View>
  );
}

const styles = StyleSheet.create({
  box: { position: 'absolute', borderWidth: 1, overflow: 'hidden', zIndex: 8 },
  head: { position: 'absolute', top: 0, height: FAB },
  iconCell: { position: 'absolute', top: 0, width: FAB - 2, height: FAB - 2, alignItems: 'center', justifyContent: 'center' },
  foldCell: { position: 'absolute', top: 0, left: 0, right: 0, height: FAB, alignItems: 'center', justifyContent: 'center', transform: [{ rotate: '180deg' }] },
  body: { position: 'absolute', paddingHorizontal: PAD },
});
