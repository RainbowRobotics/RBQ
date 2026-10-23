import { useState } from 'react';
import { Platform, Pressable, StyleSheet, View, Modal as RNModal, type ModalProps, type StyleProp, type ViewStyle , ScrollView , Text } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import Animated, { FadeIn, FadeOut } from 'react-native-reanimated';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
import { LinearGradient } from 'expo-linear-gradient';
import { useTheme } from '@/theme';
import { unfoldDown, foldUp, unfoldUp, foldDown, popIn, popOut } from '@/components/anim';

export const MODAL_ORIENTATIONS: ModalProps['supportedOrientations'] = ['landscape'];

export function Popover({
  onClose, style, children, dim = false, backdropZ, scroll = true, portal = true,
}: {
  onClose: () => void;
  style?: StyleProp<ViewStyle>;
  children: React.ReactNode;
  dim?: boolean;
  portal?: boolean;
  backdropZ?: number;
  scroll?: boolean;
}) {
  const { c } = useTheme();
  const flat = StyleSheet.flatten(style) ?? {};
  const fromBottom = flat.bottom != null && flat.top == null;
  const origin = `${fromBottom ? 'bottom' : 'top'} ${flat.right != null && flat.left == null ? 'right' : 'left'}`;
  const [contentH, setContentH] = useState(0);
  const [viewH, setViewH] = useState(0);
  const [atEnd, setAtEnd] = useState(false);
  const body = (
    <>
      <Animated.View
        entering={dim ? FadeIn.duration(120) : undefined}
        exiting={dim ? FadeOut.duration(90) : undefined}
        style={[StyleSheet.absoluteFill, dim && { backgroundColor: c.scrim }, { zIndex: backdropZ ?? 30 }]}
      >
        <Pressable style={StyleSheet.absoluteFill} onPress={onClose} />
      </Animated.View>
      <Animated.View
        entering={fromBottom ? unfoldUp : unfoldDown}
        exiting={fromBottom ? foldDown : foldUp}
        style={[styles.pop, { transformOrigin: origin } as ViewStyle, style]}
      >
        <LinearGradient colors={[c.sheetA, c.sheetB]} style={[styles.popInner, typeof flat.maxHeight === 'number' ? { maxHeight: flat.maxHeight } : null, { borderColor: c.line }]}>
          {scroll ? (
            <ScrollView showsVerticalScrollIndicator={false} style={{ flexShrink: 1 }} contentContainerStyle={{ flexGrow: 1 }}
              onContentSizeChange={(_w, h) => setContentH(h)}
              onLayout={(e) => setViewH(e.nativeEvent.layout.height)}
              onScroll={(e) => setAtEnd(e.nativeEvent.contentOffset.y + e.nativeEvent.layoutMeasurement.height >= e.nativeEvent.contentSize.height - 4)}
              scrollEventThrottle={16}>
              {children}
            </ScrollView>
          ) : children}
          {scroll && contentH > viewH + 4 && !atEnd && (
            <View pointerEvents="none" style={[styles.moreHint, { backgroundColor: c.sheetB }]}>
              <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700' }}>▾</Text>
            </View>
          )}
        </LinearGradient>
      </Animated.View>
    </>
  );
  if (portal && Platform.OS === 'android') {
    return (
      <RNModal transparent visible animationType="none" onRequestClose={onClose} supportedOrientations={MODAL_ORIENTATIONS}>
        <GestureHandlerRootView style={StyleSheet.absoluteFill} pointerEvents="box-none">{body}</GestureHandlerRootView>
      </RNModal>
    );
  }
  return body;
}

export function Modal({
  onClose, children, dismissable = true, fit = false,
}: {
  onClose: () => void;
  children: React.ReactNode;
  dismissable?: boolean;
  fit?: boolean;
}) {
  const insets = useSafeAreaInsets();
  return (
    <RNModal transparent visible animationType="none" onRequestClose={onClose} supportedOrientations={MODAL_ORIENTATIONS}>
    <View style={[styles.scrimWrap, { paddingLeft: insets.left, paddingRight: insets.right, paddingTop: insets.top, paddingBottom: insets.bottom }]} pointerEvents="box-none">
      <Animated.View entering={FadeIn.duration(130)} exiting={FadeOut.duration(110)} style={styles.scrim}>
        <Pressable style={StyleSheet.absoluteFill} onPress={dismissable ? onClose : undefined} />
      </Animated.View>
      {fit ? (
        <View style={[StyleSheet.absoluteFill, styles.modalFit]} pointerEvents="box-none">
          <Pressable style={StyleSheet.absoluteFill} onPress={dismissable ? onClose : undefined} />
          <Animated.View entering={popIn} exiting={popOut} style={styles.fitCard}>
            {children}
          </Animated.View>
        </View>
      ) : (
      <ScrollView style={StyleSheet.absoluteFill} contentContainerStyle={styles.modalScroll}
                  showsVerticalScrollIndicator={false} keyboardShouldPersistTaps="handled">
        <Pressable style={StyleSheet.absoluteFill} onPress={dismissable ? onClose : undefined} />
        <Animated.View entering={popIn} exiting={popOut}>
          {children}
        </Animated.View>
      </ScrollView>
      )}
    </View>
    </RNModal>
  );
}

const styles = StyleSheet.create({
  moreHint: { position: 'absolute', left: 0, right: 0, bottom: 0, height: 16, alignItems: 'center', justifyContent: 'center', opacity: 0.92 },
  pop: { position: 'absolute', zIndex: 40, overflow: 'hidden' },
  popInner: { borderWidth: 1, borderRadius: 14, padding: 14, maxHeight: '100%', flexShrink: 1 },
  scrimWrap: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, alignItems: 'center', justifyContent: 'center', zIndex: 50 },
  modalScroll: { flexGrow: 1, alignItems: 'center', justifyContent: 'center', padding: 12 },
  modalFit: { alignItems: 'center', justifyContent: 'center', padding: 12 },
  fitCard: { flex: 1, minHeight: 0, maxHeight: 760 },
  scrim: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, backgroundColor: 'rgba(4,5,9,0.66)' },
});
