import { View, StyleSheet } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { CameraCalibOverlays } from '@/components/CameraCalib';

export function Screen({ children, bleed }: { children: React.ReactNode; bleed?: boolean }) {
  const { c } = useTheme();
  const insets = useSafeAreaInsets();
  return (
    <View
      style={[styles.root, {
        backgroundColor: c.bg,
        paddingLeft: bleed ? 0 : insets.left, paddingRight: bleed ? 0 : insets.right,
        paddingTop: bleed ? 0 : insets.top, paddingBottom: bleed ? 0 : insets.bottom,
      }]}
    >
      {children}
      <CameraCalibOverlays />
    </View>
  );
}

const styles = StyleSheet.create({
  root: { flex: 1 },
});
