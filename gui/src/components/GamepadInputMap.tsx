import { useEffect, useState } from 'react';
import { View, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { GamepadDiagram } from '@/components/GamepadDiagram';
import { useGamepad } from '@/store/gamepad';
import { getLiveAxes, getPressedKeys } from '@/lib/gamepad/manager';
import { matchProfile, toDiagramKey } from '@/lib/gamepad/profiles';

const POLL_MS = 100;

export function GamepadInputMap({ inline, scale }: { inline?: boolean; scale?: number } = {}) {
  const { c, radius } = useTheme();
  const dev = useGamepad((s) => s.devices[0] ?? null);
  const [axes, setAxes] = useState({ lx: 0, ly: 0, rx: 0, ry: 0, hatX: 0, hatY: 0 });
  const [pressed, setPressed] = useState<number[]>([]);

  useEffect(() => {
    const t = setInterval(() => { setAxes(getLiveAxes()); setPressed(getPressedKeys()); }, POLL_MS);
    return () => clearInterval(t);
  }, []);

  if (!dev) return null;
  const profile = matchProfile(dev);
  const hasHat = profile.axes.hatX != null && dev.axes.some((a) => a.axis === profile.axes.hatX);
  const hatKeys = hasHat ? [19, 20, 21, 22] : [];
  let keys = [...(profile.presentKeys ?? dev.keys ?? [])];
  let shown = pressed;
  keys = keys.map((k) => toDiagramKey(dev, profile, k)).filter((k) => k >= 0);
  shown = pressed.map((k) => toDiagramKey(dev, profile, k)).filter((k) => k >= 0);
  keys = [...keys, ...hatKeys];

  return (
    <View style={[inline ? styles.inline : styles.strip, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]} pointerEvents="none">
      <GamepadDiagram compact scale={scale} deviceName={dev.name} keys={keys} pressed={shown} axes={axes} />
    </View>
  );
}

const styles = StyleSheet.create({
  strip: {
    position: 'absolute', bottom: 10, alignSelf: 'center',
    paddingHorizontal: 14, paddingVertical: 6, borderWidth: 1,
  },
  inline: { alignSelf: 'center', paddingHorizontal: 10, paddingVertical: 6, borderWidth: 1 },
});
