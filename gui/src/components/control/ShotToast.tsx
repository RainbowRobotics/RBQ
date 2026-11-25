import { useEffect, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useRouter } from 'expo-router';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useMedia } from '@/lib/media';
import { t } from '@/lib/i18n';

const SHOW_MS = 3200;
const TONE_OK = '#00C853';
const TONE_WARN = '#FBBA16';
const TONE_BUSY = '#7CB8FF';

export function ShotToast() {
  const { radius } = useTheme();
  const router = useRouter();
  const shot = useMedia((s) => s.lastShot);
  const shooting = useMedia((s) => s.shooting || s.videoBusy);
  const [visibleAt, setVisibleAt] = useState(0);
  useEffect(() => {
    if (!shot) return;
    setVisibleAt(shot.at);
    const id = setTimeout(() => setVisibleAt(0), SHOW_MS);
    return () => clearTimeout(id);
  }, [shot]);
  if (!shooting && (!shot || visibleAt !== shot.at)) return null;
  const ok = !shooting && !!shot?.ok;
  const tone = shooting ? TONE_BUSY : ok ? TONE_OK : TONE_WARN;
  return (
    <View pointerEvents="box-none" style={styles.wrap}>
      <Tappable onPress={() => setVisibleAt(0)} style={[styles.card, { borderColor: tone, borderRadius: radius.md }]}>
        <Icon name={shooting ? 'maximize' : ok ? 'save' : 'warn'} size={14} color={tone} />
        <Text style={{ color: tone, fontSize: 11.5, fontWeight: '700' }} numberOfLines={2}>
          {shooting ? t('처리 중…') : shot?.msg}
        </Text>
        {ok && shot?.sec && (
          <Tappable onPress={() => { setVisibleAt(0); router.push(`/media?sec=${shot.sec}`); }} style={styles.link}>
            <Text style={{ color: '#fff', fontSize: 11.5, fontWeight: '700' }}>{t('보기')} ›</Text>
          </Tappable>
        )}
      </Tappable>
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { position: 'absolute', bottom: 116, left: 0, right: 0, alignItems: 'center', zIndex: 31 },
  card: {
    flexDirection: 'row', alignItems: 'center', gap: 8, maxWidth: 460,
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1,
    backgroundColor: 'rgba(0,0,0,0.55)',
  },
  link: { marginLeft: 4, paddingHorizontal: 9, paddingVertical: 3, borderRadius: 999, backgroundColor: 'rgba(255,255,255,0.14)' },
});
