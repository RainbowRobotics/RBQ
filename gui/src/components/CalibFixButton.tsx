import { useState } from 'react';
import { Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { AutoStartModal } from '@/components/control/overlays';
import { connection } from '@/lib/connection';
import type { CalibFix } from '@/lib/commissioning';
import { t } from '@/lib/i18n';

export function CalibFixButton({ fix }: { fix: CalibFix }) {
  const { c, radius } = useTheme();
  const [autoOpen, setAutoOpen] = useState(false);
  const onPress = () => { if (fix === 'autostart') setAutoOpen(true); else connection.sendMotion(fix); };
  return (
    <>
      <Tappable onPress={onPress}
        style={[styles.btn, { borderRadius: radius.sm, backgroundColor: 'rgba(77,156,245,0.12)', borderColor: 'rgba(77,156,245,0.6)' }]}>
        <Icon name={fix === 'autostart' ? 'power' : fix} size={18} color={c.accent2} />
        <Text style={{ color: c.accent2, fontSize: 16, fontWeight: '800' }}>
          {fix === 'sit' ? t('앉기') : fix === 'stand' ? t('서기') : t('자동 기동')}
        </Text>
      </Tappable>
      {autoOpen && <AutoStartModal onClose={() => setAutoOpen(false)} />}
    </>
  );
}

const styles = StyleSheet.create({
  btn: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, height: 42, paddingHorizontal: 14, borderWidth: 1 },
});
