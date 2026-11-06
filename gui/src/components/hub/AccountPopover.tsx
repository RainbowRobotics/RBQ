import { useState } from 'react';
import { View, Text, StyleSheet, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Popover } from '@/components/ui/overlays';
import { Toggle } from '@/components/ui/controls';
import { AccessPrompt } from '@/components/ui/AccessPrompt';
import { useSettings } from '@/store/settings';
import { t } from '@/lib/i18n';
import { accountGuestItems, remote, type AccountMenuProps } from '@/modules/registry';

const notSignedIn = () => false;

type Props = AccountMenuProps;

export function AccountPopover(props: Props) {
  const signedIn = (remote.account?.useSignedIn ?? notSignedIn)();
  if (signedIn && remote.account) return <remote.account.Popover {...props} />;
  return <GuestPopover {...props} />;
}

function GuestPopover({ onClose, beforeNavigate }: Props) {
  const { c } = useTheme();
  const { height: winH } = useWindowDimensions();
  const level = useSettings((s) => s.accessLevel);
  const setLevel = useSettings((s) => s.setAccessLevel);
  const [pw, setPw] = useState(false);
  return (
    <>
      <Popover onClose={onClose} style={{ right: 12, top: 60, width: 300, maxHeight: winH - 72 }}>
        <View style={styles.who}>
          <View style={[styles.avatar, { backgroundColor: c.elev }]}>
            <Icon name="user" size={18} color={c.muted} />
          </View>
          <View style={{ flex: 1 }}>
            <Text style={{ color: c.text, fontSize: 15, fontWeight: '800' }}>{t('게스트')}</Text>
            <Text style={{ color: c.muted, fontSize: 11.5 }}>{t('로봇망 직결만 씁니다')}</Text>
          </View>
        </View>
        <View style={[styles.sep, { backgroundColor: c.line2 }]} />
        {accountGuestItems.map((Item, i) => <Item key={i} onClose={onClose} beforeNavigate={beforeNavigate} />)}
        <View style={styles.row}>
          <View style={{ flex: 1 }}>
            <Text style={{ color: c.text, fontSize: 13.5, fontWeight: '600' }}>{t('개발자 모드')}</Text>
            <Text style={{ color: c.dim, fontSize: 10.5 }}>{t('켤 때 암호 필요')}</Text>
          </View>
          <Toggle value={level > 1} onChange={(v) => (v ? setPw(true) : setLevel(1))} />
        </View>
      </Popover>
      {pw && <AccessPrompt onClose={() => setPw(false)} />}
    </>
  );
}
export const accountStyles = StyleSheet.create({
  who: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingBottom: 10 },
  avatar: { width: 36, height: 36, borderRadius: 18, alignItems: 'center', justifyContent: 'center' },
  sep: { height: 1, marginVertical: 6 },
  item: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingHorizontal: 10, paddingVertical: 10 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingHorizontal: 10, paddingVertical: 6 },
  kv: { flexDirection: 'row', justifyContent: 'space-between', paddingHorizontal: 10, paddingVertical: 7 },
  k: { color: '#9098A3', fontSize: 12.5 },
  v: { fontSize: 12.5, fontWeight: '700', fontFamily: 'monospace' },
});
const styles = accountStyles;
