import { useState } from 'react';
import { View, Text, StyleSheet, TextInput, useWindowDimensions } from 'react-native';
import { useRouter } from 'expo-router';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Popover } from '@/components/ui/overlays';
import { Toggle } from '@/components/ui/controls';
import { AccessPrompt } from '@/components/ui/AccessPrompt';
import { useAccount } from '@/store/account';
import { useSettings } from '@/store/settings';
import { logoutAccount } from '@/lib/accountSession';
import { t } from '@/lib/i18n';
import { cloudConfigured } from '@/lib/logUploadCommon';
import { goTop } from '@/lib/nav';

export function AccountPopover({ onClose }: { onClose: () => void }) {
  const { c, fonts, radius } = useTheme();
  const { height: winH } = useWindowDimensions();
  const router = useRouter();
  const acct = useAccount((s) => s.account);
  const pin = useAccount((s) => s.pin);
  const robots = useAccount((s) => s.robots.filter((r) => r.robotSerial).length);
  const level = useSettings((s) => s.accessLevel);
  const setLevel = useSettings((s) => s.setAccessLevel);
  const [pw, setPw] = useState(false);
  const nick = useSettings((s) => s.nickname);
  const setNick = useSettings((s) => s.setNickname);
  const [editing, setEditing] = useState(false);
  const [draft, setDraft] = useState(nick);
  const item = (icon: 'tag' | 'x', label: string, onPress: () => void, danger = false) => (
    <Tappable onPress={() => { onClose(); onPress(); }} style={[styles.item, { borderRadius: radius.sm }]}>
      <Icon name={icon} size={16} color={danger ? c.redTx : c.accent2} />
      <Text style={{ color: danger ? c.redTx : c.text, fontSize: 13.5, fontWeight: '600' }}>{label}</Text>
    </Tappable>
  );
  return (
    <>
      <Popover onClose={onClose} style={{ right: 12, top: 60, width: 300, maxHeight: winH - 72 }}>
        <View style={styles.who}>
          <View style={[styles.avatar, { backgroundColor: acct ? c.accent : c.elev }]}>
            {acct ? <Text style={{ color: c.onAccent, fontSize: 16, fontWeight: '800' }}>{(nick || acct.accountName).slice(0, 1).toUpperCase()}</Text>
                  : <Icon name="user" size={18} color={c.muted} />}
          </View>
          <View style={{ flex: 1 }}>
            {acct && editing ? (
              <TextInput value={draft} onChangeText={setDraft} autoFocus placeholder={acct.accountName} placeholderTextColor={c.dim}
                onBlur={() => { setNick(draft); setEditing(false); }} onSubmitEditing={() => { setNick(draft); setEditing(false); }}
                style={{ color: c.text, fontSize: 15, fontWeight: '800', borderBottomWidth: 1, borderBottomColor: c.accent, paddingVertical: 2 }} />
            ) : (
              <Tappable onPress={acct ? () => { setDraft(nick); setEditing(true); } : undefined} style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
                <Text style={{ color: c.text, fontSize: 15, fontWeight: '800' }}>{acct ? (nick || acct.accountName) : t('게스트')}</Text>
                {acct && <Icon name="sliders" size={12} color={c.accent2} />}
              </Tappable>
            )}
            <Text style={{ color: c.muted, fontSize: 11.5, fontFamily: acct ? fonts.mono : undefined }}>
              {acct ? `${acct.accountName} · ${t('레벨')} ${acct.level}` : t('로봇망 직결만 씁니다')}
            </Text>
          </View>
        </View>
        <View style={[styles.sep, { backgroundColor: c.line2 }]} />
        {acct ? (
          <>
            <View style={styles.kv}><Text style={styles.k}>{t('배정 로봇')}</Text><Text style={[styles.v, { color: c.text }]}>{robots}</Text></View>
            <View style={styles.kv}><Text style={styles.k}>{t('접근 코드')}</Text><Text style={[styles.v, { color: pin ? c.greenTx : c.amberTx }]}>{pin ? t('저장됨') : t('없음')}</Text></View>
            {!pin && item('tag', t('코드 입력'), () => router.push('/pin'))}
            <View style={[styles.sep, { backgroundColor: c.line2 }]} />
            {item('x', t('로그아웃'), () => { logoutAccount(); goTop('/pin'); }, true)}
          </>
        ) : (
          <>
            {cloudConfigured && (<>
            {item('tag', t('접근 코드로 로그인'), () => router.push('/pin'))}
            <Text style={{ color: c.dim, fontSize: 10.5, paddingHorizontal: 10, lineHeight: 14 }}>{t('대시보드에서 발급한 4자리 — 처음 한 번만 넣으면 됩니다')}</Text>
            <View style={[styles.sep, { backgroundColor: c.line2 }]} />
            </>)}
            <View style={styles.row}>
              <View style={{ flex: 1 }}>
                <Text style={{ color: c.text, fontSize: 13.5, fontWeight: '600' }}>{t('개발자 모드')}</Text>
                <Text style={{ color: c.dim, fontSize: 10.5 }}>{t('켤 때 암호 필요')}</Text>
              </View>
              <Toggle value={level > 1} onChange={(v) => (v ? setPw(true) : setLevel(1))} />
            </View>
          </>
        )}
      </Popover>
      {pw && <AccessPrompt onClose={() => setPw(false)} />}
    </>
  );
}
const styles = StyleSheet.create({
  who: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingBottom: 10 },
  avatar: { width: 36, height: 36, borderRadius: 18, alignItems: 'center', justifyContent: 'center' },
  sep: { height: 1, marginVertical: 6 },
  item: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingHorizontal: 10, paddingVertical: 10 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingHorizontal: 10, paddingVertical: 6 },
  kv: { flexDirection: 'row', justifyContent: 'space-between', paddingHorizontal: 10, paddingVertical: 7 },
  k: { color: '#9098A3', fontSize: 12.5 },
  v: { fontSize: 12.5, fontWeight: '700', fontFamily: 'monospace' },
});
