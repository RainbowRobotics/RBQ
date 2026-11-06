import { useState } from 'react';
import { View, Text, TextInput, Pressable, Modal as RNModal, StyleSheet } from 'react-native';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import { useTheme } from '@/theme';
import { inputVFix } from '@/components/ui/controls';
import { resolveAccessLevel } from '@/lib/accessLevel';
import { useSettings } from '@/store/settings';
import { t } from '@/lib/i18n';

export function AccessPrompt({ onClose }: { onClose: () => void }) {
  const { c, radius, fonts } = useTheme();
  const [pw, setPw] = useState('');
  const [err, setErr] = useState(false);
  const submit = () => {
    const lv = resolveAccessLevel(pw);
    if (!lv) { setErr(true); setPw(''); return; }
    useSettings.getState().setAccessLevel(lv);
    onClose();
  };
  return (
    <RNModal supportedOrientations={MODAL_ORIENTATIONS} transparent animationType="fade" visible onRequestClose={onClose}>
      <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', backgroundColor: 'rgba(0,0,0,0.45)' }}>
        <Pressable style={StyleSheet.absoluteFill} onPress={onClose} />
      <View style={{ width: 300, borderRadius: 14, padding: 18, backgroundColor: c.panel, borderWidth: 1, borderColor: c.line }}>
        <Text style={{ color: c.text, fontSize: 14, fontWeight: '700', marginBottom: 10 }}>{t('개발자 모드 암호')}</Text>
        <TextInput value={pw} onChangeText={(v) => { setPw(v); setErr(false); }} onSubmitEditing={submit}
          secureTextEntry autoFocus autoCapitalize="none" placeholder={t('암호 입력')} placeholderTextColor={c.dim}
          style={[{ height: 40, borderRadius: radius.sm, borderWidth: 1, borderColor: err ? c.redbright : c.line, paddingHorizontal: 12, color: c.text, fontSize: 13, fontFamily: fonts.mono, backgroundColor: c.bg }, inputVFix]} />
        {err && <Text style={{ color: c.redbright, fontSize: 11, marginTop: 6 }}>{t('암호가 올바르지 않습니다')}</Text>}
        <View style={{ flexDirection: 'row', gap: 10, marginTop: 14 }}>
          <Pressable onPress={onClose}
            style={{ flex: 1, height: 38, borderRadius: radius.sm, alignItems: 'center', justifyContent: 'center', borderWidth: 1, borderColor: c.line }}>
            <Text style={{ color: c.text, fontSize: 13 }}>{t('취소')}</Text>
          </Pressable>
          <Pressable onPress={submit}
            style={{ flex: 1, height: 38, borderRadius: radius.sm, alignItems: 'center', justifyContent: 'center', backgroundColor: c.accent }}>
            <Text style={{ color: c.onAccent, fontSize: 13, fontWeight: '700' }}>{t('확인')}</Text>
          </Pressable>
        </View>
      </View>
      </View>
    </RNModal>
  );
}
