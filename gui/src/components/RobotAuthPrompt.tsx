import { useState } from 'react';
import { View, Text, TextInput, Pressable, Modal as RNModal } from 'react-native';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import { inputVFix } from '@/components/ui/controls';
import { useTheme } from '@/theme';
import { useRobot } from '@/store/robot';
import { useRobots, passwordKeyFor } from '@/store/robots';
import { authFailure } from '@/lib/auth';
import { reconnectCurrent } from '@/lib/connectNow';
import { t } from '@/lib/i18n';

export function RobotAuthPrompt() {
  const open = useRobot((s) => s.authNeeded);
  if (!open) return null;
  return <Prompt />;
}

function Prompt() {
  const { c, radius, fonts } = useTheme();
  const [pw, setPw] = useState('');
  const close = () => useRobot.getState().setAuthNeeded(false);
  const submit = () => {
    if (!pw) return;
    const { target, probe } = authFailure();
    const key = passwordKeyFor(target, probe);
    if (key) useRobots.getState().setPassword(key, pw);
    useRobot.getState().setConnError(null);
    close();
    if (probe) useRobot.getState().bumpAuthEpoch();
    else reconnectCurrent();
  };
  return (
    <RNModal supportedOrientations={MODAL_ORIENTATIONS} transparent animationType="fade" visible onRequestClose={close}>
      <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', backgroundColor: 'rgba(0,0,0,0.45)' }}>
        <View style={{ width: 320, borderRadius: 14, padding: 18, backgroundColor: c.panel, borderWidth: 1, borderColor: c.line }}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '700', marginBottom: 6 }}>{t('로봇 비밀번호')}</Text>
          <Text style={{ color: c.dim, fontSize: 12, marginBottom: 10 }}>{t('로봇이 비밀번호를 요구합니다. 한 번 입력하면 이 기기에 저장됩니다.')}</Text>
          <TextInput value={pw} onChangeText={setPw} onSubmitEditing={submit}
            secureTextEntry autoFocus autoCapitalize="none" autoCorrect={false} placeholder={t('비밀번호 입력')} placeholderTextColor={c.dim}
            style={[{ height: 40, borderRadius: radius.sm, borderWidth: 1, borderColor: c.line, paddingHorizontal: 12, color: c.text, fontSize: 13, fontFamily: fonts.mono, backgroundColor: c.bg }, inputVFix]} />
          <View style={{ flexDirection: 'row', gap: 10, marginTop: 14 }}>
            <Pressable onPress={close}
              style={{ flex: 1, height: 38, borderRadius: radius.sm, alignItems: 'center', justifyContent: 'center', borderWidth: 1, borderColor: c.line }}>
              <Text style={{ color: c.text, fontSize: 13 }}>{t('취소')}</Text>
            </Pressable>
            <Pressable onPress={submit} disabled={!pw}
              style={{ flex: 1, height: 38, borderRadius: radius.sm, alignItems: 'center', justifyContent: 'center', backgroundColor: c.accent, opacity: pw ? 1 : 0.5 }}>
              <Text style={{ color: c.onAccent, fontSize: 13, fontWeight: '700' }}>{t('연결')}</Text>
            </Pressable>
          </View>
        </View>
      </View>
    </RNModal>
  );
}
