import { useEffect, useState } from 'react';
import { View, Text, TextInput, StyleSheet, ScrollView, ActivityIndicator, useWindowDimensions } from 'react-native';
import { LinearGradient } from 'expo-linear-gradient';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { useWifi } from '@/store/wifi';
import type { WifiNetwork } from '@/lib/desktopBridge';
import { t } from '@/lib/i18n';
import { inputVFix } from '@/components/ui/controls';

export function friendlyWifiError(raw: string): string {
  if (/psk|802-11-wireless-security/i.test(raw)) return t('비밀번호가 올바르지 않습니다 (WPA 는 8자 이상)');
  if (/secret|passphrase|password|key.*(invalid|required)/i.test(raw) || /비밀|암호/.test(raw)) {
    return t('비밀번호가 올바르지 않습니다');
  }
  if (/no network with ssid|not found|없습니다/i.test(raw)) return t('네트워크를 찾을 수 없습니다');
  if (/timeout|timed out/i.test(raw)) return t('연결 시간이 초과되었습니다');
  return raw;
}

function Bars({ strength, color, dim }: { strength: number; color: string; dim: string }) {
  const lit = Math.max(1, Math.ceil((strength / 100) * 4));
  return (
    <View style={{ flexDirection: 'row', alignItems: 'flex-end', gap: 2, height: 14 }}>
      {[5, 8, 11, 14].map((h, i) => (
        <View key={h} style={{ width: 3.5, height: h, borderRadius: 1.5, backgroundColor: i < lit ? color : dim }} />
      ))}
    </View>
  );
}

export function WifiPicker({ onClose }: { onClose: () => void }) {
  const { c, fonts } = useTheme();
  const { height: sh } = useWindowDimensions();
  const { networks, scanning, connecting, error, scan, connect } = useWifi();
  const [sel, setSel] = useState<WifiNetwork | null>(null);
  const [pw, setPw] = useState('');
  const [showPw, setShowPw] = useState(false);
  const [hint, setHint] = useState<string | null>(null);
  const [busySsid, setBusySsid] = useState<string | null>(null);

  useEffect(() => { scan(); }, []); // eslint-disable-line react-hooks/exhaustive-deps

  const onPick = async (n: WifiNetwork) => {
    if (connecting) return;
    if (n.secured && !n.saved) { setSel(n); setPw(''); return; }
    setBusySsid(n.ssid);
    const ok = await connect(n.ssid);
    setBusySsid(null);
    if (ok) { onClose(); return; }
    if (n.secured) { setSel(n); setPw(''); }
  };
  const doConnect = async () => {
    if (!sel || connecting) return;
    if (!pw) { setHint(t('비밀번호를 입력하세요')); return; }
    setHint(null);
    if (await connect(sel.ssid, pw)) onClose();
  };

  return (
    <Modal onClose={onClose}>
      <LinearGradient colors={[c.modalA, c.modalB]} style={[styles.box, { borderColor: c.line, maxHeight: Math.min(440, sh - 40) }]}>
        <View style={[styles.head, { borderBottomColor: c.line2 }]}>
          <Text style={{ fontSize: 14, fontWeight: '700', color: c.text }}>{t('네트워크 선택')}</Text>
          <View style={{ flexDirection: 'row', gap: 8 }}>
            <Tappable onPress={() => { if (!scanning) scan(); }} style={[styles.hbtn, { backgroundColor: c.elev, borderColor: c.line, opacity: scanning ? 0.6 : 1 }]}>
              {scanning ? <ActivityIndicator size="small" color={c.accent2} /> : <Icon name="wifi" size={14} color={c.muted} />}
              <Text style={{ color: c.muted, fontSize: 11 }}>{scanning ? t('검색 중…') : t('새로고침')}</Text>
            </Tappable>
            <Tappable onPress={onClose} style={[styles.hbtn, { backgroundColor: c.elev, borderColor: c.line }]}>
              <Icon name="x" size={15} color={c.muted} />
            </Tappable>
          </View>
        </View>

        {sel ? (
          <View style={{ padding: 18, gap: 12 }}>
            <View style={{ flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 8 }}>
              <Text style={{ color: c.text, fontSize: 13, flexShrink: 1 }} numberOfLines={1}>
                <Text style={{ fontWeight: '700' }}>{sel.ssid}</Text> {t('비밀번호')}
              </Text>
              <Tappable onPress={() => setShowPw((v) => !v)} style={[styles.hbtn, { backgroundColor: c.elev, borderColor: c.line }]}>
                <Text style={{ color: c.muted, fontSize: 11 }}>{showPw ? t('숨기기') : t('보기')}</Text>
              </Tappable>
            </View>
            <TextInput
              value={pw}
              onChangeText={(v) => { setPw(v); if (hint) setHint(null); }}
              onSubmitEditing={doConnect}
              secureTextEntry={!showPw}
              autoFocus
              autoCapitalize="none"
              placeholder={t('WiFi 비밀번호')}
              placeholderTextColor={c.dim}
              style={[styles.input, { backgroundColor: c.bg, borderColor: c.line, color: c.text, fontFamily: fonts.mono }, inputVFix]}
            />
            {hint ? <Text style={{ color: c.amber, fontSize: 11 }}>{hint}</Text> : null}
            {error ? <Text style={{ color: c.amber, fontSize: 11 }}>{friendlyWifiError(error)}</Text> : null}
            <View style={{ flexDirection: 'row', gap: 10, marginTop: 4 }}>
              <Tappable onPress={() => setSel(null)} style={[styles.abtn, { backgroundColor: c.elev, borderColor: c.line }]}>
                <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{t('뒤로')}</Text>
              </Tappable>
              <Tappable onPress={doConnect} style={[styles.abtn, { backgroundColor: c.accent, borderColor: 'transparent' }]}>
                {connecting ? (
                  <>
                    <ActivityIndicator size="small" color={c.onAccent} />
                    <Text style={{ color: c.onAccent, fontSize: 13, fontWeight: '600', marginLeft: 8 }}>{t('연결 중…')}</Text>
                  </>
                ) : (
                  <Text style={{ color: c.onAccent, fontSize: 13, fontWeight: '600' }}>{t('연결')}</Text>
                )}
              </Tappable>
            </View>
          </View>
        ) : (
          <ScrollView style={{ flexShrink: 1, backgroundColor: c.modalA }} contentContainerStyle={{ padding: 12 }}>
            {error ? <Text style={{ color: c.amber, fontSize: 11, marginBottom: 12 }}>{friendlyWifiError(error)}</Text> : null}
            {scanning && networks.length === 0 ? (
              <View style={{ alignItems: 'center', paddingVertical: 24, gap: 8 }}>
                <ActivityIndicator color={c.accent2} />
                <Text style={{ color: c.dim, fontSize: 11 }}>{t('스캔 중…')}</Text>
              </View>
            ) : networks.length === 0 ? (
              <Text style={{ color: c.dim, fontSize: 12, textAlign: 'center', paddingVertical: 20 }}>{t('검색된 네트워크가 없습니다')}</Text>
            ) : (
              networks.map((n) => (
                <Tappable key={n.ssid} onPress={() => onPick(n)} style={[styles.row, { borderColor: c.line2, opacity: busySsid && busySsid !== n.ssid ? 0.45 : 1 }]}>
                  {n.secured ? <Text style={{ fontSize: 12 }}>🔒</Text> : <Icon name="wifi" size={14} color={n.active ? c.green : c.muted} />}
                  <Text style={{ color: n.active ? c.green : c.text, fontSize: 13, flex: 1 }} numberOfLines={1}>
                    {n.ssid}{n.active ? t('  ·  연결됨') : ''}
                  </Text>
                  {busySsid === n.ssid ? (
                    <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
                      <ActivityIndicator size="small" color={c.accent2} />
                      <Text style={{ color: c.accent2, fontSize: 11, fontWeight: '700' }}>{t('연결 중…')}</Text>
                    </View>
                  ) : <Bars strength={n.signal} color={n.active ? c.green : c.accent2} dim={c.elev2} />}
                </Tappable>
              ))
            )}
          </ScrollView>
        )}
      </LinearGradient>
    </Modal>
  );
}

const styles = StyleSheet.create({
  box: { width: 380, maxHeight: 440, borderWidth: 1, borderRadius: 18, overflow: 'hidden' },
  head: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', padding: 14, borderBottomWidth: 1 },
  hbtn: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 30, paddingHorizontal: 10, borderRadius: 8, borderWidth: 1 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 11, paddingVertical: 11, paddingHorizontal: 8, borderBottomWidth: 1 },
  input: { height: 40, borderRadius: 9, borderWidth: 1, paddingHorizontal: 12, fontSize: 13 },
  abtn: { flex: 1, height: 42, borderRadius: 11, borderWidth: 1, flexDirection: 'row', alignItems: 'center', justifyContent: 'center' },
});
