import { useEffect, useState } from 'react';
import { openExternal } from '@/lib/openExternal';
import { registerPush, unregisterPush } from '@/lib/push';
import { useAccount } from '@/store/account';
import { View, Text, Platform } from 'react-native';
import { useTheme } from '@/theme';
import { useLang } from '@/store/lang';
import { Toggle, Segmented, Slider } from '@/components/ui/controls';
import { useSettings } from '@/store/settings';
import { warnBeep } from '@/components/SignalStrength';
import { isDesktop, setUiZoom } from '@/lib/desktopBridge';
import { H2, Desc, TRow } from './common';
import { Tappable } from '@/components/anim';
import { useRobot } from '@/store/robot';
import { currentAppVersion } from '@/lib/appSelfUpdate';
import { t } from '@/lib/i18n';
import { cloudConfigured } from '@/lib/logUploadCommon';

const ISSUES_REPO = process.env.EXPO_PUBLIC_ISSUES_REPO || '';

function FullscreenRow() {
  const supported = !isDesktop() && Platform.OS === 'web'
    && typeof document !== 'undefined' && !!document.fullscreenEnabled;
  const [full, setFull] = useState(supported && !!document.fullscreenElement);
  useEffect(() => {
    if (!supported) return;
    const on = () => setFull(!!document.fullscreenElement);
    document.addEventListener('fullscreenchange', on);
    return () => document.removeEventListener('fullscreenchange', on);
  }, [supported]);
  if (!supported) return null;
  return (
    <TRow nm={t('전체화면')} sub={t('브라우저/데스크탑 창 전체화면 (Qt Full Screen Mode)')} right={
      <Toggle value={full} onChange={(v) => {
        if (v) document.documentElement.requestFullscreen().catch(() => {});
        else document.exitFullscreen().catch(() => {});
      }} />
    } />
  );
}

export function AppPanel() {
  const robotIp = useRobot((rs) => rs.ip);
  const openGithubIssue = () => {
    const body = [
      '## 증상', '', '', '## 재현 절차', '1. ', '', '## 환경',
      `- 앱: ${currentAppVersion()} (${Platform.OS})`,
      `- 로봇: ${robotIp || '-'}`,
      '', '_앱 [GitHub 이슈 등록]에서 생성됨_',
    ].join('\n');
    const issueUrl = `https://github.com/${ISSUES_REPO}/issues/new?title=${encodeURIComponent('bug: ')}&body=${encodeURIComponent(body)}`;
    openExternal(`https://github.com/login?return_to=${encodeURIComponent(issueUrl)}`);
  };
  const { name, setTheme, c, fonts } = useTheme();
  const lang = useLang((s) => s.lang);
  const setLang = useLang((s) => s.setLang);
  const st = useSettings();
  const accountPin = useAccount((s2) => s2.pin);
  return (
    <>
      <H2>🎨 {t('앱 설정')}</H2>
      <Desc>{t('앱 자체 환경설정 (로봇 API 아님)')}</Desc>
      <TRow nm={t('테마')} sub={t('화면 색상 모드')} right={
        <View style={{ width: 150 }}>
          <Segmented options={[{ key: 'dark', label: t('다크') }, { key: 'light', label: t('라이트') }]} value={name} onChange={(v) => setTheme(v as any)} />
        </View>
      } />
      <TRow nm={t('언어 / Language')} sub={t('Korean / English (구·신 공유)')} right={
        <View style={{ width: 150 }}>
          <Segmented options={[{ key: 'ko', label: t('한국어') }, { key: 'en', label: 'English' }]} value={lang} onChange={(v) => setLang(v as any)} />
        </View>
      } />
      <TRow nm={t('조이스틱 감도')} sub={t('입력 응답 곡선')} right={<View style={{ width: 300 }}><Slider value={st.sensitivity} width="100%" onChange={st.setSensitivity} /></View>} />
      <TRow nm={t('화면 항상 켜기')} sub={t('Wake Lock — 제어 중 화면 유지')} right={<Toggle value={st.wakeLock} onChange={st.setWakeLock} />} />
      <TRow nm={t('데드존')} sub={t('중심 무시 범위')} right={<View style={{ width: 300 }}><Slider value={st.deadzone} width="100%" onChange={st.setDeadzone} /></View>} />
      <TRow nm={t('Gyro 위젯')} sub={t('자세계(roll/pitch) 표시 — 레거시 우상단 위젯 + 대시보드 인공수평 카드 (Qt gyroWidgetEnabled)')} right={<Toggle value={st.gyroWidgetEnabled} onChange={st.setGyroWidgetEnabled} />} />
      <TRow nm={t('속도 HUD')} sub={t('영상 위 주행 속도 표시 — 제어 홈 하단 중앙')}
        right={<Toggle value={st.speedHud} onChange={st.setSpeedHud} />} />
      {isDesktop() && (
        <TRow nm={t('UI 배율')} sub={t('화면 확대 — 스팀덱 등 소형 화면용 (자동: SteamOS 125%)')} right={
          <View style={{ width: 300 }}>
            <Segmented
              options={[
                { key: 'auto', label: t('자동') },
                { key: '1', label: '100%' }, { key: '1.25', label: '125%' },
                { key: '1.5', label: '150%' }, { key: '1.75', label: '175%' }, { key: '2', label: '200%' },
              ]}
              value={st.uiZoom == null ? 'auto' : String(st.uiZoom)}
              onChange={(v) => {
                const z = v === 'auto' ? null : Number(v);
                st.setUiZoom(z);
                setUiZoom(z).catch(() => {});
              }} />
          </View>
        } />
      )}
      <FullscreenRow />
      <TRow nm={t('저신호 경고')} sub={t('WiFi 신호가 임계 미만이면 상단 경고 배너 (Qt Low Connection Warning)')}
        right={<Toggle value={st.lowConnWarnEnabled} onChange={st.setLowConnWarnEnabled} />} />
      {st.lowConnWarnEnabled && (
        <TRow nm={t('경고 임계')} sub={t('이 % 미만이면 경고')} right={
          <View style={{ width: 300, flexDirection: 'row', alignItems: 'center', gap: 10 }}>
            <View style={{ flex: 1 }}>
              <Slider value={st.lowConnThreshold} width="100%" onChange={st.setLowConnThreshold} />
            </View>
            <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 12, fontWeight: '600', width: 38, textAlign: 'right' }}>
              {st.lowConnThreshold}%
            </Text>
          </View>
        } />
      )}
      {cloudConfigured && (Platform.OS === 'android' || Platform.OS === 'ios') && (
        <TRow nm={t('푸시 알림')}
          sub={accountPin
            ? t('앱을 닫아도 로봇 낙상·배터리 부족·꺼짐 등을 알림으로 받습니다')
            : t('계정 키를 입력하면 등록됩니다 — 배정된 로봇의 알림만 받습니다')}
          right={<Toggle value={st.pushAlerts} onChange={(v) => {
            st.setPushAlerts(v);
            if (v) { if (accountPin) registerPush(accountPin, true).catch(() => {}); }
            else unregisterPush().catch(() => {});
          }} />} />
      )}
      {Platform.OS === 'web' && (
        <TRow nm={t('경고 비프 (Notify BEEP)')} sub={t('저신호 등 경고 시 짧은 알림음')}
          right={<Toggle value={st.notifyBeep} onChange={(v) => { st.setNotifyBeep(v); if (v) warnBeep(); }} />} />
      )}
      {Platform.OS === 'web' && (
        <TRow nm={t('배터리 부족 경고음')} sub={t('로봇 배터리가 부족하면 알림음 — 경고 비프가 켜져 있어야 합니다')}
          right={<Toggle value={st.batteryLowBeep} onChange={st.setBatteryLowBeep} />} />
      )}
      {st.accessLevel >= 3 && !!ISSUES_REPO && (
        <TRow nm={t('GitHub 이슈 등록')} sub={`${t('앱·로봇 정보가 채워진 이슈 작성 화면을 브라우저로 엽니다')} (${ISSUES_REPO})`}
          right={
            <Tappable onPress={openGithubIssue}
              style={{ paddingHorizontal: 14, paddingVertical: 7, borderRadius: 9, borderWidth: 1, borderColor: c.line, backgroundColor: c.elev }}>
              <Text style={{ color: c.accent2, fontSize: 11, fontWeight: '600' }}>{t('이슈 작성 열기')}</Text>
            </Tappable>
          } />
      )}
    </>
  );
}
