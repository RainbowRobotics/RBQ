import { useEffect, useState } from 'react';
import { View, Text, Pressable, StyleSheet, Platform , useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Popover } from '@/components/ui/overlays';
import { Select } from '@/components/ui/controls';
import { Toggle, Slider } from '@/components/ui/controls';
import { useSettings } from '@/store/settings';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { ptt, usePtt } from '@/lib/ptt';
import { webrtcClient } from '@/lib/webrtcClient';
import { PDU_PORT } from '@/lib/robotState';
import { actions } from '@/lib/rest';
import { railAnchor } from '@/components/control/ToolRail';
import { t } from '@/lib/i18n';
import { RecordControl, SoundQuick } from '@/components/panels/MediaPanel';
import { useRouter } from 'expo-router';

const micGainSupported =
  Platform.OS === 'web' &&
  typeof navigator !== 'undefined' &&
  !!(navigator.mediaDevices?.getSupportedConstraints?.() as any)?.volume;

function applyMicGain(pct: number) {
  const track: MediaStreamTrack | null = (webrtcClient as any).micTrack ?? null;
  track?.applyConstraints({ advanced: [{ volume: pct / 100 } as any] }).catch(() => {});
}

const deviceListSupported =
  Platform.OS === 'web' &&
  typeof navigator !== 'undefined' &&
  typeof navigator.mediaDevices?.enumerateDevices === 'function';

const sinkSupported =
  deviceListSupported &&
  typeof document !== 'undefined' &&
  typeof (document.createElement('audio') as any).setSinkId === 'function';

type Dev = { deviceId: string; label: string };

function GcsDeviceRows() {
  const { c, radius } = useTheme();
  const { height: winH } = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const spkId = useSettings((s) => s.gcsSpeakerId);
  const micId = useSettings((s) => s.gcsMicId);
  const [spks, setSpks] = useState<Dev[]>([]);
  const [mics, setMics] = useState<Dev[]>([]);
  const [noLabels, setNoLabels] = useState(false);

  const enumerate = () => {
    navigator.mediaDevices.enumerateDevices().then((list) => {
      const pick = (kind: string): Dev[] =>
        list.filter((d) => d.kind === kind && d.deviceId !== 'default' && d.deviceId !== 'communications').map((d, i) => ({
          deviceId: d.deviceId,
          label: d.label || t('장치 {n}').replace('{n}', String(i + 1)),
        }));
      setSpks(pick('audiooutput'));
      setMics(pick('audioinput'));
      setNoLabels(list.some((d) => d.kind === 'audioinput' || d.kind === 'audiooutput') && list.every((d) => !d.label));
    }).catch(() => {});
  };
  useEffect(() => {
    enumerate();
    navigator.mediaDevices.addEventListener?.('devicechange', enumerate);
    return () => navigator.mediaDevices.removeEventListener?.('devicechange', enumerate);
  }, []);

  const askPermission = async () => {
    try {
      const s = await navigator.mediaDevices.getUserMedia({ audio: true });
      s.getTracks().forEach((t) => t.stop());
    } catch {}
    enumerate();
  };

  const DevList = ({ title, devs, selected, onPick }: {
    title: string; devs: Dev[]; selected: string; onPick: (id: string) => void;
  }) => devs.length < 2 ? (
    <View style={[styles.slTop, { marginBottom: 6, gap: 8 }]}>
      <Text numberOfLines={1} style={{ color: c.dim, fontSize: 9.5, fontWeight: '700' }}>{title}</Text>
      <Text numberOfLines={1} style={{ flex: 1, color: c.text, fontSize: 10, textAlign: 'right' }}>{devs[0]?.label}</Text>
    </View>
  ) : (
    <View style={{ marginBottom: 6 }}>
      <Text numberOfLines={1} style={{ color: c.dim, fontSize: 9.5, fontWeight: '700', marginBottom: 3 }}>{title}</Text>
      <Select label={title} value={selected || devs[0]?.deviceId || ''}
        options={devs.map((d) => ({ key: d.deviceId || d.label, label: d.label }))}
        onChange={(id) => onPick(String(id))} />
    </View>
  );

  return (
    <View style={{ marginBottom: 4 }}>
      {noLabels && (
        <Pressable onPress={askPermission}
          style={[styles.devRow, { borderRadius: radius.sm, borderColor: 'rgba(210,153,34,0.5)', backgroundColor: 'rgba(210,153,34,0.10)' }]}>
          <Text style={{ flex: 1, color: c.amberTx, fontSize: 10, fontWeight: '600' }}>{t('장치 이름 표시 — 마이크 권한 허용')}</Text>
        </Pressable>
      )}
      {sinkSupported && spks.length > 0 && (
        <DevList title={t('스피커 (출력)')} devs={spks} selected={spkId}
          onPick={(id) => {
            useSettings.getState().setGcsSpeakerId(id === 'default' ? '' : id);
            (webrtcClient as any).applySpeakerDevice?.(id === 'default' ? '' : id);
          }} />
      )}
      {mics.length > 0 && (
        <DevList title={t('마이크')} devs={mics} selected={micId}
          onPick={(id) => {
            useSettings.getState().setGcsMicId(id === 'default' ? '' : id);
            (webrtcClient as any).resetMicTrack?.();
          }} />
      )}
    </View>
  );
}

function GcsMicRow() {
  const { c, fonts } = useTheme();
  const v = useSettings((s) => s.gcsMicGain);
  const setGcsMicGain = useSettings((s) => s.setGcsMicGain);
  return (
    <View style={{ marginBottom: 12, width: '100%' }}>
      <View style={styles.slTop}>
        <Text style={{ fontSize: 11, color: c.text }}>{t('내 마이크')}<Text style={{ color: c.dim, fontSize: 9 }}>{t('  송출 게인')}</Text></Text>
        <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 12, fontWeight: '600' }}>{v}%</Text>
      </View>
      <Slider value={v} width="100%" onChange={setGcsMicGain} onCommit={applyMicGain} />
    </View>
  );
}

function VolRow({ k, param, onCommit }: {
  k: string; param: 'robotSpk' | 'robotMic' | 'appVol'; onCommit: (v: number) => void;
}) {
  const { c, fonts } = useTheme();
  const v = useSettings((s) => s.audio[param]);
  const setAudio = useSettings((s) => s.setAudio);
  return (
    <View style={{ marginBottom: 9 }}>
      <View style={styles.slTop}>
        <Text numberOfLines={1} style={{ fontSize: 11, color: c.text, flex: 1 }}>{k}</Text>
        <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 11.5, fontWeight: '600' }}>{v}%</Text>
      </View>
      <Slider value={v} width="100%" onChange={(nv) => setAudio(param, nv)} onCommit={onCommit} />
    </View>
  );
}

export function AudioBody({ onOpenList }: { onOpenList?: () => void } = {}) {
  const { c, radius } = useTheme();
  const listen = usePtt((s) => s.listen);
  const error = usePtt((s) => s.error);
  const noMic = usePtt((s) => s.status?.input === 'none');
  const toPtz = usePtt((s) => s.status?.output === 'ptz');
  const noOut = usePtt((s) => s.status?.output === 'none');
  const fromPtz = usePtt((s) => s.status?.input === 'ptz');
  const micQuiet = usePtt((s) => s.status?.input === 'robot' && s.status?.mic_signal === false);
  useEffect(() => {
    ptt.refreshStatus();
    const id = setInterval(() => ptt.refreshStatus(), 5000);
    return () => clearInterval(id);
  }, []);
  const appVol = useSettings((s) => s.audio.appVol);
  const ampOff = useTelemetry((s) => s.pdu?.amp === false);
  const ip = useRobot((s) => s.ip);

  useEffect(() => { ptt.setAppVolume(appVol); }, [appVol]);

  return (
    <>
      <View style={styles.listenRow}>
        <Icon name="headset" size={14} color={c.muted} />
        <View style={{ flex: 1 }}>
          <Text style={{ fontSize: 11, fontWeight: '600', color: c.text }}>{t('로봇 소리 듣기')}</Text>
          <Text style={{ fontSize: 9, color: noMic ? c.amberTx : listen ? c.greenTx : c.dim }}>
            {noMic ? t('로봇에 마이크가 없습니다') : listen ? (fromPtz ? t('듣는 중 · PTZ 마이크') : t('듣는 중')) : t('꺼짐')}
          </Text>
        </View>
        <Toggle value={listen && !noMic} onChange={(v) => { if (!noMic || !v) ptt.setListen(v); }} disabled={noMic} />
      </View>
      {micQuiet && (
        <Text style={{ fontSize: 9, color: c.amberTx, marginTop: -6, marginBottom: 10 }}>
          {t('로봇 마이크에 소리가 거의 들어오지 않습니다 — 마이크가 꽂혀 있는지 확인하세요')}
        </Text>
      )}
      {(toPtz || noOut) && (
        <Text style={{ fontSize: 9, color: noOut ? c.amberTx : c.muted, marginTop: -6, marginBottom: 10 }}>
          {noOut ? t('로봇에 스피커가 없어 말해도 소리가 나지 않습니다') : t('소리는 PTZ 스피커로 나갑니다')}
        </Text>
      )}

      <VolRow k={t('로봇 스피커')} param="robotSpk" onCommit={(v) => ptt.setRobotSpeakerVolume(v)} />
      <VolRow k={t('로봇 마이크')} param="robotMic" onCommit={(v) => ptt.setRobotMicVolume(v)} />
      <VolRow k={t('내 볼륨')} param="appVol" onCommit={(v) => ptt.setAppVolume(v)} />

      {ampOff && !toPtz && (
        <View style={[styles.ampWarn, { backgroundColor: 'rgba(210,153,34,0.10)', borderColor: 'rgba(210,153,34,0.5)', borderRadius: radius.md }]}>
          <View style={{ flex: 1 }}>
            <Text style={{ fontSize: 10.5, fontWeight: '600', color: c.amberTx }}>{t('로봇 스피커 전원 꺼짐')}</Text>
            <Text style={{ fontSize: 8.5, color: c.dim, marginTop: 1 }}>{t('PDU 앰프 레일이 꺼져 있어 말해도 소리가 안 납니다')}</Text>
          </View>
          <Pressable
            onPress={() => actions.pduPower(ip, PDU_PORT.SPEAKER, true).catch(() => {})}
            style={[styles.ampBtn, { backgroundColor: 'rgba(210,153,34,0.18)', borderColor: 'rgba(210,153,34,0.6)' }]}
          >
            <Text style={{ fontSize: 11, fontWeight: '700', color: c.amberTx }}>{t('켜기')}</Text>
          </Pressable>
        </View>
      )}
      <Text style={{ fontSize: 8.5, color: error ? c.amberTx : c.dim, marginTop: 7, textAlign: 'center' }}>
        {error === 'denied'
          ? t('마이크 권한이 없습니다 — 앱 설정에서 허용해주세요')
          : error === 'unavailable'
            ? t('로봇 오디오 채널 연결 실패 — 로봇 연결 상태를 확인해주세요')
            : t('말하기는 화면 오른쪽 아래 [워키토키]를 누르고 있는 동안')}
      </Text>
      <RecordControl compact onOpenList={onOpenList} />
      {(micGainSupported || deviceListSupported) && (
        <View style={{ marginTop: 12, paddingTop: 11, borderTopWidth: 1, borderColor: c.line2 }}>
          <Text style={{ color: c.muted, fontSize: 10, fontWeight: '700', marginBottom: 6 }}>{t('이 기기 (컨트롤러)')}</Text>
          {micGainSupported && <GcsMicRow />}
          {deviceListSupported && <GcsDeviceRows />}
        </View>
      )}
    </>
  );
}

export function AudioPopover({ onClose }: { onClose: () => void }) {
  const { height: winH } = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const pos = { ...railAnchor(winH, 'right', insets.right), width: 250 };
  const { c } = useTheme();
  const router = useRouter();
  return (
    <Popover onClose={onClose} style={pos}>
      <View style={styles.popH}>
        <Icon name="mic" size={13} color={c.accent2} />
        <Text style={{ color: c.muted, fontSize: 11, fontWeight: '600' }}>
          {t('워키토키')} <Text style={{ color: c.dim, fontWeight: '500' }}>{t('· 로봇 스피커/마이크')}</Text>
        </Text>
      </View>
      <AudioBody onOpenList={() => { onClose(); router.push('/media?sec=audio'); }} />
    </Popover>
  );
}

const styles = StyleSheet.create({
  popH: { flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 11 },
  slTop: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'baseline', marginBottom: 2 },
  listenRow: { flexDirection: 'row', alignItems: 'center', gap: 10, marginBottom: 12 },
  ampWarn: {
    flexDirection: 'row', alignItems: 'center', gap: 8,
    paddingHorizontal: 10, paddingVertical: 7, marginBottom: 8, borderWidth: 1,
  },
  ampBtn: { height: 28, paddingHorizontal: 12, borderRadius: 8, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  devRow: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 26, paddingHorizontal: 8, borderWidth: 1, marginBottom: 3 },
  devDot: { width: 5, height: 5, borderRadius: 3 },
});

export function SoundPopover({ onClose, right, bottom }: { onClose: () => void; right: number; bottom: number }) {
  const { height: winH } = useWindowDimensions();
  const { c } = useTheme();
  const router = useRouter();
  return (
    <Popover onClose={onClose} style={{ right, bottom, width: 300, maxHeight: Math.max(160, winH - bottom - 56 - 12) }}>
      <View style={styles.popH}>
        <Icon name="speaker" size={14} color={c.accent2} />
        <Text style={{ flex: 1, fontSize: 12, fontWeight: '700', color: c.text }}>{t('사운드')}</Text>
        <Pressable onPress={() => { onClose(); router.push('/media?sec=sound'); }} hitSlop={6}>
          <Text style={{ fontSize: 10, color: c.accent2, fontWeight: '600' }}>{t('목록')} ›</Text>
        </Pressable>
      </View>
      <SoundQuick />
    </Popover>
  );
}
