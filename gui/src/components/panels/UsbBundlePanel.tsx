import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { useRobot } from '@/store/robot';
import { useNotMine } from '@/lib/spectating';
import { useKnownSystem } from '@/store/robots';
import { useFeatures } from '@/store/capability';
import { LEGACY_SYSTEM } from '@/lib/firmwareRelease';
import { pickBundle, coreOn } from '@/lib/usbBundle';
import { useUsbJob, usbRun, usbRefs, startUsbBundle } from '@/store/usbBundle';
import { useSwJobs } from '@/store/swUpdate';
import { t } from '@/lib/i18n';

export function UsbBundlePanel({ robotVersion }: { robotVersion?: string }) {
  const { c, fonts, radius } = useTheme();
  const ip = useRobot((s) => s.ip);
  const conn = useRobot((s) => s.conn);
  const vacant = useRobot((s) => !s.owner);
  const notMine = useNotMine();
  const sys = useKnownSystem('motion', ip);
  const features = useFeatures((s) => s.features);
  const tgt = useSwJobs((s) => s.tgt);
  const job = useUsbJob();
  useUsbJob.useScreen();

  const close = () => useUsbJob.setState({ picked: null });
  const run = usbRun(job);
  const p = job.prog;

  let ask = null;
  if (job.picked) {
    const { manifest } = job.picked.bundle;
    const robot = {
      system: sys && sys !== LEGACY_SYSTEM ? sys : undefined, legacy: sys === LEGACY_SYSTEM, version: robotVersion,
      option: typeof features?.option === 'string' ? features.option : '', optionVersion: String(features?.option_version ?? ''),
    };
    const why = conn !== 'connected' ? t('로봇에 연결한 뒤 고르세요')
      : notMine ? (vacant ? t('조종하는 기기가 없습니다 — 제어권을 가져온 뒤에 설치할 수 있습니다')
        : t('다른 기기가 조종 중입니다 — 받기는 되고, 로봇 전송은 제어권을 가져온 뒤에 됩니다'))
      : tgt !== 'motion' ? t('묶음 설치는 Motion 대상에서만 됩니다 — 대상을 Motion 으로 바꾸세요') : null;
    const pick = why ?? pickBundle(job.picked.bundle, robot);
    const title = `${t('USB 묶음')} ${manifest.label}`;
    if (typeof pick === 'string') {
      ask = <ConfirmModal danger={false} title={title} message={pick} confirmLabel={t('확인')} onConfirm={close} onClose={close} />;
    } else {
      const optName = `${pick.option.option.toUpperCase()} ${t('옵션')}`;
      const same = (on: boolean) => (on ? ` · ${t('이미 깔림')}` : '');
      const row = (k: string, v: string) => (
        <View key={k} style={{ flexDirection: 'row', justifyContent: 'space-between', paddingVertical: 3, gap: 10 }}>
          <Text style={{ color: c.muted, fontSize: 12 }}>{k}</Text>
          <Text style={{ color: c.text, fontSize: 12, fontFamily: fonts.mono }}>{v}</Text>
        </View>
      );
      ask = (
        <ConfirmModal title={title} confirmLabel={t('설치')} onConfirm={() => startUsbBundle(ip)} onClose={close}
          message={pick.legacy ? t('구형 로봇은 중간에 로봇 PC 를 재부팅합니다 — 반드시 앉혀 두세요')
            : t('로봇을 앉혀 두세요 — 중간에 로봇 소프트웨어가 다시 켜집니다')}>
          <View>
            {row(t('대상'), `Motion (${ip})`)}
            {row(t('본체'), `${manifest.label}${same(!pick.legacy && coreOn(robotVersion, manifest.label, false))}`)}
            {row(optName, `${manifest.label}${same(robot.option === pick.option.option && robot.optionVersion === manifest.label)}`)}
            {row(t('시스템'), pick.system)}
            {pick.legacy ? (
              <Text style={{ color: c.amberTx, fontSize: 11.5, marginTop: 6, lineHeight: 15 }}>{t('구형 로봇 — 옛 꾸러미를 먼저 깝니다')}</Text>
            ) : null}
          </View>
        </ConfirmModal>
      );
    }
  }

  return (
    <>
      {run ? (
        <View style={{ paddingHorizontal: 14, paddingVertical: 11, borderWidth: 1, borderColor: c.line, borderRadius: radius.md, backgroundColor: c.bg, marginTop: 10 }}>
          <View style={{ flexDirection: 'row', justifyContent: 'space-between', gap: 10, marginBottom: 6 }}>
            <Text style={{ color: c.text, fontSize: 11.5, fontFamily: fonts.mono }}>{t('USB 묶음')} {job.label}</Text>
            <Text style={{ color: c.accent2, fontSize: 11.5, fontWeight: '700', flexShrink: 1, textAlign: 'right' }}>{run}</Text>
          </View>
          {p?.phase === 'transfer' ? (
            <>
              <View style={{ height: 8, borderRadius: 4, backgroundColor: c.elev, overflow: 'hidden' }}>
                <View style={{ height: 8, width: `${p.percentage}%`, backgroundColor: c.accent }} />
              </View>
              <View style={{ flexDirection: 'row', justifyContent: 'flex-end', marginTop: 8 }}>
                <Tappable onPress={() => usbRefs.handle.current?.cancel()}
                  style={{ flexDirection: 'row', alignItems: 'center', height: 28, paddingHorizontal: 14, borderWidth: 1, backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }}>
                  <Text style={{ color: c.text, fontSize: 12 }}>{t('취소')}</Text>
                </Tappable>
              </View>
            </>
          ) : null}
        </View>
      ) : null}
      {job.reading ? <Text style={{ color: c.dim, fontSize: 12, marginTop: 8 }}>{t('USB 묶음 읽는 중…')}</Text> : null}
      {!run && job.err ? <Text style={{ color: c.redTx, fontSize: 12, marginTop: 8 }}>{t('오류:')} {job.err}</Text> : null}
      {!run && job.done ? <Text style={{ color: c.greenTx, fontSize: 12, marginTop: 8 }}>✓ {job.done}</Text> : null}
      {ask}
    </>
  );
}
