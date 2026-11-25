import { View, Text, Pressable, StyleSheet } from 'react-native';
import { useRouter } from 'expo-router';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import type { JobSource } from '@/store/jobs';
import { swJob } from '@/store/swUpdate';
import { optionJob } from '@/store/optionInstall';
import { usbJob } from '@/store/usbBundle';
import { robotCheckJob, cameraCheckJob } from '@/store/robotCheck';
import { logSaveJob } from '@/store/logSave';
import { mediaJob } from '@/store/mediaDownload';
import { zmpJob } from '@/store/zmpCalib';
import { autoStartJob } from '@/store/autoStart';
import { jobSources } from '@/modules/registry';
import { t } from '@/lib/i18n';

const CORE_JOBS: JobSource[] = [swJob, usbJob, optionJob, autoStartJob, zmpJob, robotCheckJob, cameraCheckJob, logSaveJob, mediaJob];

const TONE = { run: '#FBBA16', ok: '#00C853', bad: '#FF7A70' } as const;

export function JobBanners() {
  return <>{[...CORE_JOBS, ...jobSources].map((j) => <JobBanner key={j.key} job={j} />)}</>;
}

function JobBanner({ job }: { job: JobSource }) {
  const { radius } = useTheme();
  const router = useRouter();
  const b = job.useLine();
  if (!b) return null;
  const tone = TONE[b.tone];
  return (
    <View style={[styles.banner, { borderColor: tone, borderRadius: radius.md }]}>
      <Tappable onPress={() => router.navigate(job.route as never)} accessibilityLabel={job.label()} style={styles.body}>
        <Icon name={job.icon} size={14} color={tone} />
        <Text style={[styles.txt, { color: tone }]}>{b.text}</Text>
      </Tappable>
      {b.closable ? (
        <Pressable onPress={job.dismiss} hitSlop={8} accessibilityLabel={t('닫기')} style={{ marginLeft: 2 }}>
          <Icon name="x" size={13} color={tone} />
        </Pressable>
      ) : null}
    </View>
  );
}

const styles = StyleSheet.create({
  banner: {
    flexDirection: 'row', alignItems: 'center', gap: 8, maxWidth: '92%',
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1, backgroundColor: 'rgba(0,0,0,0.55)',
  },
  body: { flexDirection: 'row', alignItems: 'center', gap: 8, flexShrink: 1 },
  txt: { fontSize: 11.5, fontWeight: '700', flexShrink: 1 },
});
