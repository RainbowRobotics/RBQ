import { View } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { RBAlert } from '@/rb/components/RBDialog';
import { RbModal } from '@/rb/components/RBOverlayModal';
import { useRobot } from '@/store/robot';
import { rememberAgain, dismissTlsPrompt } from '@/lib/tlsPrompt';
import { t } from '@/lib/i18n';

export function RobotTlsPrompt() {
  const issue = useRobot((s) => s.tlsIssue);
  const insets = useSafeAreaInsets();
  if (!issue) return null;
  const close = dismissTlsPrompt;
  if (issue.kind === 'changed') {
    return (
      <ConfirmModal title={t('로봇이 바뀌었습니다')}
        message={t('전에 붙던 로봇과 인증서가 다릅니다. 로봇을 새로 설치했다면 다시 기억을 누르세요. 아니라면 다른 기기가 이 로봇 행세를 하고 있을 수 있습니다.')}
        confirmLabel={t('다시 기억')}
        onConfirm={() => rememberAgain(issue.entryKey, issue.fps, issue.forget)}
        onClose={close} />
    );
  }
  return (
    <RbModal visible onRequestClose={close}>
      <View style={{ position: 'absolute', top: insets.top, bottom: insets.bottom, left: insets.left, right: insets.right }}>
        <RBAlert open description={t('이 로봇은 HTTPS 가 없는 옛 버전입니다 — 최신으로 패치하면 다시 붙습니다')} buttonText={t('닫기')}
          onConfirm={close} onOpenChange={(o) => { if (!o) close(); }} />
      </View>
    </RbModal>
  );
}
