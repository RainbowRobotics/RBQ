import { H2, Desc } from '@/components/panels/settings/common';
import { t } from '@/lib/i18n';

export function CameraQcPanel() {
  return (
    <>
      <H2>{t('카메라 QC')}</H2>
      <Desc>{t('준비 중입니다.')}</Desc>
    </>
  );
}
