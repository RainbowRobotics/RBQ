import { dashboardCards } from '@/modules/registry';
import { useFeatures } from '@/store/capability';
import { useLang } from '@/store/lang';
import { t } from '@/lib/i18n';
import { isFresh, optionStatusRows, useOptionStatus, useOptionStatusPoll } from '@/lib/optionStatus';

dashboardCards.push({
  key: 'option', icon: 'box',
  useData: () => {
    useOptionStatusPoll();
    const option = useFeatures((s) => s.features?.option ?? '');
    const resp = useOptionStatus((s) => s.resp);
    const lang = useLang((s) => s.lang);
    if (!option) return null;
    const fresh = isFresh(resp);
    return {
      title: `${t('옵션 상태')} · ${option}`,
      live: fresh,
      rows: fresh && resp ? optionStatusRows(resp, lang === 'en' ? 'en' : 'ko') : [],
      empty: t('옵션 프로그램 응답 없음 — 장치가 없거나 프로그램이 멈췄습니다.'),
    };
  },
});
