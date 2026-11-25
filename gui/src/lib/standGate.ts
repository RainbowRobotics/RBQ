import { create } from 'zustand';
import { getLang } from '@/store/lang';
import { t } from '@/lib/i18n';

export type StandBlock = { reason: 'equipment_without_payload' | 'option'; ids: string[]; option?: { ko?: string; en?: string } };
const REASONS: StandBlock['reason'][] = ['equipment_without_payload', 'option'];

export function standBlockOf(e: unknown): StandBlock | null {
  const err = e as { status?: number; detail?: string } | null;
  if (err?.status !== 409 || typeof err.detail !== 'string') return null;
  try {
    const b = JSON.parse(err.detail);
    if (!REASONS.includes(b?.error)) return null;
    return { reason: b.error, ids: Array.isArray(b.ids) ? b.ids.map(String) : [], option: b.option && typeof b.option === 'object' ? b.option : undefined };
  } catch { return null; }
}

export function standBlockText(b: StandBlock): string {
  if (b.reason === 'option') {
    const why = (getLang() === 'en' ? b.option?.en : b.option?.ko) || b.option?.ko || b.option?.en;
    return `${t('옵션 프로그램이 기립을 막았습니다')}${why ? ` — ${why}` : ''}. ${t('장비 무게가 반영되지 않으면 일어서다 넘어질 수 있습니다.')}`;
  }
  const ids = b.ids.length ? ` (${b.ids.join(' · ')})` : '';
  return `${t('등록 안 된 장비가 장비 버스에 있는데 페이로드 무게가 0 입니다')}${ids}. ${t('장비 무게가 반영되지 않으면 일어서다 넘어질 수 있습니다.')} ${t('[페이로드 설정]에서 무게를 넣거나 옵션을 확인하세요.')}`;
}

type State = { pending: (StandBlock & { force: () => void; ip?: string }) | null; close: () => void };
export const useStandGate = create<State>((set) => ({ pending: null, close: () => set({ pending: null }) }));

export function askStandGate(e: unknown, force: () => void, ip?: string): boolean {
  const b = standBlockOf(e);
  if (!b) return false;
  useStandGate.setState({ pending: { ...b, force, ip } });
  return true;
}
