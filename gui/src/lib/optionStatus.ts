import { useEffect } from 'react';
import { create } from 'zustand';
import { t } from '@/lib/i18n';
import { useRobot } from '@/store/robot';
import { useFeatures } from '@/store/capability';
import { rest } from './rest';

export type OptionStatusItem = {
  key: string; label: { ko: string; en: string }; value: string | number | boolean | null; unit: string; level: 'ok' | 'warn' | 'error';
};
export type OptionStatusResp = { items: OptionStatusItem[]; age_ms: number | null; fresh_ms: number };

const isItem = (x: unknown): x is OptionStatusItem => {
  const o = x as OptionStatusItem;
  return !!o && typeof o === 'object' && typeof o.key === 'string' && !!o.label && typeof o.label.ko === 'string'
    && typeof o.label.en === 'string' && 'value' in o;
};

export const isFresh = (r?: OptionStatusResp) => !!r && typeof r.age_ms === 'number' && r.age_ms <= r.fresh_ms;

const show = (v: OptionStatusItem['value']) =>
  v === null || v === undefined ? '—' : typeof v === 'boolean' ? (v ? t('예') : t('아니오')) : String(v);

export function optionStatusRows(r: OptionStatusResp, lang: 'ko' | 'en'): [string, string][] {
  return (Array.isArray(r?.items) ? r.items : []).filter(isItem).map((it) => [
    lang === 'en' ? it.label.en : it.label.ko,
    it.unit ? `${show(it.value)} ${it.unit}` : show(it.value),
  ]);
}

export function itemValue(r: OptionStatusResp | undefined, key: string) {
  if (!isFresh(r)) return undefined;
  return r!.items.find((it) => isItem(it) && it.key === key)?.value;
}
export function itemBool(r: OptionStatusResp | undefined, key: string) {
  const v = itemValue(r, key);
  return typeof v === 'boolean' ? v : undefined;
}

export const useOptionStatus = create<{ resp?: OptionStatusResp; set: (r?: OptionStatusResp) => void }>()((set) => ({
  resp: undefined,
  set: (resp) => set({ resp }),
}));

export function useOptionStatusPoll(intervalMs = 1000) {
  const ip = useRobot((s) => s.ip);
  const option = useFeatures((s) => s.features?.option ?? '');
  useEffect(() => {
    if (!ip || !option) { useOptionStatus.getState().set(undefined); return; }
    let alive = true;
    const tick = () => {
      rest.optionStatus(ip)
        .then((r) => { if (alive && r && Array.isArray(r.items)) useOptionStatus.getState().set(r); })
        .catch(() => {});
    };
    tick();
    const timer = setInterval(tick, intervalMs);
    return () => { alive = false; clearInterval(timer); };
  }, [ip, option, intervalMs]);
}
