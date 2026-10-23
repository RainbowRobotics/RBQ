import { Platform } from 'react-native';
import { RBSwitch } from '@/rb/components/RBSwitch';
import { RBToggleGroup } from '@/rb/components/RBToggleGroup';
import { RBSlider } from '@/rb/components/RBSlider';
import { RBSelectBox } from '@/rb/components/RBSelectBox';

export const inputVFix = Platform.OS === 'android'
  ? ({ paddingVertical: 0, textAlignVertical: 'center', includeFontPadding: false } as const)
  : null;

export function Toggle({ value, onChange, disabled }: { value: boolean; onChange?: (v: boolean) => void; disabled?: boolean }) {
  return <RBSwitch checked={value} disabled={disabled} onCheckedChange={(v) => onChange?.(v)} />;
}

export function Segmented<T extends string>({
  options, value, onChange,
}: {
  options: { key: T; label: string }[];
  value: T;
  onChange?: (v: T) => void;
}) {
  return (
    <RBToggleGroup stretched value={value} onValueChange={(v) => onChange?.(v as T)}
      items={options.map((o) => ({ value: o.key, label: o.label }))} />
  );
}

export function Slider({
  value = 50, onChange, onCommit, width = 230,
}: {
  value?: number; onChange?: (v: number) => void; onCommit?: (v: number) => void;
  width?: number | `${number}%`;
}) {
  return (
    <RBSlider value={value} min={0} max={100} step={1} style={{ width, paddingTop: 8, paddingBottom: 8 }}
      onValueChange={(v) => onChange?.(Math.round(v))} onValueCommit={(v) => onCommit?.(Math.round(v))} />
  );
}

export function Select<T extends string | number>({
  options, value, onChange, label, dense,
}: {
  options: { key: T; label: string }[];
  value: T;
  onChange?: (v: T) => void;
  label?: string;
  dense?: boolean;
}) {
  return (
    <RBSelectBox size={dense ? 'xs2' : 'sm'} placeholder={label ?? '—'} value={String(value)}
      options={options.map((o) => ({ value: String(o.key), label: o.label }))}
      onChange={(v) => { const hit = options.find((o) => String(o.key) === v); if (hit) onChange?.(hit.key); }} />
  );
}
