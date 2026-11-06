import { createContext, useContext } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { Text as RbText } from '@/rb/native';
import { useTheme } from '@/theme';
import { useRb } from '@/rb/theme';
import { RBTextField } from '@/rb/components/RBTextField';
import { RBTextInput } from '@/rb/components/RBTextInput';
import { RBLabelButton, type LabelButtonVariant } from '@/rb/components/RBLabelButton';
import { Icon, type IconName } from '@/components/Icon';

export const LegibleText = createContext(false);
export const useLegible = () => useContext(LegibleText);

export function H2({ children }: { children: React.ReactNode }) {
  const { c, t } = useRb();
  const lg = useLegible();
  return <RbText style={[t(lg ? 'title-03-bold' : 'title-01-bold'), { color: c('fg-default'), marginBottom: lg ? 4 : 3 }]}>
    {lg ? <RbText style={{ color: c('fill-brand') }}>▍</RbText> : null}{children}</RbText>;
}
export function Desc({ children }: { children: React.ReactNode }) {
  const { c, t } = useRb();
  const lg = useLegible();
  return <RbText style={[t(lg ? 'body-sm-normal' : 'body-xs-normal'), { color: c(lg ? 'fg-subtle' : 'fg-subtler'), marginBottom: lg ? 12 : 16 }]}>{children}</RbText>;
}
export function Group({ children }: { children: React.ReactNode }) {
  const { radius } = useTheme();
  const { c } = useRb();
  if (!useLegible()) return <>{children}</>;
  return (
    <View style={{ marginLeft: 14, borderWidth: 1, borderColor: c('border-subtle'), borderRadius: radius.md, backgroundColor: c('bg-card'), paddingHorizontal: 16, overflow: 'hidden' }}>
      <View style={{ marginTop: -1 }}>{children}</View>
    </View>
  );
}
const FILL = { width: 'auto', minWidth: 0, alignSelf: 'stretch' } as const;
export function Field({ label, value, muted }: { label: string; value: string; muted?: boolean }) {
  const { fonts } = useTheme();
  const { c } = useRb();
  return (
    <RBTextField label={label} style={{ marginBottom: 14, flex: 1 }}>
      <RBTextInput size="sm" value={value} readOnly hideBtnReset style={FILL} inputStyle={{ fontFamily: fonts.mono, color: c(muted ? 'fg-subtlest' : 'fg-default') }} />
    </RBTextField>
  );
}
export function EditField({ label, value, onChangeText, onSubmit, keyboardType = 'default', locked, onUnlock }: {
  label: string; value: string; onChangeText: (t: string) => void; onSubmit?: () => void;
  keyboardType?: 'default' | 'decimal-pad' | 'number-pad' | 'numbers-and-punctuation';
  locked?: boolean; onUnlock?: () => void;
}) {
  const { fonts } = useTheme();
  const { c } = useRb();
  return (
    <RBTextField label={label} style={{ marginBottom: 14, flex: 1 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, alignSelf: 'stretch' }}>
        <RBTextInput size="sm" value={value} onChangeText={onChangeText} onEnter={() => onSubmit?.()} keyboardType={keyboardType} autoCapitalize="none"
          readOnly={locked} hideBtnReset style={[FILL, { flex: 1 }]} inputStyle={{ fontFamily: fonts.mono }} />
        {locked && (
          <Text onPress={onUnlock} accessibilityLabel="unlock"
            style={{ color: c('fg-subtle'), fontSize: 15, paddingHorizontal: 8, paddingVertical: 6 }}>🔒</Text>
        )}
      </View>
    </RBTextField>
  );
}
export function TRow({ nm, sub, right }: { nm: string; sub: string; right: React.ReactNode }) {
  const { c, t } = useRb();
  return (
    <View style={[S.trow, { borderTopColor: c('border-subtler') }]}>
      <View style={{ flex: 1 }}>
        <RbText style={[t('body-sm-normal'), { color: c('fg-default') }]}>{nm}</RbText>
        <RbText style={[t('compact-xs-normal'), { color: c('fg-subtler'), marginTop: 2 }]}>{sub}</RbText>
      </View>
      {right}
    </View>
  );
}
const SBTN_VARIANT: Record<'primary' | 'ghost' | 'danger', LabelButtonVariant> = { primary: 'brand', ghost: 'outlined', danger: 'danger' };
export function SBtn({ kind, icon, label, onPress, disabled }: { kind: 'primary' | 'ghost' | 'danger'; icon: IconName; label: string; onPress?: () => void; disabled?: boolean }) {
  const { c } = useRb();
  return (
    <RBLabelButton size="sm" variant={SBTN_VARIANT[kind]} disabled={disabled} onPress={onPress}
      leadingIcon={<Icon name={icon} size={15} color={kind === 'ghost' ? c('fg-default') : c('fg-static-white')} />}>{label}</RBLabelButton>
  );
}

export const S = StyleSheet.create({
  row2: { flexDirection: 'row', gap: 13 },
  row3: { flexDirection: 'row', gap: 11 },
  note: { borderRadius: 9, borderWidth: 1, paddingHorizontal: 12, paddingVertical: 9, marginBottom: 15 },
  statusline: { flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 16 },
  sdot: { width: 7, height: 7, borderRadius: 4 },
  actions: { flexDirection: 'row', gap: 10, marginTop: 20 },
  trow: { flexDirection: 'row', alignItems: 'center', paddingVertical: 12, borderTopWidth: 1 },
  keychip: { minWidth: 36, height: 26, paddingHorizontal: 8, borderRadius: 7, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  slrange: { flexDirection: 'row', justifyContent: 'space-between', marginTop: 6 },
  rngTxt: { fontSize: 10, color: '#888', fontFamily: 'monospace' },
});
