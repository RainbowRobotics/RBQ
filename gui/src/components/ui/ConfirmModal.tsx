import { useState } from 'react';
import { ScrollView, View, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { RBConfirm } from '@/rb/components/RBDialog';
import { RbModal } from '@/rb/components/RBOverlayModal';
import { RBCheckBox } from '@/rb/components/RBCheckBox';
import { t } from '@/lib/i18n';

const SKIP_MS = 24 * 60 * 60 * 1000;

type SkipState = { until: Record<string, number>; setSkip: (key: string) => void };
const useConfirmSkip = create<SkipState>()(
  persist(
    (set, get) => ({
      until: {},
      setSkip: (key) => set({ until: { ...get().until, [key]: Date.now() + SKIP_MS } }),
    }),
    { name: 'rbq-confirm-skip', storage: createJSONStorage(() => AsyncStorage) },
  ),
);

export function confirmSkipped(key: string): boolean {
  const u = useConfirmSkip.getState().until[key];
  return !!u && Date.now() < u;
}

export function ConfirmModal({ title, message, confirmLabel, danger = true, skipKey, onConfirm, onClose, children }: {
  title: string;
  message?: string;
  confirmLabel: string;
  danger?: boolean;
  skipKey?: string;
  onConfirm: () => void;
  onClose: () => void;
  children?: React.ReactNode;
}) {
  const [skip, setSkip] = useState(false);
  const extra = children || skipKey != null;
  const insets = useSafeAreaInsets();
  const { height } = useWindowDimensions();
  const bodyMax = Math.max(96, height - insets.top - insets.bottom - 200);
  return (
    <RbModal visible onRequestClose={onClose}>
    <View style={{ position: 'absolute', top: insets.top, bottom: insets.bottom, left: insets.left, right: insets.right }}>
    <RBConfirm open title={title} description={message} yesVariant={danger ? 'danger' : 'success'}
      buttonText={{ yes: confirmLabel, no: t('취소') }}
      onCancel={onClose} onOpenChange={(o) => { if (!o) onClose(); }}
      onConfirm={() => { if (skip && skipKey != null) useConfirmSkip.getState().setSkip(skipKey); onConfirm(); }}
      content={extra ? (
        <ScrollView style={{ maxHeight: bodyMax }} contentContainerStyle={{ gap: 12 }}>
          {children}
          {skipKey != null && <RBCheckBox checked={skip} onChange={setSkip}>{t('하루 동안 묻지 않음')}</RBCheckBox>}
        </ScrollView>
      ) : undefined} />
    </View>
    </RbModal>
  );
}
