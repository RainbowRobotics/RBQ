import { useState } from 'react';
import { View, Text, ScrollView, TextInput, useWindowDimensions } from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { RBDialog } from '@/rb/components/RBDialog';
import { RbModal } from '@/rb/components/RBOverlayModal';
import { RBCheckBox } from '@/rb/components/RBCheckBox';
import { RBLabelButton } from '@/rb/components/RBLabelButton';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { t } from '@/lib/i18n';
import {
  PRESET_LINES, clockLabel, kindLabel, prepNote, rerunNeedsPresets, serialOk, startLines, stepName, withPrereqs,
  type EngineStatus, type Kind, type Level,
} from '@/lib/ptzQc';
import { Note, VerdictChip, bareName } from '@/components/ptzqc/parts';

export function QcDialog({ title, desc, children, footer, onClose, width = 560 }: {
  title: string; desc?: string; children?: React.ReactNode; footer: React.ReactNode; onClose?: () => void; width?: number;
}) {
  const insets = useSafeAreaInsets();
  const { height, width: sw } = useWindowDimensions();
  const bodyMax = Math.max(120, height - insets.top - insets.bottom - 210);
  return (
    <RbModal visible onRequestClose={onClose}>
      <View style={{ position: 'absolute', top: insets.top, bottom: insets.bottom, left: insets.left, right: insets.right }}>
        <RBDialog open title={title} description={desc} width={Math.min(width, sw - 24)} onEscape={onClose} onOverlayPress={onClose}
          footer={<View style={{ flexDirection: 'row', gap: 8, justifyContent: 'flex-end', width: '100%' }}>{footer}</View>}>
          <ScrollView style={{ maxHeight: bodyMax }} contentContainerStyle={{ gap: 10, paddingHorizontal: 16, paddingBottom: 6 }}>
            {children}
          </ScrollView>
        </RBDialog>
      </View>
    </RbModal>
  );
}

function Bullets({ lines }: { lines: string[] }) {
  const { c } = useTheme();
  return (
    <View style={{ gap: 6 }}>
      {lines.map((l) => (
        <View key={l} style={{ flexDirection: 'row', gap: 8 }}>
          <Text style={{ color: c.muted, fontSize: 13 }}>•</Text>
          <Text style={{ color: c.text, fontSize: 13, flex: 1, lineHeight: 19 }}>{l}</Text>
        </View>
      ))}
    </View>
  );
}

export function StartDialog({ level, kind, steps, soakMin, recentWarn, onStart, onClose }: {
  level: Level; kind: Kind; steps?: number[]; soakMin: number; recentWarn: boolean; onStart: (presets: boolean) => void; onClose: () => void;
}) {
  const { c, radius } = useTheme();
  const [agree, setAgree] = useState(false);
  const askPresets = level >= 2 && (kind !== 'rerun' || rerunNeedsPresets(steps));
  const what = `${kindLabel(kind, kind === 'aging' ? soakMin : undefined)} · ${t('레벨')} ${level}`;
  return (
    <QcDialog title={t('PTZ 점검을 시작할까요?')} desc={what} onClose={onClose}
      footer={<>
        <RBLabelButton variant="outlined" onPress={onClose}>{t('취소')}</RBLabelButton>
        <RBLabelButton variant="brand" onPress={() => onStart(askPresets ? agree : false)}>{t('시작')}</RBLabelButton>
      </>}>
      {recentWarn ? <Note tone="warn" title={t('최근 10초 안에 PTZ 조작이 있었습니다.')} text={t('시작하면 그 조작이 끊깁니다.')} /> : null}
      <Bullets lines={startLines(level, kind, steps)} />
      {askPresets ? (
        <View style={{ borderWidth: 1, borderColor: c.line, borderRadius: radius.sm, padding: 10, gap: 6 }}>
          <RBCheckBox checked={agree} onChange={setAgree}>{t('프리셋 변경에 동의합니다')}</RBCheckBox>
          <Bullets lines={PRESET_LINES.map((l) => t(l))} />
          {!agree ? <Text style={{ color: c.dim, fontSize: 12 }}>{t('동의하지 않으면 11 · 18단계를 건너뜁니다.')}</Text> : null}
        </View>
      ) : null}
    </QcDialog>
  );
}

export function AskDialog({ st, canAnswer, stale, onEnd, onGo }: {
  st: EngineStatus; canAnswer: boolean; stale: string | null; onEnd: () => void; onGo: () => void;
}) {
  const { c, radius, fonts } = useTheme();
  const g = st.gate!;
  return (
    <QcDialog title={t('QC FAIL — 에이징을 계속할까요?')}
      footer={<>
        <View style={{ flex: 1 }}><RBLabelButton variant="outlined" disabled={!canAnswer} onPress={onEnd} style={{ width: '100%' }}>{t('여기서 종료')}</RBLabelButton></View>
        <View style={{ flex: 1 }}><RBLabelButton variant="brand" disabled={!canAnswer} onPress={onGo} style={{ width: '100%' }}>{t('에이징 진행')}</RBLabelButton></View>
      </>}>
      {stale ? <Note tone="warn" text={stale} /> : null}
      {!canAnswer ? <Note tone="warn" text={t('관전 중이거나 연결이 없어 답할 수 없습니다 — 조종하는 기기에서 답해 주세요')} /> : null}
      <View style={{ borderWidth: 1, borderColor: c.line, borderRadius: radius.sm, paddingHorizontal: 10, paddingVertical: 4 }}>
        {(g.fails ?? []).map((f) => (
          <View key={f.n} style={{ flexDirection: 'row', alignItems: 'center', gap: 10, paddingVertical: 6, borderBottomWidth: 1, borderColor: c.line }}>
            <VerdictChip v="FAIL" />
            <Text style={{ color: c.text, fontSize: 13, fontWeight: '700', width: 130 }}>{f.n} {bareName(f.name)}</Text>
            <Text style={{ color: c.muted, fontSize: 12.5, flex: 1 }} numberOfLines={2}>{f.measured}</Text>
          </View>
        ))}
      </View>
      <Text style={{ color: c.text, fontSize: 13 }}>{t('계속하면 공회전과 재검사를 한 뒤 최종 보고서에 FAIL 로 남습니다.')}</Text>
      <Text style={{ color: c.dim, fontSize: 12, fontFamily: fonts.mono }}>
        {t("{t} 뒤 '여기서 종료' 로 넘어갑니다").replace('{t}', clockLabel(g.remainS))}
      </Text>
    </QcDialog>
  );
}

export function RerunDialog({ base, level, onRun, onClose }: { base: string; level: Level; onRun: (steps: number[]) => void; onClose: () => void }) {
  const { c } = useTheme();
  const [picked, setPicked] = useState<number[]>([]);
  const can = (n: number) => !(level === 1 && (n === 11 || n === 18));
  const toggle = (n: number) => setPicked((p) => (p.includes(n) ? p.filter((x) => x !== n) : [...p, n].sort((a, b) => a - b)));
  const all = withPrereqs(picked, level);
  return (
    <QcDialog title={t('단계 다시 돌리기')} width={640}
      desc={`${t('가장 최근 검사')} ${base} · ${t('고른 단계만 QC 순서대로 다시 돕니다 · 19 · 20 은 고를 수 없습니다')}`} onClose={onClose}
      footer={<>
        <RBLabelButton variant="outlined" onPress={onClose}>{t('취소')}</RBLabelButton>
        <RBLabelButton variant="brand" disabled={!picked.length} onPress={() => onRun(picked)}
          leadingIcon={<Icon name="recover" size={15} color="#fff" />}>
          {picked.length ? t('{n}단계 다시 돌리기').replace('{n}', String(picked.length)) : t('단계를 고르세요')}
        </RBLabelButton>
      </>}>
      <View style={{ flexDirection: 'row', flexWrap: 'wrap' }}>
        {Array.from({ length: 18 }, (_, i) => i + 1).map((n) => (
          <View key={n} style={{ width: '50%', paddingVertical: 3, opacity: can(n) ? 1 : 0.4 }}>
            <RBCheckBox checked={picked.includes(n)} onChange={() => can(n) && toggle(n)} disabled={!can(n)}>
              {`${n}  ${stepName(n)}`}
            </RBCheckBox>
          </View>
        ))}
      </View>
      {picked.length ? <Note icon="recover" text={prepNote(picked, level)} /> : null}
      {picked.length ? <Text style={{ color: c.dim, fontSize: 12 }}>{t('이번에 도는 단계')}: {all.join(' · ')}</Text> : null}
      <Text style={{ color: c.dim, fontSize: 12, lineHeight: 18 }}>
        {t('처음 결과는 지워지지 않습니다. 처음 FAIL 이었던 단계는 다시 돌려 PASS 가 나와도 종합이 WARN 까지만 올라갑니다.')}
      </Text>
    </QcDialog>
  );
}

export function SerialDialog({ current, onWrite, onClose }: { current: string; onWrite: (s: string) => void; onClose: () => void }) {
  const { c, radius, fonts } = useTheme();
  const [v, setV] = useState(current);
  const val = v.trim().toUpperCase();
  return (
    <QcDialog title={t('제품 시리얼 쓰기')} desc={t('YAW 보드 EEPROM 의 시리얼 칸에 씁니다. 다음 검사부터 보고서 이름이 이 시리얼이 됩니다.')} onClose={onClose} width={460}
      footer={<>
        <RBLabelButton variant="outlined" onPress={onClose}>{t('취소')}</RBLabelButton>
        <RBLabelButton variant="brand" disabled={!serialOk(val) || val === current} onPress={() => onWrite(val)}>{t('쓰기')}</RBLabelButton>
      </>}>
      <TextInput value={v} onChangeText={setV} autoCapitalize="characters" autoCorrect={false} maxLength={16}
        style={{ borderWidth: 1, borderColor: serialOk(val) || !val ? c.line : c.red, borderRadius: radius.sm, paddingHorizontal: 10, paddingVertical: 8,
          color: c.text, fontFamily: fonts.mono, fontSize: 15 }} />
      <Text style={{ color: serialOk(val) || !val ? c.dim : c.redTx, fontSize: 12 }}>{t('영문 대문자 · 숫자 · 하이픈, 1~16자')}</Text>
      <Note tone="warn" text={t('쓰는 동안(몇 초) PTZ 를 조작할 수 없습니다.')} />
    </QcDialog>
  );
}
