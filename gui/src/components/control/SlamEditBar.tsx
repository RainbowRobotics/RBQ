import { useEffect } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Segmented, Toggle } from '@/components/ui/controls';
import { useSettings } from '@/store/settings';
import { useSlamnav } from '@/lib/slamnav';
import { TASK_NODE_TYPES, linkState } from '@/lib/slamnavWire';
import {
  ADD_TYPES, setSlamTool, slamAnnotSave, slamClearSelection, slamDeleteNode, slamGoTo, slamSetType, slamStartMove,
  slamTaskAdd, slamTaskCancel, slamTaskClear, slamTaskPauseResume, slamTaskRemove, slamTaskStart, slamToggleLink, slamUndo,
  useSlamEdit, type SlamTool,
} from '@/lib/slamEdit';
import { t } from '@/lib/i18n';

const ERROR_SHOW_MS = 10000;

export function useSlamEditAllowed(): boolean {
  const level = useSettings((s) => s.accessLevel);
  const ok = useSlamnav((s) => !!s.state && s.state.flags.mapLoaded && !s.state.flags.slam);
  return level >= 2 && ok;
}

function Btn({ label, onPress, active, disabled, danger }: {
  label: string; onPress: () => void; active?: boolean; disabled?: boolean; danger?: boolean;
}) {
  const { c, radius } = useTheme();
  return (
    <Tappable disabled={disabled} onPress={onPress}
      style={[styles.btn, {
        borderRadius: radius.sm, opacity: disabled ? 0.4 : 1,
        backgroundColor: active ? 'rgba(77,156,245,0.12)' : c.elev,
        borderColor: danger ? c.dangerLine : active ? 'rgba(77,156,245,0.5)' : c.line,
      }]}>
      <Text style={{ color: danger ? c.redTx : active ? c.accent2 : c.text, fontSize: 10.5, fontWeight: '600' }}>{label}</Text>
    </Tappable>
  );
}

export function SlamEditBar() {
  const { c, radius } = useTheme();
  const allowed = useSlamEditAllowed();
  const ed = useSlamEdit();
  const st = useSlamnav((s) => s.state);
  const topo = useSlamnav((s) => s.topo);
  const lastError = useSlamnav((s) => s.lastError);

  useEffect(() => { if (!allowed && ed.tool !== 'view') setSlamTool('view'); }, [allowed, ed.tool]);
  if (!allowed || !st) return null;

  const nodes = topo ?? [];
  const label = (id: string) => { const n = nodes.find((m) => m.id === id); return n?.name || id; };
  const sel = ed.selected ? nodes.find((n) => n.id === ed.selected) : undefined;
  const ready = st.mode === 'ANNOTATION';
  const paused = st.ctrl.state === 'pause';
  const showError = !!lastError && Date.now() - lastError.at < ERROR_SHOW_MS;
  const hint = ed.moving ? t('옮길 자리를 탭하세요 — 길게 눌러 끌면 방향')
    : ed.tool === 'edit' ? (ready ? t('빈 곳 탭 = 노드 추가 · 길게 눌러 끌면 방향 · 노드 탭 = 선택') : t('편집 모드로 바꾸는 중…'))
    : ed.tool === 'route' ? t('노드를 탭해 보내거나 경로에 넣으세요')
    : null;
  const link = sel && ed.prev ? linkState(nodes, ed.prev, sel.id) : null;

  return (
    <View style={[styles.card, { backgroundColor: c.glass, borderColor: c.glassLine, borderRadius: radius.md }]}>
      <View style={styles.row}>
        <View style={{ width: 210 }}>
          <Segmented<SlamTool> value={ed.tool} onChange={setSlamTool}
            options={[{ key: 'view', label: t('둘러보기') }, { key: 'edit', label: t('편집') }, { key: 'route', label: t('경로') }]} />
        </View>
        {ed.tool === 'edit' && (
          <>
            <Btn label={t('↶ 되돌리기')} disabled={!ed.undo.length} onPress={slamUndo} />
            <Btn label={t('저장')} disabled={!ready} onPress={() => { slamAnnotSave(); setSlamTool('view'); }} />
          </>
        )}
      </View>

      {ed.tool === 'edit' && ready && !ed.moving && (
        <View style={styles.row}>
          <View style={{ width: 260 }}>
            <Segmented value={ed.addType} onChange={(v) => useSlamEdit.setState({ addType: v })}
              options={ADD_TYPES.map((k) => ({ key: k, label: k }))} />
          </View>
          <Text style={{ color: c.text, fontSize: 10.5 }}>{t('앞 노드와 잇기')}</Text>
          <Toggle value={ed.autoLink} onChange={(v) => useSlamEdit.setState({ autoLink: v })} />
        </View>
      )}

      {ed.tool === 'route' && (
        <>
          <View style={[styles.row, { flexWrap: 'wrap' }]}>
            <Text style={{ color: c.dim, fontSize: 10 }}>{t('경로')}</Text>
            {st.task.nodes.length === 0
              ? <Text style={{ color: c.dim, fontSize: 10 }}>{t('비어 있음 — GOAL·BASE 노드를 넣으세요')}</Text>
              : st.task.nodes.map((id, i) => (
                <Tappable key={`${id}-${i}`} disabled={st.flags.tasking} onPress={() => slamTaskRemove(id)}
                  style={[styles.chip, { borderColor: id === st.ctrl.goal ? c.accent2 : c.line, borderRadius: radius.sm }]}>
                  <Text style={{ color: c.text, fontSize: 10 }}>{i + 1}. {label(id)}{st.flags.tasking ? '' : '  ×'}</Text>
                </Tappable>
              ))}
          </View>
          <View style={styles.row}>
            <Btn label={t('출발')} active={st.flags.tasking} disabled={!st.task.nodes.length || st.flags.tasking} onPress={slamTaskStart} />
            <Btn label={paused ? t('재개') : t('일시정지')} disabled={!st.flags.tasking && !paused} onPress={slamTaskPauseResume} />
            <Btn label={t('취소')} disabled={!st.flags.tasking && st.ctrl.state === 'stop'} onPress={slamTaskCancel} />
            <Btn label={t('비우기')} disabled={!st.task.nodes.length || st.flags.tasking} onPress={slamTaskClear} />
          </View>
          {(st.flags.tasking || st.ctrl.state !== 'stop') && (
            <Text style={{ color: c.accent2, fontSize: 10 }} numberOfLines={1}>
              {st.ctrl.goal ? `${t('목표')} ${label(st.ctrl.goal)} · ` : ''}{st.task.phase || st.ctrl.fsm}
            </Text>
          )}
        </>
      )}

      {sel && !ed.moving && (
        <View style={[styles.sheet, { borderColor: c.line, borderRadius: radius.sm }]}>
          <View style={styles.row}>
            <Text style={{ color: c.text, fontSize: 11, fontWeight: '700' }}>{sel.name || sel.id}</Text>
            <Text style={{ color: c.dim, fontSize: 10 }}>{sel.type}</Text>
            <View style={{ flex: 1 }} />
            <Btn label="✕" onPress={slamClearSelection} />
          </View>
          {ed.tool === 'edit' ? (
            <>
              {(ADD_TYPES as readonly string[]).includes(sel.type) && (
                <View style={{ width: 260 }}>
                  <Segmented value={sel.type} onChange={(v) => slamSetType(sel.id, v)} options={ADD_TYPES.map((k) => ({ key: k, label: k }))} />
                </View>
              )}
              <View style={styles.row}>
                <Btn label={t('이동')} onPress={() => slamStartMove(sel.id)} />
                {ed.prev && link && (
                  <Btn label={(link.ab || link.ba ? t('↔ {n} 끊기') : t('↔ {n} 연결')).replace('{n}', label(ed.prev))}
                    onPress={() => slamToggleLink(ed.prev!, sel.id)} />
                )}
                <Btn label={t('삭제')} danger onPress={() => slamDeleteNode(sel.id)} />
              </View>
            </>
          ) : (
            <View style={styles.row}>
              <Btn label={t('여기로 가기')} disabled={st.flags.tasking} onPress={() => slamGoTo(sel.id)} />
              <Btn label={t('경로에 추가')} disabled={st.flags.tasking || !TASK_NODE_TYPES.includes(sel.type)} onPress={() => slamTaskAdd(sel.id)} />
              {!TASK_NODE_TYPES.includes(sel.type) && <Text style={{ color: c.dim, fontSize: 9 }}>{t('경로엔 GOAL·BASE 만')}</Text>}
            </View>
          )}
        </View>
      )}

      {ed.moving && (
        <View style={styles.row}><Btn label={t('이동 취소')} onPress={() => useSlamEdit.setState({ moving: undefined })} /></View>
      )}
      {hint && <Text style={{ color: c.muted, fontSize: 9.5 }}>{hint}</Text>}
      {showError && ed.tool !== 'view' && (
        <Text style={{ color: c.redTx, fontSize: 9.5 }} numberOfLines={2}>{t('명령 실패')}: {lastError?.error}</Text>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  card: { padding: 8, gap: 6, borderWidth: 1, maxWidth: 520 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 6 },
  sheet: { borderWidth: 1, padding: 6, gap: 6 },
  btn: { minHeight: 28, paddingHorizontal: 10, paddingVertical: 4, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  chip: { borderWidth: 1, paddingHorizontal: 7, paddingVertical: 3 },
});
