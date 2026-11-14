import { useCallback, useEffect, useState } from 'react';
import { View, Text, StyleSheet, ScrollView } from 'react-native';
import { cacheDirectory } from 'expo-file-system/legacy';
import { pickDocument } from '@/lib/pickDocument';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { notify } from '@/components/DoneToast';
import { useRobot } from '@/store/robot';
import { useNotMine } from '@/lib/spectating';
import { useKnownSystem } from '@/store/robots';
import { useFeatures } from '@/store/capability';
import { connection } from '@/lib/connection';
import { actions } from '@/lib/rest';
import { otaTransfer, otaRemoveOption, otaAvailable } from '@/lib/ota';
import { useOptionJob, optionRefs, type OptionRowFile } from '@/store/optionInstall';
import { useUsbJob } from '@/store/usbBundle';
import { LEGACY_SYSTEM } from '@/lib/firmwareRelease';
import { parseOptionFile, optionRowState, optionTitle, optionErrorText, listCachedOptions, importOptionFile, removeCachedOption,
  type OptionRow } from '@/lib/optionPackage';
import { t } from '@/lib/i18n';

const fmtMB = (b: number) => `${(b / 1048576).toFixed(1)} MB`;

type Row = OptionRowFile;

export function OptionPanel() {
  const { c, fonts, radius } = useTheme();
  const ip = useRobot((s) => s.ip);
  const conn = useRobot((s) => s.conn);
  const notMine = useNotMine();
  const vacant = useRobot((s) => !s.owner);
  const usbBusy = useUsbJob((s) => s.step != null);
  const features = useFeatures((s) => s.features);
  const sys = useKnownSystem('motion', ip);
  const system = sys && sys !== LEGACY_SYSTEM ? sys : '';
  const knows = typeof features?.option === 'string';
  const robot = { option: String(features?.option ?? ''), optionVersion: String(features?.option_version ?? ''), system };

  const [cached, setCached] = useState<Row[]>([]);
  const [picked, setPicked] = useOptionJob.use('picked');
  const [busy, setBusy] = useOptionJob.use('busy');
  const [prog, setProg] = useOptionJob.use('prog');
  const [err, setErr] = useOptionJob.use('err');
  const [removed, setRemoved] = useOptionJob.use('removed');
  const [installedName, setInstalledName] = useOptionJob.use('installed');
  const setDone = useOptionJob.setter('done');
  const setBusyTitle = useOptionJob.setter('busyTitle');
  const [ask, setAsk] = useState<{ kind: 'install'; row: Row } | { kind: 'remove' } | null>(null);
  const handle = optionRefs.handle;
  useOptionJob.useScreen();

  const load = useCallback(() => {
    listCachedOptions().then((list) => setCached(list.flatMap((f) => {
      const opt = parseOptionFile(f.name);
      return opt ? [{ file: { uri: `${cacheDirectory}${f.name}`, name: f.name, size: f.size }, opt, cached: true }] : [];
    }))).catch(() => {});
  }, []);
  useEffect(load, [load]);

  const rows = [...cached, ...(picked && !cached.some((r) => r.file.name === picked.file.name) ? [picked] : [])];

  const pick = async () => {
    setErr(null);
    const r = await pickDocument({ copyToCacheDirectory: true });
    const a = r.assets?.[0];
    if (r.canceled || !a) return;
    const opt = parseOptionFile(a.name);
    if (!opt) { setErr(`${a.name} — ${t('옵션 파일 이름이 아닙니다 (option-<이름>-<판>-<시스템>.rbo)')}`); return; }
    if (a.size == null) { setErr(t('파일 크기를 확인할 수 없습니다 — 다른 위치에서 선택해 보세요')); return; }
    try {
      const kept = await importOptionFile({ uri: a.uri, name: a.name, size: a.size });
      if (kept) load();
      else setPicked({ file: { uri: a.uri, name: a.name, size: a.size }, opt, cached: false });
    } catch (e) { setErr((e as Error).message); }
  };

  const install = (row: Row) => {
    if (handle.current || notMine || useUsbJob.getState().step) return;
    setAsk(null); setErr(null); setDone(null); setRemoved(null); setInstalledName(null); setBusyTitle(`${optionTitle(row.opt.name)} ${row.opt.label}`); setBusy(row.file.name); setProg(null);
    handle.current = otaTransfer(ip, row.file, setProg);
    handle.current.done
      .then(() => { const m = `${optionTitle(row.opt.name)} ${row.opt.label} — ${t('설치했습니다')}`; setDone(m); notify(m); setInstalledName(row.opt.name); })
      .catch((e: Error) => { if (e.message !== 'cancelled') setErr(optionErrorText(e.message)); })
      .finally(() => { handle.current = null; setBusy(null); setProg(null); connection.fetchFeatures(); });
  };

  const remove = () => {
    if (notMine || useUsbJob.getState().step) return;
    setAsk(null); setErr(null); setDone(null); setInstalledName(null); setBusy('remove');
    otaRemoveOption(ip)
      .then((r) => { const m = `${optionTitle(r.name)} — ${t('제거했습니다')}`; setDone(m); notify(m); setRemoved(r.name); })
      .catch((e: Error) => setErr(e.message))
      .finally(() => { setBusy(null); connection.fetchFeatures(); });
  };

  const forget = (row: Row) => {
    if (row.cached) removeCachedOption(row.file.name).then(load).catch((e: Error) => setErr(e.message));
    else setPicked(null);
  };

  const pill = (text: string, color: string) => (
    <Text style={{ color, fontSize: 9.5, fontWeight: '700', borderWidth: 1, borderColor: c.line, borderRadius: 5, paddingHorizontal: 5, paddingVertical: 1 }}>{text}</Text>
  );
  const btn = (label: string, onPress: () => void, kind: 'primary' | 'danger' | 'plain', disabled = false) => (
    <Tappable onPress={onPress} disabled={disabled} accessibilityLabel={label}
      style={[styles.action, { height: 26, borderRadius: radius.md, opacity: disabled ? 0.4 : 1,
        backgroundColor: kind === 'primary' ? c.accent : c.elev, borderColor: kind === 'primary' ? c.accent : c.line }]}>
      <Text style={{ color: kind === 'primary' ? c.onAccent : kind === 'danger' ? c.redTx : c.text, fontSize: 11.5, fontWeight: '600' }}>{label}</Text>
    </Tappable>
  );
  const canWrite = otaAvailable && knows && conn === 'connected' && !busy && !notMine && !usbBusy;

  const rowView = (row: Row) => {
    const state: OptionRow = optionRowState(row.opt, robot);
    const mine = busy === row.file.name;
    return (
      <View key={row.file.name} style={{ paddingVertical: 6, borderTopWidth: 1, borderTopColor: c.line, gap: 4 }}>
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, flexWrap: 'wrap' }}>
          <Text style={{ color: state === 'otherSystem' ? c.dim : c.text, fontSize: 13, fontWeight: '700' }}>{optionTitle(row.opt.name)}</Text>
          <Text style={{ color: c.muted, fontSize: 11.5, fontFamily: fonts.mono }}>{row.opt.label}</Text>
          <Text style={{ color: c.dim, fontSize: 10.5, fontFamily: fonts.mono }}>{row.opt.system} · {fmtMB(row.file.size)}</Text>
          {state === 'installed' ? pill(t('설치됨'), c.greenTx) : null}
          {state === 'update' ? pill(`${t('로봇')} ${robot.optionVersion}`, c.amberTx) : null}
          <View style={{ flex: 1 }} />
          {mine && prog ? (
            <Text style={{ color: c.accent2, fontSize: 11, fontFamily: fonts.mono }}>
              {prog.phase === 'transfer' ? `${prog.percentage}%` : prog.phase === 'applying' ? t('로봇에서 설치 중') : t('연결 중…')}
            </Text>
          ) : (
            <>
              {state === 'update' ? btn(t('업데이트'), () => setAsk({ kind: 'install', row }), 'primary', !canWrite) : null}
              {state === 'installable' ? btn(t('설치'), () => setAsk({ kind: 'install', row }), 'primary', !canWrite) : null}
              {btn(t('파일 삭제'), () => forget(row), 'plain', !!busy)}
            </>
          )}
        </View>
        {state === 'otherInstalled' ? (
          <Text style={{ color: c.amberTx, fontSize: 11.5 }}>
            {t('이 로봇에는 {x}이 깔려 있습니다 — 먼저 지우세요').replace('{x}', optionTitle(robot.option))}
          </Text>
        ) : null}
        {state === 'otherSystem' ? (
          <Text style={{ color: c.dim, fontSize: 11.5 }}>
            {t('다른 로봇용 파일입니다 — 이 로봇은 {s}').replace('{s}', robot.system)}
          </Text>
        ) : null}
      </View>
    );
  };

  return (
    <View style={{ flex: 1 }}>
      <ScrollView showsVerticalScrollIndicator={false}>
        <Text style={{ color: c.text, fontSize: 17, fontWeight: '700', marginBottom: 3 }}>{t('옵션 패키지')}</Text>
        <Text style={{ color: c.dim, fontSize: 12, marginBottom: 10, lineHeight: 16 }}>
          {t('고객별 프로그램·설정을 로봇 소프트웨어와 따로 깔고 지웁니다. 한 로봇에 옵션은 하나입니다.')}
        </Text>

        <View style={[styles.card, { borderColor: c.line, borderRadius: radius.md, backgroundColor: c.bg }]}>
          <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginBottom: 5 }}>{t('이 로봇')} · Motion ({ip})</Text>
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, flexWrap: 'wrap' }}>
            <Text style={{ color: c.muted, fontSize: 12 }}>{t('깔린 옵션')}</Text>
            <Text style={{ color: robot.option ? c.greenTx : c.text, fontSize: 12, fontFamily: fonts.mono }}>
              {!features ? '—' : !knows ? '—' : robot.option ? `${optionTitle(robot.option)} ${robot.optionVersion}` : t('없음')}
            </Text>
            <View style={{ flex: 1 }} />
            {knows && robot.option ? btn(t('제거'), () => setAsk({ kind: 'remove' }), 'danger', !canWrite) : null}
            {busy === 'remove' ? <Text style={{ color: c.accent2, fontSize: 11 }}>{t('제거하는 중…')}</Text> : null}
          </View>
          {conn !== 'connected' ? (
            <Text style={{ color: c.dim, fontSize: 11.5, marginTop: 4 }}>{t('로봇에 연결하면 깔린 옵션이 보입니다.')}</Text>
          ) : features && !knows ? (
            <Text style={{ color: c.amberTx, fontSize: 11.5, marginTop: 4, lineHeight: 15 }}>
              {t('이 로봇 소프트웨어는 옵션 패키지를 모릅니다 — 소프트웨어 업데이트를 먼저 하세요.')}
            </Text>
          ) : null}
          {(removed && !robot.option) || (installedName && robot.option === installedName) ? (
            <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, marginTop: 6, flexWrap: 'wrap' }}>
              <Text style={{ color: c.amberTx, fontSize: 11.5, lineHeight: 15, flex: 1 }}>
                {removed && !robot.option
                  ? t('동작 제한은 바로 풀렸습니다. 옵션이 넘긴 값(탑재물 무게 등)은 로봇 소프트웨어를 다시 시작해야 기본값으로 돌아갑니다. 로봇을 앉힌 뒤 다시 시작하세요.')
                  : t('동작 제한은 바로 걸렸습니다. 옵션의 나머지 설정(통신 확인 주소 등)까지 맞추려면 로봇을 앉힌 뒤 로봇 소프트웨어를 다시 시작하세요.')}
              </Text>
              {btn(t('로봇 서비스 재시작'), () => {
                actions.restart(ip).then(() => { setRemoved(null); setInstalledName(null); notify(t('로봇을 재시작하는 중입니다 — 잠시 뒤 다시 연결해 주세요')); },
                  (e) => notify(`${t('재시작 요청 실패')}: ${e?.message ?? e}`, 'warn'));
              }, 'plain', conn !== 'connected' || notMine)}
            </View>
          ) : null}
        </View>

        <View style={[styles.card, { borderColor: c.line, borderRadius: radius.md, backgroundColor: c.bg, marginTop: 10 }]}>
          <View style={{ flexDirection: 'row', alignItems: 'center', marginBottom: 4 }}>
            <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', flex: 1 }}>{t('받아 둔 옵션 파일')}</Text>
            {otaAvailable ? (
              <Tappable onPress={pick} disabled={!!busy} accessibilityLabel={t('파일 가져오기')}
                style={[styles.action, { height: 28, borderRadius: radius.md, backgroundColor: c.elev, borderColor: c.line, opacity: busy ? 0.4 : 1 }]}>
                <Icon name="upload" size={12} color={c.muted} />
                <Text style={{ color: c.text, fontSize: 11.5, fontWeight: '600' }}>{t('파일 가져오기')}</Text>
              </Tappable>
            ) : null}
          </View>
          {rows.length === 0 ? (
            <Text style={{ color: c.dim, fontSize: 11.5, lineHeight: 15, paddingVertical: 4 }}>
              {t('받아 둔 옵션 파일이 없습니다 — USB·메신저로 받은 옵션 파일을 [파일 가져오기]로 고르세요.')}
            </Text>
          ) : rows.map(rowView)}
        </View>

        {notMine ? <Text style={{ color: c.amberTx, fontSize: 11.5, marginTop: 8 }}>{vacant
          ? t('조종하는 기기가 없습니다 — 파일 가져오기는 되고, 설치·제거는 제어권을 가져온 뒤에 됩니다')
          : t('다른 기기가 조종 중입니다 — 파일 가져오기는 되고, 설치·제거는 제어권을 가져온 뒤에 됩니다')}</Text> : null}
        {usbBusy ? <Text style={{ color: c.amberTx, fontSize: 11.5, marginTop: 8 }}>{t('USB 묶음 설치가 도는 중입니다 — 끝난 뒤 다시 하세요')}</Text> : null}
        {err ? <Text style={{ color: c.redTx, fontSize: 12, marginTop: 8 }}>{t('오류:')} {err}</Text> : null}
        <Text style={{ color: c.dim, fontSize: 10.5, marginTop: 8, lineHeight: 14 }}>
          {t('옵션 파일은 레인보우로보틱스 서명이 있어야 깔립니다. 다른 옵션으로 바꾸려면 깔린 것을 먼저 지우세요.')}
        </Text>
      </ScrollView>

      {ask?.kind === 'install' ? (
        <ConfirmModal danger={false}
          title={`${optionTitle(ask.row.opt.name)} ${ask.row.opt.label}`}
          message={t('로봇에 이 옵션을 깝니다 — 옵션 프로그램과 동작 제한이 바로 켜집니다. 로봇 소프트웨어는 다시 시작하지 않습니다.')}
          confirmLabel={optionRowState(ask.row.opt, robot) === 'update' ? t('업데이트') : t('설치')}
          onConfirm={() => install(ask.row)} onClose={() => setAsk(null)} />
      ) : null}
      {ask?.kind === 'remove' ? (
        <ConfirmModal
          title={`${optionTitle(robot.option)} ${t('제거')}`}
          message={t('옵션 프로그램이 바로 멈추고 동작 제한도 풀립니다. 옵션이 넘긴 값(탑재물 무게 등)은 로봇 소프트웨어를 다시 시작하면 기본값으로 돌아갑니다.')}
          confirmLabel={t('제거')}
          onConfirm={remove} onClose={() => setAsk(null)} />
      ) : null}
    </View>
  );
}

const styles = StyleSheet.create({
  action: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 12, borderWidth: 1 },
  card: { paddingHorizontal: 14, paddingVertical: 11, borderWidth: 1 },
});
