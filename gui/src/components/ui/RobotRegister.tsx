import { useEffect, useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { useAccount, MIN_REGISTER_LEVEL } from '@/store/account';
import { useRobot } from '@/store/robot';
import { SB_URL, SB_KEY } from '@/lib/logUploadCommon';
import { registerRobot } from '@/lib/remoteLogin';
import { validateDraft, registerValues } from '@/lib/robotRegister';
import { type IniSection, applyRendezvousSection, readRendezvousKey } from '@/lib/robotRendezvousConfig';
import { currentTarget } from '@/lib/connectNow';
import { Modal } from '@/components/ui/overlays';
import { t as tr } from '@/lib/i18n';
import { rest, actions, HttpError } from '@/lib/rest';
import { SBtn } from '@/components/panels/settings/common';

async function serverReachable(): Promise<boolean> {
  if (!SB_URL) return false;
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), 3000);
  try { await fetch(`${SB_URL}/rest/v1/`, { headers: { apikey: SB_KEY }, signal: ctl.signal }); return true; }
  catch { return false; } finally { clearTimeout(timer); }
}

export function useCanOfferRegister(): boolean {
  const level = useAccount((s) => s.account?.level ?? null);
  const pin = useAccount((s) => s.pin);
  const conn = useRobot((s) => s.conn);
  const via = useRobot((s) => s.via);
  const base = level !== null && level >= MIN_REGISTER_LEVEL && !!pin && conn === 'connected' && via !== 'rendezvous'
    && currentTarget()?.route === 'direct';
  const [online, setOnline] = useState(false);
  useEffect(() => {
    if (!base) { setOnline(false); return; }
    let alive = true;
    void serverReachable().then((ok) => { if (alive) setOnline(ok); });
    return () => { alive = false; };
  }, [base]);
  return base && online;
}

type Pre = { kind: 'checking' } | { kind: 'busy_other' } | { kind: 'ready'; already: null | 'account' | 'robot'; robotId?: string };

export function RobotRegisterDialog({ serial, onClose }: { serial: string; onClose: () => void }) {
  const t = tr;
  const { c, fonts } = useTheme();
  const account = useAccount((s) => s.account);
  const accountRobots = useAccount((s) => s.robots);
  const pin = useAccount((s) => s.pin);
  const ip = useRobot((s) => s.ip);
  const isMine = useRobot((s) => s.isMine);
  const level = account?.level ?? null;
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<{ tone: 'err' | 'ok'; text: string } | null>(null);
  const [needRestart, setNeedRestart] = useState(false);
  const [pre, setPre] = useState<Pre>({ kind: 'checking' });

  useEffect(() => {
    let alive = true;
    (async () => {
      if (!isMine) { setPre({ kind: 'busy_other' }); return; }
      if (accountRobots.some((r) => r.robotSerial === serial)) { setPre({ kind: 'ready', already: 'account' }); return; }
      try {
        const cur = (await rest.webrtcSetting(ip)).ini.sections;
        const enabled = readRendezvousKey(cur, 'enabled') === 'true';
        const rid = readRendezvousKey(cur, 'robot_id');
        if (alive) setPre({ kind: 'ready', already: enabled && rid ? 'robot' : null, robotId: rid || undefined });
      } catch { if (alive) setPre({ kind: 'ready', already: null }); }
    })();
    return () => { alive = false; };
  }, [isMine, accountRobots, serial, ip]);

  const register = async () => {
    setBusy(true); setMsg(null);
    try {
      const sn = (await rest.serialNumber(ip)).serial_number?.trim() ?? '';
      const draft = { serial: sn, name: sn, lanIp: ip, rendezvousUrl: '' };
      const v = validateDraft(draft);
      if (!v.ok) {
        setBusy(false);
        setMsg({ tone: 'err', text: v.reason === 'serial_empty'
          ? t('로봇이 시리얼을 알려주지 않습니다 — 펌웨어를 확인해 주세요')
          : t('등록할 값이 올바르지 않습니다') });
        return;
      }
      const val = registerValues(draft);

      const r = await registerRobot(pin, val, { url: SB_URL, anonKey: SB_KEY });
      if (!r.ok) {
        setBusy(false);
        setMsg({ tone: 'err', text: r.reason === 'denied'
          ? t('서버가 등록을 거부했습니다 — 코드가 유효하고 레벨 3인지 확인해 주세요')
          : r.reason === 'offline'
            ? t('인터넷이 필요합니다 — 로봇망에는 인터넷이 없을 수 있습니다. 인터넷 되는 망에서 다시 시도해 주세요')
            : t('서버 오류 — 잠시 후 다시 시도하세요') });
        return;
      }

      const patch = {
        enabled: true,
        robotId: val.robotId,
        ...(r.rendezvousUrl ? { url: r.rendezvousUrl } : {}),
        ...(r.rendezvousToken ? { rendezvousToken: r.rendezvousToken } : {}),
      };
      let cur: IniSection[];
      try {
        cur = (await rest.webrtcSetting(ip)).ini.sections;
      } catch (e) {
        if (!(e instanceof HttpError) || e.status !== 404) throw e;
        setBusy(false);
        setMsg({ tone: 'err', text: t('로봇 펌웨어를 업데이트해 주세요 — 이 등록 방식(WebRTC.ini)은 새 펌웨어부터 지원됩니다') });
        return;
      }
      await actions.setWebrtcSetting(ip, applyRendezvousSection(cur, patch));

      setBusy(false);
      setNeedRestart(true);
      setMsg({ tone: 'ok', text: `${val.serial} — ${t('등록됐습니다. 로봇을 다시 시작해야 적용됩니다.')}` });
    } catch (e) {
      setBusy(false);
      setMsg({ tone: 'err', text: `${t('로봇 등록 실패')} — ${e instanceof Error ? e.message : String(e)}` });
    }
  };

  const reboot = async () => {
    setBusy(true);
    try {
      await actions.reboot(useRobot.getState().ip);
      setMsg({ tone: 'ok', text: t('로봇을 재시작하는 중입니다 — 잠시 뒤 다시 연결해 주세요') });
      setNeedRestart(false);
    } catch (e) {
      setMsg({ tone: 'err', text: `${t('재시작 요청 실패 — 로봇에서 직접 껐다 켜 주세요')} (${e instanceof Error ? e.message : String(e)})` });
    } finally {
      setBusy(false);
    }
  };

  return (
    <Modal onClose={busy ? () => {} : onClose}>
      <View style={{ width: 380, maxWidth: '100%', borderWidth: 1, borderColor: c.line, borderRadius: 14, backgroundColor: c.panel, overflow: 'hidden' }}>
        <View style={{ paddingHorizontal: 14, paddingTop: 13, paddingBottom: 11 }}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('원격 등록')}</Text>
          <Text style={{ color: c.dim, fontSize: 11, lineHeight: 16, marginTop: 3 }}>
            {t('이 로봇을 서버에 올려 밖에서도 접속할 수 있게 합니다.')}
          </Text>
        </View>
        <View style={{ height: 1, backgroundColor: c.line2 }} />
        <View style={{ paddingHorizontal: 14, paddingVertical: 12, gap: 10 }}>
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
            <Text style={{ color: c.muted, fontSize: 11, width: 52 }}>{t('대상')}</Text>
            <Text style={{ color: c.text, fontFamily: fonts.mono, fontSize: 12 }}>{serial} · {ip}</Text>
          </View>
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
            <Text style={{ color: c.muted, fontSize: 11, width: 52 }}>{t('계정')}</Text>
            <Text style={{ color: c.text, fontSize: 12 }}>{account?.accountName}</Text>
            <Text style={{ color: c.dim, fontSize: 10, fontWeight: '700' }}>L{level}</Text>
          </View>
          {pre.kind === 'checking' && <Text style={{ color: c.muted, fontSize: 11.5 }}>{t('확인 중…')}</Text>}
          {pre.kind === 'busy_other' && (
            <Text style={{ color: c.amberTx, fontSize: 11.5, lineHeight: 17 }}>
              {t('다른 기기가 이 로봇을 제어 중입니다 — 등록하면 로봇을 다시 시작해야 해서 그 제어가 끊깁니다. 제어가 끝난 뒤 등록하세요.')}
            </Text>
          )}
          {pre.kind === 'ready' && pre.already && (
            <Text style={{ color: c.amberTx, fontSize: 11.5, lineHeight: 17 }}>
              {pre.already === 'account'
                ? t('이미 계정에 등록된 로봇입니다. 밖에서 안 붙을 때만 설정을 다시 써 주세요.')
                : `${t('이 로봇에는 이미 원격 설정이 있습니다')} (robot_id ${pre.robotId}). ${t('다른 계정에 등록된 로봇일 수 있습니다.')}`}
            </Text>
          )}
          {pre.kind === 'ready' && !needRestart && (
            <SBtn kind={pre.already ? 'ghost' : 'primary'} icon="plus"
              label={busy ? t('등록 중…') : pre.already ? t('다시 등록 (설정 복구)') : t('이 로봇 등록')}
              onPress={register} disabled={busy} />
          )}
        </View>
        {needRestart && (
          <>
            <View style={{ height: 1, backgroundColor: c.line2 }} />
            <View style={{ paddingHorizontal: 14, paddingVertical: 12, backgroundColor: c.elev, gap: 8 }}>
              <Text style={{ color: c.text, fontSize: 11.5, lineHeight: 17 }}>
                {t('로봇을 다시 시작해야 적용됩니다. 로봇이 인터넷에 연결되어 있어야 합니다.')}
              </Text>
              <SBtn kind="ghost" icon="power" label={t('로봇 재시작')} onPress={reboot} disabled={busy} />
            </View>
          </>
        )}
        {!!msg && (
          <>
            <View style={{ height: 1, backgroundColor: c.line2 }} />
            <Text style={{ color: msg.tone === 'err' ? c.redTx : c.greenTx, fontSize: 11.5, lineHeight: 17, paddingHorizontal: 14, paddingVertical: 10 }}>
              {msg.text}
            </Text>
          </>
        )}
        <View style={{ height: 1, backgroundColor: c.line2 }} />
        <View style={{ padding: 10, alignItems: 'flex-end' }}>
          <SBtn kind="ghost" icon="x" label={t('닫기')} onPress={onClose} disabled={busy} />
        </View>
      </View>
    </Modal>
  );
}