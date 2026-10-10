import { describe, it, expect } from 'vitest';
import {
  parseFwStatus, summarize, fwCommonGuard, fwAllGuard, fwBanners, fwAsk, sameAsk, fwConfirmLines, fwRunRows, fwRunResult,
  fwPostError, fwPowerLeft, fwReadyText, boardListText, shortNames, neverChecked, boardUpdatable, bootRecover, variantFix,
  modeLabel, reachable, runKindLabel, isMotorSlot, fwTargets, motorGroup, motorsUpdatable, motorVerdictLabel,
  type FwGuardInput, type FwStatus, type FwTarget,
} from './boardFirmware';
import { demoFwStatus, demoFwLiveRun, demoFwLiveCheck, demoHwVer, FW_DEMO_SCENARIOS } from './boardFirmwareDemo';

const st = (scn: string): FwStatus => {
  const s = parseFwStatus(demoFwStatus(scn));
  if (!s) throw new Error(`parse failed: ${scn}`);
  return s;
};
const input = (status: FwStatus | null, over: Partial<FwGuardInput> = {}): FwGuardInput => ({
  connected: true, featureOn: true, status, gaitId: 0, controlOn: false, otherOwner: false, ...over,
});
const asTransport = (path: string, status: number, body: Record<string, unknown>) => {
  const why = (body.reason as unknown) || body.error;
  return Object.assign(new Error(`${path} → ${status}${why ? ` (${String(why)})` : ''}`), { status });
};
const mk = (over: (b: Record<string, unknown>) => Record<string, unknown>, run: Record<string, unknown> = { id: 1, kind: 'none', state: 'none', stage: 'none', board: -1, reason: 0 }) =>
  parseFwStatus({ supported: true, check_seq: 3, run, boards: Array.from({ length: 21 }, (_, slot) => over({
    slot, present: true, mode: 'app', verdict: 'latest', result: 'none', reason: 0,
    hw: { state: slot < 16 ? 'old_firmware' : 'ok', ver: slot < 16 ? null : demoHwVer(1) },
  })) })!;

describe('parseFwStatus — 모르는 값은 보수적으로', () => {
  it('모양이 아니면 null', () => {
    expect(parseFwStatus(null)).toBeNull();
    expect(parseFwStatus({ status: 'ok' })).toBeNull();
    expect(parseFwStatus('x')).toBeNull();
  });
  it('모르는 문자열은 확인 전·모름, supported 가 없으면 지원 안 함', () => {
    const s = parseFwStatus({ boards: [{ slot: 20, present: true, mode: 'weird', verdict: 'new_value', hw: { state: '??' } }] })!;
    expect(s.supported).toBe(false);
    expect(s.boards[0]).toMatchObject({ name: 'TOP', mode: 'unknown', verdict: 'unchecked', hw: { state: 'unchecked', ver: null } });
    expect(s.run).toMatchObject({ kind: 'none', state: 'none', board: -1 });
  });
  it('busy — 없으면 쉼, 아는 값은 그대로, 모르는 값은 업데이트 중(막힘)', () => {
    expect(parseFwStatus({ boards: [] })!.busy).toBe('none');
    expect(parseFwStatus({ boards: [], busy: 'check' })!.busy).toBe('check');
    expect(parseFwStatus({ boards: [], busy: 'mystery' })!.busy).toBe('update');
  });
  it('판정 unmanaged 를 읽는다', () => {
    expect(parseFwStatus({ boards: [{ slot: 0, verdict: 'unmanaged' }] })!.boards[0].verdict).toBe('unmanaged');
  });
  it('null 값과 file:null 을 받아도 죽지 않고, 칸 순서로 정렬한다', () => {
    const s = parseFwStatus({ supported: true, check_seq: 4, boards: [
      { slot: 17, name: 'IF', present: true, app_version: null, file: null, hw: { state: 'ok', ver: demoHwVer(2), remembered: true } },
      { slot: 16, name: 'PDU', present: true, app_version: 261003, file: { name: 'a.bin', version: 261004, a: 1 } },
    ] })!;
    expect(s.boards.map((b) => b.slot)).toEqual([16, 17]);
    expect(s.boards[1]).toMatchObject({ appVersion: null, file: { name: null, version: null, a: null }, hw: { ver: demoHwVer(2), remembered: true } });
    expect(s.boards[0].file.version).toBe(261004);
  });
  it('데모 시나리오는 전부 21칸으로 읽힌다(로봇과 같은 모양)', () => {
    const extra = ['motor_rejected', 'failed:10', 'failed:12', 'failed:13', 'failed:14', 'failed:9', 'failed:0', 'rejected:13', 'rejected:7', 'rejected:5', 'rejected:6', 'changed:after'];
    for (const scn of [...FW_DEMO_SCENARIOS, ...extra]) {
      const s = st(scn);
      expect(s.boards, scn).toHaveLength(21);
      expect(s.boards.every((b, i) => b.slot === i), scn).toBe(true);
    }
  });
  it('데모 파일 이름에 사내 라벨 접두어가 없다(공개 스냅샷 이름 규칙)', () => {
    for (const scn of FW_DEMO_SCENARIOS) {
      for (const b of st(scn).boards) expect(b.file.name ?? '', `${scn} ${b.name}`).not.toMatch(/\bRBQ_[A-Z]/);
    }
  });
});

describe('summarize', () => {
  it('업데이트 가능에는 다운그레이드도 들어간다 · 모터는 묶음 하나로 센다', () => {
    const sm = summarize(st('update'));
    expect(sm.targets.map((b) => b.name)).toEqual(['PDU', 'IF', 'SIDE_H', 'TOP']);
    expect(sm.motors).toMatchObject({ verdict: 'update', mixed: false });
    expect(sm.motors.todo).toHaveLength(16);
    expect(sm).toMatchObject({ motorsGo: true, updatable: 5, latest: 1, motorsLater: false });
    expect(sm.needCheck).toEqual([]);
    expect(fwTargets('all', st('update'))).toHaveLength(20);
  });
  it('부트에 남아 hw 를 모르는 보드는 확인 필요가 아니라 대상 — 로봇이 앱으로 보내 확인한 뒤 올린다', () => {
    const s = st('boot');
    const iF = s.boards[17];
    expect(bootRecover(iF)).toBe(true);
    expect(boardUpdatable(iF)).toBe(true);
    expect(summarize(s)).toMatchObject({ needCheck: [], targets: [iF] });
    expect(fwReadyText(summarize(s))).toBe('올릴 보드 — IF · IF는 부트에 남아 있어 앱으로 보내 확인한 뒤 올립니다.');
  });
  it('대상 아님(모터) — 막지 않고 건너뛴다, 묶음 [업데이트] 도 없다', () => {
    const sm = summarize(st('unmanaged'));
    expect(sm).toMatchObject({ unmanaged: 1, motorsGo: false, updatable: 4 });
    expect(sm.motors.verdict).toBe('unmanaged');
    expect(motorsUpdatable(sm.motors)).toBe(false);
    expect(fwAsk('motors', st('unmanaged'), false)).toBeNull();
  });
  it('다리가 꺼져 모터가 전부 응답 없음 — 모터는 업데이트 때 확인한다', () => {
    const sm = summarize(st('legs_off'));
    expect(sm).toMatchObject({ motorsLater: true, targets: [], noResponse: 0, updatable: 0 });
    expect(sm.motors.verdict).toBe('legs_off');
    expect(motorsUpdatable(sm.motors)).toBe(true);
    expect(fwReadyText(sm)).toBe('다리 전원이 꺼져 있어 모터는 업데이트하면서 다리 전원을 켜고 확인합니다');
  });
  it('확인 전', () => {
    expect(neverChecked(st('unchecked'))).toBe(true);
    expect(neverChecked(st('update'))).toBe(false);
  });
});

describe('모터 묶음(계약 7-1)', () => {
  it('모두 최신 · 모두 옛 버전 · 다운그레이드 · 대상 아님 · 다리 꺼짐', () => {
    expect(motorGroup(st('latest'))).toMatchObject({ verdict: 'latest', mixed: false, todo: [] });
    expect(motorGroup(st('update'))).toMatchObject({ verdict: 'update', mixed: false });
    expect(motorGroup(st('motor_downgrade'))).toMatchObject({ verdict: 'downgrade', mixed: false });
    expect(motorGroup(st('unmanaged')).verdict).toBe('unmanaged');
    expect(motorGroup(st('legs_off')).verdict).toBe('legs_off');
    expect(motorVerdictLabel(motorGroup(st('legs_off')))).toBe('다리 꺼짐');
  });
  it('섞임 — 판정이나 앱 버전이 다르면. 같은 버전인데 내용이 다른 모터(same)도 굽는다', () => {
    const g = motorGroup(st('mixed'));
    expect(g).toMatchObject({ verdict: 'update', mixed: true });
    expect(g.todo.map((m) => m.name)).toEqual(['HRK', 'FLW']);
    expect(fwBanners(st('mixed')).mixedMotors).toBe(true);
    const um = mk((b) => (isMotorSlot(b.slot as number) ? { ...b, verdict: 'unmanaged', app_version: b.slot === 3 ? 261001 : 261004 } : b));
    expect(motorGroup(um)).toMatchObject({ verdict: 'unmanaged', mixed: true });
    expect(fwBanners(um).mixedMotors).toBe(false);
  });
  it('확인창 — 모터는 한 항목, 내려가는 모터 · 다시 굽는 모터를 한 줄씩', () => {
    expect(fwConfirmLines(fwAsk('motors', st('mixed'), false)!)).toEqual([
      '다음 보드를 업데이트합니다: 모터(2개)',
      '모터 1개는 버전이 같아도 파일과 내용이 달라 다시 굽습니다 — 모터는 모두 같은 펌웨어여야 합니다.',
      '모터를 올리는 동안 다리 전원을 켰다가 끝나면 끕니다. 끝나도 다리 전원은 꺼진 채로 남으니 다시 기동하세요.',
    ]);
    expect(fwConfirmLines(fwAsk('all', st('motor_downgrade'), false)!)).toContain('모터 16개가 보드(261005)보다 낮은 버전(261004)으로 내려갑니다.');
  });
  it('묶음 거절은 칸 이름이 아니라 모터 묶음으로 쓴다 · 끝난 뒤 다리 전원', () => {
    expect(fwRunResult(st('motor_rejected'))).toEqual({ ok: false, title: '업데이트를 시작하지 못했습니다 — 모터',
      text: '모터 중에 낮은 버전으로 내려가는 모터가 있어 동의가 필요합니다. 다리 전원을 켠 채 [버전 확인]을 누른 뒤 확인창에서 동의하세요.',
      power: '다리 전원은 꺼진 채입니다 — 다시 기동하세요.' });
    const r7 = mk((b) => b, { id: 5, kind: 'board', state: 'rejected', stage: 'leg_off', board: 0, reason: 7 });
    expect(fwRunResult(r7)?.text).toBe('모터가 모두 이미 최신이라 굽지 않았습니다.');
    expect(fwPostError(asTransport('/api/firmware/update', 400, { error: 'motors are updated together — use board "motors"' })).text)
      .toBe('모터는 묶음으로만 업데이트합니다 — 모터 줄의 [업데이트]를 쓰세요');
  });
  it('부트에 남은 모터는 섞임 배너가 맡는다(업데이트 배너에 겹쳐 쓰지 않는다)', () => {
    const s = mk((b) => (b.slot === 4 ? { ...b, mode: 'boot', present: false, verdict: 'update', app_version: null } : b));
    expect(fwBanners(s)).toMatchObject({ mixedMotors: true, update: [] });
  });
});

describe('막힌 이유 — 로봇과 같은 식(6-2)', () => {
  const s = st('update');
  const MOVING = '구동 중입니다 — 로봇을 앉힌 뒤 다시 시도하세요';
  it('연결 → 기능 → 상태 → CAN-FD 순', () => {
    expect(fwAllGuard(input(s, { connected: false, featureOn: false })).reason).toContain('연결되어 있지 않습니다');
    expect(fwAllGuard(input(s, { featureOn: false })).reason).toContain('소프트웨어 업데이트를 먼저');
    expect(fwAllGuard(input(null)).reason).toContain('받는 중');
    expect(fwAllGuard(input(st('classic'))).reason).toContain('CAN-FD');
  });
  it('진행 중이면 진행 화면(running) — 다른 기기가 시작한 것도', () => {
    expect(fwAllGuard(input(st('running')))).toMatchObject({ blocked: true, running: true });
    expect(fwAllGuard(input(st('running'), { otherOwner: true }))).toMatchObject({ blocked: true, running: true });
  });
  it('다른 펌웨어 작업이 도는 중(busy=update) · 버전 확인 중(busy=check)', () => {
    expect(fwAllGuard(input(st('busy')))).toMatchObject({ blocked: true, running: false, reason: '로봇이 펌웨어 작업을 하는 중입니다 — 끝난 뒤 다시 시도하세요' });
    expect(fwAllGuard(input(st('checking'))).reason).toBe('버전을 확인하는 중입니다');
    expect(fwCommonGuard(input({ ...s, busy: 'check' })).reason).toBe('버전을 확인하는 중입니다');
  });
  it('소유권은 구동 중보다 먼저 — 로봇은 403 을 409 보다 먼저 본다', () => {
    expect(fwAllGuard(input(s, { otherOwner: true, gaitId: 3 })).reason).toBe('다른 기기가 조종 중입니다');
  });
  it('로봇 fwPrecheckReason 과 같은 표 — gait × 제어', () => {
    const cases: [number, boolean, 'moving' | 'consent' | 'ok'][] = [
      [-1, false, 'ok'], [0, false, 'ok'], [1, false, 'moving'], [3, false, 'moving'], [-2, false, 'ok'],
      [-1, true, 'moving'], [0, true, 'consent'], [1, true, 'moving'], [3, true, 'moving'], [-2, true, 'moving'],
    ];
    for (const [gaitId, controlOn, want] of cases) {
      const g = fwAllGuard(input(s, { gaitId, controlOn }));
      const got = g.blocked ? (g.reason === MOVING ? 'moving' : g.reason) : g.needPowerDown ? 'consent' : 'ok';
      expect(got, `gait ${gaitId} control ${controlOn}`).toBe(want);
    }
    expect(fwAllGuard(input(s, { gaitId: null })).blocked).toBe(true);
  });
  it('확인이 필요한 보드가 있으면 이름과 사유', () => {
    expect(fwAllGuard(input(st('check'))).reason).toBe('확인이 필요한 보드가 있습니다: SIDE_F — 로봇에 이 보드(HW2)에 맞는 파일이 없습니다');
    expect(fwAllGuard(input(st('standby'))).reason).toContain('TOP — hw 기록이 없어');
    expect(fwAllGuard(input(st('eeprom'))).reason).toContain('TOP — EEPROM이 응답하지 않아');
  });
  it('부트 보드 · 대상 아님 · 다리 꺼짐은 막지 않는다', () => {
    expect(fwAllGuard(input(st('boot'))).blocked).toBe(false);
    expect(fwAllGuard(input(st('unmanaged'))).blocked).toBe(false);
    expect(fwAllGuard(input(st('legs_off'))).blocked).toBe(false);
  });
  it('확인 전 · 올릴 것 없음', () => {
    expect(fwAllGuard(input(st('unchecked'))).reason).toContain('[버전 확인]');
    expect(fwAllGuard(input(st('latest'))).reason).toBe('업데이트할 보드가 없습니다');
  });
  it('보드 하나는 다른 보드의 확인 필요에 막히지 않는다', () => {
    expect(fwCommonGuard(input(st('check'))).blocked).toBe(false);
  });
});

describe('배너', () => {
  it('부트에 남은 보드 → 업데이트 필요(present 가 내려가도 닿는 보드다)', () => {
    const s = st('boot');
    expect(fwBanners(s).update.map((b) => b.name)).toEqual(['IF']);
    expect(s.boards[17]).toMatchObject({ present: false, mode: 'boot_no_app' });
    expect(reachable(s.boards[17])).toBe(true);
    expect(modeLabel(s.boards[17])).toBe('부트(앱 없음)');
    expect(modeLabel({ ...s.boards[17], mode: 'unknown' })).toBe('없음');
  });
  it('hw 기록이 없어 대기 → hw 기록 필요 / EEPROM 무응답으로 대기 → 따로', () => {
    expect(fwBanners(st('standby'))).toMatchObject({ update: [], noEeprom: [] });
    expect(fwBanners(st('standby')).hwRecord.map((x) => x.name)).toEqual(['TOP']);
    expect(fwBanners(st('eeprom'))).toMatchObject({ update: [], hwRecord: [] });
    expect(fwBanners(st('eeprom')).noEeprom.map((x) => x.name)).toEqual(['TOP']);
  });
  it('hw 는 아는데 지원 안 해 대기 → 업데이트 필요', () => {
    expect(fwBanners(st('failed:14')).update.map((b) => b.name)).toEqual(['TOP']);
    expect(fwBanners(st('variant')).update.map((b) => b.name)).toEqual(['IF']);
  });
  it('확인 전에 대기 중인 보드(hw 확인 전)는 아직 내지 않는다', () => {
    const s = mk((b) => (b.slot === 20 ? { ...b, mode: 'standby', hw: { state: 'unchecked' } } : b));
    expect(fwBanners(s)).toEqual({ update: [], mixedMotors: false, hwRecord: [], noEeprom: [] });
  });
  it('굽는 중 · 확인 중엔 내지 않는다', () => {
    const none = { update: [], mixedMotors: false, hwRecord: [], noEeprom: [] };
    expect(fwBanners(st('running'))).toEqual(none);
    expect(fwBanners({ ...st('mixed'), busy: 'check' })).toEqual(none);
  });
  it('이름이 많으면 줄인다', () => {
    expect(shortNames(st('update').boards.slice(0, 5))).toBe('HRR, HRP, HRK 외 2');
  });
});

describe('확인창 — 동의를 한 창에', () => {
  const ask = (scn: string, target: FwTarget = 'all', powerDown = false) => {
    const a = fwAsk(target, st(scn), powerDown);
    if (!a) throw new Error(`no ask: ${scn} ${target}`);
    return a;
  };
  const CUT = '다리·팔 전원을 끄고 진행합니다. 끝나도 꺼진 채로 남으니 다시 기동하세요.';
  it('전체 — 대상 · 다운그레이드 · 전원(끄고 남김 · 모터 때 다리 · UPC · 12V)', () => {
    expect(fwConfirmLines(ask('update'))).toEqual([
      '다음 보드를 업데이트합니다: PDU, IF, SIDE_H, TOP, 모터(16개)',
      'SIDE_H는 보드(261005)보다 낮은 버전(261004)으로 내려갑니다.',
      CUT,
      '모터를 올리는 동안에는 다리 전원을 켰다가 끝나면 다시 끕니다.',
      'PDU를 올리는 동안 UPC 전원이 잠시 꺼집니다.',
      'TOP을 올리는 동안 12V 포트가 잠시 꺼집니다.',
    ]);
  });
  it('제어 중이면 전원 내림 줄', () => {
    expect(fwConfirmLines(ask('update', 'all', true))).toContain('다리 또는 팔에 제어가 들어가 있습니다. 전원을 내리고 진행합니다.');
  });
  it('모터가 대상 아님이면 다리 켜는 줄이 없다', () => {
    expect(fwConfirmLines(ask('unmanaged')).some((l) => l.startsWith('모터를 올리는'))).toBe(false);
  });
  it('부트 보드 복구 · 변형 교정', () => {
    expect(fwConfirmLines(ask('boot'))).toContain('IF는 부트에 남아 있어 앱으로 보내 확인한 뒤 올립니다.');
    expect(variantFix(st('variant').boards[17])).toBe(2);
    expect(fwConfirmLines(ask('variant'))).toContain('IF를 맞는 변형(HW2)으로 바꿉니다.');
  });
  it('다리가 꺼져 모터를 업데이트 때 확인 — 몸통이 다 최신이어도 묻는다', () => {
    const l = fwConfirmLines(ask('legs_off'));
    expect(l[0]).toBe('모터를 확인해 새 버전이 있으면 올립니다.');
    expect(l).toContain('다리 전원이 꺼져 있어 모터는 업데이트하면서 다리 전원을 켜고 확인합니다.');
    expect(l).toContain(CUT);
  });
  it('보드 하나 — 제어 없이 IF·TOP 은 다리·팔 전원을 건드리지 않는다, PDU·모터는 전원 문구', () => {
    expect(fwConfirmLines(ask('update', 17))).toEqual(['다음 보드를 업데이트합니다: IF', '다리·팔 전원은 건드리지 않습니다.']);
    expect(fwConfirmLines(ask('update', 20)).slice(1)).toEqual(['다리·팔 전원은 건드리지 않습니다.', 'TOP을 올리는 동안 12V 포트가 잠시 꺼집니다.']);
    expect(fwConfirmLines(ask('update', 16)).slice(1)).toEqual([CUT, 'PDU를 올리는 동안 UPC 전원이 잠시 꺼집니다.']);
    expect(fwConfirmLines(ask('update', 'motors'))).toEqual(['다음 보드를 업데이트합니다: 모터(16개)',
      '모터를 올리는 동안 다리 전원을 켰다가 끝나면 끕니다. 끝나도 다리 전원은 꺼진 채로 남으니 다시 기동하세요.']);
    expect(fwAsk(0, st('update'), false)).toBeNull();
    expect(fwConfirmLines(ask('update', 17, true))).toContain(CUT);
  });
  it('상태가 바뀌어 다시 묻는 창은 그 사실을 먼저 적는다', () => {
    expect(fwConfirmLines({ ...ask('update'), changed: true })[1]).toBe('확인한 뒤 보드 상태가 바뀌었습니다 — 바뀐 내용을 다시 확인하세요.');
  });
  it('같은 것에 동의했나 — 대상 판정·파일·전원 동의가 같아야 같다', () => {
    const a = ask('update');
    expect(sameAsk(a, ask('update'))).toBe(true);
    expect(sameAsk(a, ask('update', 'all', true))).toBe(false);
    expect(sameAsk(a, ask('changed:after'))).toBe(false);
    expect(fwAsk('all', st('latest'), false)).toBeNull();
    expect(fwAsk(18, st('update'), false)).toBeNull();
  });
  it('모터가 많으면 묶는다', () => {
    expect(boardListText(st('latest').boards)).toBe('PDU, IF, SIDE_F, SIDE_H, TOP, 모터(16개)');
  });
});

describe('진행 단계', () => {
  const rows = (s: FwStatus, hint = false) => fwRunRows(s, hint).map((r) => `${r.label}:${r.state}${r.pct != null ? `:${r.pct}%` : ''}${r.retry ? `:r${r.retry}` : ''}`);
  it('보드 단계 — TOP 을 다시 보내는 중, 모터 줄은 이름과 진행을 나눠 쓴다', () => {
    const s = st('running');
    expect(rows(s)).toEqual([
      '48V 끄기:done', 'PDU:done', 'IF:done', 'SIDE_F:skipped', 'SIDE_H:skipped', 'TOP:active:43%:r1',
      '다리 켜기:pending', '모터:pending', '다리 끄기:pending',
    ]);
    expect(fwRunRows(s).find((r) => r.key === 'motors')?.note).toBe('16개 올릴 예정');
  });
  it('모터 단계 — 지금 모터 · 단계 · 몇 개 중 몇 개', () => {
    const r = fwRunRows(st('motors'));
    expect(r.find((x) => x.key === 'leg_on')?.state).toBe('done');
    expect(r.find((x) => x.key === 'motors')).toMatchObject({ label: '모터', state: 'active', pct: 70, note: 'HRP · 전송 중 70% · 16개 중 1개 끝' });
  });
  it('성공 — 전부 끝', () => {
    expect(fwRunRows(st('success')).every((r) => r.state === 'done' || r.state === 'skipped')).toBe(true);
  });
  it('전송 실패 — 실패한 보드에서 멈추고, 다리 끄기는 끝난 것(실패하면 로봇이 다리를 끈다)', () => {
    expect(rows(st('failed'))).toEqual([
      '48V 끄기:done', 'PDU:done', 'IF:done', 'SIDE_F:skipped', 'SIDE_H:skipped', 'TOP:failed:r3',
      '다리 켜기:pending', '모터:pending', '다리 끄기:done',
    ]);
  });
  it('모터 무응답(12) — 로봇이 멈춘 단계(leg_on) 그대로', () => {
    const r = fwRunRows(st('failed:12'));
    expect(r.find((x) => x.key === 'leg_on')?.state).toBe('failed');
    expect(r.find((x) => x.key === 'leg_off')?.state).toBe('done');
    expect(r.find((x) => x.key === 'motors')).toMatchObject({ state: 'pending', note: '응답 없음' });
    expect(fwRunRows(st('failed')).find((x) => x.key === 'motors')?.note).toBeUndefined();
  });
  it('모터가 대상 아님이면 다리 켜기 줄 없이 모터 한 줄', () => {
    const s = parseFwStatus(demoFwLiveRun(demoFwStatus('unmanaged'), 'all', true, false, 3000, 9))!;
    expect(rows(s).filter((x) => x.startsWith('다리 켜기'))).toEqual([]);
    expect(fwRunRows(s).find((x) => x.key === 'motors')).toMatchObject({ state: 'skipped', note: '대상 아님' });
  });
  it('다리가 꺼진 채 시작 — 모터는 다리를 켠 뒤 확인한다', () => {
    const s = parseFwStatus(demoFwLiveRun(demoFwStatus('legs_off'), 'all', false, false, 2000, 9))!;
    expect(fwRunRows(s).find((x) => x.key === 'motors')).toMatchObject({ state: 'pending', note: '다리 전원을 켠 뒤 확인합니다' });
  });
  it('보드 하나 — 48V 끄기 줄은 그 단계를 봤거나 동의를 보냈을 때만', () => {
    const at = mk((b) => (b.slot === 17 ? { ...b, mode: 'boot', present: false } : b), { id: 3, kind: 'board', state: 'running', stage: 'power_off', board: 17, reason: 0 });
    expect(rows(at)).toEqual(['48V 끄기:active', 'IF:pending']);
    const done = mk((b) => (b.slot === 17 ? { ...b, result: 'success' } : b), { id: 3, kind: 'board', state: 'success', stage: 'boards', board: 17, reason: 0 });
    expect(rows(done)).toEqual(['IF:done']);
    expect(rows(done, true)).toEqual(['48V 끄기:done', 'IF:done']);
  });
  it('보드 하나 — PDU 는 늘 48V 끄기 · 모터 묶음은 다리 켜기 · 모터 한 줄 · 다리 끄기', () => {
    const pdu = mk((b) => (b.slot === 16 ? { ...b, result: 'success' } : b), { id: 3, kind: 'board', state: 'success', stage: 'boards', board: 16, reason: 0 });
    expect(rows(pdu)).toEqual(['48V 끄기:done', 'PDU:done']);
    const m = mk((b) => (b.slot === 2 || b.slot === 5 ? { ...b, result: 'success' } : b), { id: 3, kind: 'board', state: 'success', stage: 'leg_off', board: 5, reason: 0 });
    expect(rows(m)).toEqual(['다리 켜기:done', '모터:done', '다리 끄기:done']);
    expect(fwRunRows(m).find((r) => r.key === 'motors')?.note).toBe('2개 올림');
    expect(runKindLabel(m)).toBe('모터');
    expect(rows({ ...m, run: { ...m.run, kind: 'force' } })).toEqual(['다리 켜기:done', 'HLP:done', '다리 끄기:done']);
  });
  it('전체 성공 뒤 다리를 꺼서 모터가 답하지 않아도 모터 줄은 결과 — n개 올림(10-08 실로봇)', () => {
    const legs = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14];
    const s = mk((b) => (Number(b.slot) < 16 ? { ...b, present: false, verdict: 'no_response', result: legs.includes(Number(b.slot)) ? 'success' : 'none' } : b),
      { id: 1, kind: 'all', state: 'success', stage: 'leg_off', board: -1, reason: 0 });
    const motor = fwRunRows(s).find((r) => r.key === 'motors');
    expect(motor).toMatchObject({ state: 'done', note: '12개 올림' });
    expect(fwRunResult(s)?.title).toBe('업데이트 완료 — 보드 12개');
  });
  it('업데이트를 돌린 적이 없거나 거절이면 빈 목록', () => {
    expect(fwRunRows(st('update'))).toEqual([]);
    expect(fwRunRows(st('rejected:5'))).toEqual([]);
  });
});

describe('결과 문구 — 사유 코드별', () => {
  it('성공 — 전원은 꺼진 채라고 한 줄', () => {
    expect(fwRunResult(st('success'))).toEqual({ ok: true, title: '업데이트 완료 — 보드 19개', text: '', power: '다리·팔 전원은 꺼진 채입니다 — 다시 기동하세요.' });
  });
  it('제목의 수는 이번 업데이트가 다룬 줄만 — 옛 GUI hw 쓰기가 남긴 다른 보드의 성공은 세지 않는다(10-07 실로봇)', () => {
    const s = st('success');
    const one: FwStatus = { ...s, run: { ...s.run, kind: 'force', board: 20, stage: 'boards' } };
    expect(fwRunRows(one).map((r) => `${r.label}:${r.state}`)).toEqual(['TOP:done']);
    expect(fwRunResult(one)?.title).toBe('업데이트 완료 — 보드 1개');
  });
  it('10·11·13 은 처음 전송 뒤 재시도 3번까지 실패, 보드는 부트에', () => {
    expect(fwRunResult(st('failed:10'))?.text).toBe('TOP 전송이 시간 초과로 재시도 3번까지 모두 실패했습니다. 보드는 부트 상태로 두었습니다. 잠시 뒤 다시 업데이트하세요.');
    expect(fwRunResult(st('failed:11'))?.text).toContain('해시 불일치');
    expect(fwRunResult(st('failed:13'))?.text).toContain('업데이트 중에 응답하지 않아 재시도 3번까지');
    expect(fwRunResult(st('failed:11'))).toMatchObject({ ok: false, title: '업데이트 실패 — TOP', power: '다리·팔 전원은 꺼진 채입니다 — 다시 기동하세요.' });
  });
  it('12 · 14 · 9 · 0', () => {
    expect(fwRunResult(st('failed:12'))?.text).toBe('다리 전원을 켰지만 모터 보드가 하나도 응답하지 않습니다. 전원과 배선을 확인해 주세요.');
    expect(fwRunResult(st('failed:14'))).toMatchObject({ ok: false, title: '업데이트 실패 — TOP' });
    expect(fwRunResult(st('failed:9'))?.text).toContain('IF를 부트에서 앱으로');
    expect(fwRunResult(st('failed:0'))).toMatchObject({ title: '업데이트 실패', text: '업데이트 중 로봇에서 오류가 나 멈췄습니다. 로봇 로그를 확인한 뒤 다시 시도하세요.' });
  });
  it('보드 하나가 응답하지 않아 거절(13) — 전송 실패 문구가 아니다', () => {
    const r = fwRunResult(st('rejected:13'));
    expect(r).toMatchObject({ title: '업데이트를 시작하지 못했습니다 — IF', power: '' });
    expect(r?.text).toBe('IF가 응답하지 않아 업데이트를 시작하지 못했습니다. 보드 전원과 배선을 확인한 뒤 [버전 확인]을 다시 누르세요.');
  });
  it('전체가 같은 버전으로 거절(7) — 아무 보드 이름을 붙이지 않는다', () => {
    expect(fwRunResult(st('rejected:7'))).toMatchObject({ title: '업데이트를 시작하지 못했습니다', text: '올릴 보드가 없습니다 — 모든 보드가 이미 최신입니다.' });
  });
  it('거절 — 사유를 단 보드 이름', () => {
    expect(fwRunResult(st('rejected:5'))).toMatchObject({ title: '업데이트를 시작하지 못했습니다 — TOP', text: 'TOP의 hw를 몰라 맞는 파일을 정할 수 없습니다.' });
    const one = mk((b) => (b.slot === 16 ? { ...b, verdict: 'same', reason: 7 } : b), { id: 4, kind: 'board', state: 'rejected', stage: 'precheck', board: 16, reason: 7 });
    expect(fwRunResult(one)?.text).toBe('PDU는 이미 같은 버전입니다.');
  });
  it('끝난 뒤 전원 — 전체·PDU 는 다리·팔, 모터 하나·실패는 다리, 제어 없이 IF 하나는 그대로', () => {
    const one = (slot: number, state: string, stage: string) => mk((b) => (b.slot === slot ? { ...b, result: state === 'success' ? 'success' : 'failed' } : b), { id: 4, kind: 'board', state, stage, board: slot, reason: state === 'failed' ? 10 : 0 });
    expect(fwPowerLeft(one(16, 'success', 'boards'))).toBe('legs_arms');
    expect(fwPowerLeft(one(3, 'success', 'leg_off'))).toBe('legs');
    expect(fwPowerLeft(one(17, 'success', 'boards'))).toBeNull();
    expect(fwPowerLeft(one(17, 'success', 'boards'), true)).toBe('legs_arms');
    expect(fwPowerLeft(one(17, 'failed', 'boards'))).toBe('legs');
    expect(fwPowerLeft(st('rejected:5'))).toBeNull();
  });
  it('업데이트가 없었으면 null', () => {
    expect(fwRunResult(st('update'))).toBeNull();
  });
});

describe('시작 명령 거절 — 전송 계층이 넘기는 실제 모양', () => {
  const P = '/api/firmware/update';
  it('전원 내림 동의 필요 → 다시 묻는다', () => {
    const e = asTransport(P, 409, { error: 'power_down consent required' });
    expect(e.message).toBe('/api/firmware/update → 409 (power_down consent required)');
    expect(fwPostError(e)).toMatchObject({ needPowerDown: true, changed: false });
  });
  it('확인 결과가 바뀜 → 다시 받아 다시 묻는다', () => {
    expect(fwPostError(asTransport(P, 409, { error: 'firmware status changed' }))).toMatchObject({ changed: true, needPowerDown: false });
  });
  it('소유권 · 구동 중 · 진행 중 · CAN-FD 아님 · 무응답', () => {
    expect(fwPostError(asTransport(P, 403, { error: 'not owner' })).text).toBe('다른 기기가 조종 중입니다');
    expect(fwPostError(asTransport(P, 409, { error: 'robot is moving' })).text).toContain('앉힌 뒤');
    expect(fwPostError(asTransport(P, 409, { error: 'firmware update in progress' })).text).toContain('진행 중');
    expect(fwPostError(asTransport(P, 409, { error: 'not a CAN FD robot' })).text).toContain('CAN-FD');
    expect(fwPostError(Object.assign(new Error('/api/firmware/update — command 응답 없음(8s)'), { status: 0 })).text).toContain('응답하지 않습니다');
    expect(fwPostError(asTransport(P, 400, { error: 'bad' })).needPowerDown).toBe(false);
  });
});

describe('데모 흉내 — [업데이트]·[버전 확인]', () => {
  const before = demoFwStatus('update');
  const live = (b: ReturnType<typeof demoFwStatus>, target: FwTarget, down: boolean, pd: boolean, ms: number) => parseFwStatus(demoFwLiveRun(b, target, down, pd, ms, 9))!;
  it('시작 직후는 사전 확인(작업 중), 끝나면 성공하고 올린 보드는 최신', () => {
    expect(live(before, 'all', true, false, 0)).toMatchObject({ busy: 'update', run: { state: 'running', stage: 'precheck', id: 9 } });
    const done = live(before, 'all', true, false, 600_000);
    expect(done).toMatchObject({ busy: 'none', run: { state: 'success', stage: 'leg_off' } });
    expect(summarize(done).targets).toEqual([]);
    expect(done.boards.filter((b) => b.result === 'success')).toHaveLength(20);
  });
  it('모터 묶음 — 같은 버전이어도 내용이 다른 모터까지 굽고, 끝나면 마지막 칸을 가리킨다', () => {
    const done = live(demoFwStatus('mixed'), 'motors', false, false, 600_000);
    expect(done.run).toMatchObject({ kind: 'board', state: 'success', stage: 'leg_off', board: 15 });
    expect(done.boards.filter((b) => b.result === 'success').map((b) => b.name)).toEqual(['HRK', 'FLW']);
    expect(motorGroup(done)).toMatchObject({ verdict: 'latest', mixed: false });
  });
  it('TOP 은 한 번 다시 보낸다(재시도 표시)', () => {
    let seen = false;
    for (let ms = 0; ms < 60_000 && !seen; ms += 250) {
      const s = live(before, 'all', true, false, ms);
      seen = s.run.board === 20 && s.boards[20].step === 'transfer' && s.boards[20].retry === 1;
    }
    expect(seen).toBe(true);
  });
  it('다운그레이드에 동의하지 않으면 그 보드는 굽지 않는다', () => {
    expect(live(before, 'all', false, false, 600_000).boards[19].result).toBe('skipped');
  });
  it('보드 하나 — 끝난 뒤에도 그 칸을 가리키고, 동의를 보냈으면 48V 끄기를 거친다', () => {
    const done = live(before, 17, false, false, 600_000);
    expect(done.run).toMatchObject({ kind: 'board', board: 17, stage: 'boards' });
    expect(done.boards.filter((b) => b.result === 'success').map((b) => b.name)).toEqual(['IF']);
    expect(live(before, 17, false, true, 1500).run.stage).toBe('power_off');
    expect(live(before, 17, false, false, 1500).run.stage).toBe('boards');
  });
  it('부트 보드 — 앱으로 보내 hw 를 읽고 파일을 정한 뒤 굽는다', () => {
    const b = demoFwStatus('boot');
    expect(live(b, 'all', false, false, 2600).boards[17].step).toBe('to_app');
    const done = live(b, 'all', false, false, 600_000);
    expect(done.boards[17]).toMatchObject({ result: 'success', mode: 'app', hw: { state: 'ok', ver: demoHwVer(2) } });
  });
  it('다리가 꺼진 채 — 다리를 켜면 모터가 답하고 새 버전인 모터를 굽는다', () => {
    const done = live(demoFwStatus('legs_off'), 'all', false, false, 600_000);
    expect(done.run.state).toBe('success');
    expect(done.boards.filter((b) => b.result === 'success').map((b) => b.name)).toEqual(['HRR']);
  });
  it('[버전 확인] — 2초 확인 중(작업 중), 그다음 확인 회차가 오른다', () => {
    expect(parseFwStatus(demoFwLiveCheck(before, 500))).toMatchObject({ busy: 'check', run: { state: 'checking' } });
    expect(parseFwStatus(demoFwLiveCheck(before, 2500))).toMatchObject({ busy: 'none', checkSeq: 13 });
    expect(neverChecked(parseFwStatus(demoFwLiveCheck(demoFwStatus('unchecked'), 2500))!)).toBe(false);
  });
});
