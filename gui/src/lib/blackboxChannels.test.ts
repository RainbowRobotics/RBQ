import { describe, it, expect } from 'vitest';
import { groupChannels, groupChannelNames, filterGroups } from './blackboxChannels';
import { demoBlackboxChannels } from './blackboxSchema';

const NAMES = demoBlackboxChannels();

describe('groupChannels — 실기 헤더 425채널', () => {
  const groups = groupChannels(NAMES);
  const rendered = groups.flatMap(groupChannelNames);

  it('레코더 채널이 425개다', () => {
    expect(NAMES).toHaveLength(425);
  });

  it('모든 채널이 화면에 올라간다 — 누락 0', () => {
    const missing = NAMES.filter((n) => !rendered.includes(n));
    expect(missing).toEqual([]);
    expect(rendered).toHaveLength(NAMES.length);
  });

  it('한 채널이 두 그룹에 중복되지 않는다', () => {
    expect(new Set(rendered).size).toBe(rendered.length);
  });

  it('없는 칸은 undefined — 표가 빈칸을 그리게 한다', () => {
    const joint = groups.find((g) => g.key === 'joint');
    expect(joint?.kind).toBe('matrix');
    if (joint?.kind !== 'matrix') return;
    expect(joint.name('0', 'ref.joint.pos')).toBe('ref.joint.pos[0]');
    expect(joint.name('0', 'ref.joint.vel')).toBeUndefined();
  });

  it('조인트 그룹 = 12행, 한 행에 그 관절의 모든 채널이 모인다', () => {
    const joint = groups.find((g) => g.key === 'joint');
    if (joint?.kind !== 'matrix') throw new Error('joint matrix 없음');
    expect(joint.rows).toEqual(['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11']);
    for (const stem of ['joint.pos', 'joint.vel', 'joint.torque', 'motor.cur', 'motor.temp',
      'board.temp', 'motor.err.jam', 'motor.err.cur', 'motor.connect', 'motor.comm_stat',
      'ref.joint.pos', 'ref.joint.torque', 'can.motor.rx_hz', 'can.motor.gap_ms']) {
      expect(joint.cols).toContain(stem);
      expect(joint.name('3', stem)).toBe(`${stem}[3]`);
    }
  });

  it('휠 채널이 관절과 섞이지 않는다 — 인덱스 범위가 다르다(4 vs 12)', () => {
    const wheel = groups.find((g) => g.key === 'wheel');
    const joint = groups.find((g) => g.key === 'joint');
    if (wheel?.kind !== 'matrix' || joint?.kind !== 'matrix') throw new Error('matrix 없음');
    expect(wheel.rows).toEqual(['0', '1', '2', '3']);
    expect(joint.cols.some((c) => c.startsWith('wheel.'))).toBe(false);
    expect(wheel.name('0', 'can.wheel.rx_hz')).toBe('can.wheel.rx_hz[0]');
  });

  it('CAN 채널(4)과 CAN 모터(12)는 인덱스 공간이 달라 따로 묶인다', () => {
    const ch = groups.find((g) => g.key === 'can.ch');
    if (ch?.kind !== 'matrix') throw new Error('can.ch matrix 없음');
    expect(ch.rows).toEqual(['0', '1', '2', '3']);
    expect(ch.cols).toContain('can.ch.bus_off');
  });

  it('이름 인덱스(포트·배터리)도 행이 된다', () => {
    const out = groups.find((g) => g.key === 'pdu.out');
    const bat = groups.find((g) => g.key === 'pdu.bat');
    if (out?.kind !== 'matrix' || bat?.kind !== 'matrix') throw new Error('pdu matrix 없음');
    expect(out.rows).toContain('leg_48v');
    expect(out.name('leg_48v', 'v')).toBe('pdu.out.leg_48v.v');
    expect(bat.rows).toEqual(['left', 'right']);
    expect(bat.name('left', 'soc')).toBe('pdu.bat.left.soc');
  });

  it('로봇이 안 남기는 채널은 아예 그룹이 생기지 않는다 — 빈 섹션 없음', () => {
    for (const key of ['body', 'contact', 'error_cnt', 'error_cnt2', 'state']) {
      expect(groups.find((g) => g.key === key)).toBeUndefined();
    }
  });

  it('검색이 그룹을 좁힌다', () => {
    expect(filterGroups(groups, '').length).toBe(groups.length);
    const cur = filterGroups(groups, 'motor.cur');
    expect(cur.length).toBeGreaterThan(0);
    expect(cur.flatMap(groupChannelNames).every((n) => n.includes('motor.cur'))).toBe(true);
    expect(filterGroups(groups, 'zzz-없는채널')).toEqual([]);
  });
});

describe('groupChannels — 방어', () => {
  it('빈 헤더는 빈 결과', () => {
    expect(groupChannels([])).toEqual([]);
  });

  it('규칙에 없는 새 채널도 버리지 않는다(폴백)', () => {
    const g = groupChannels(['brandnew.thing[0]', 'brandnew.thing[1]', 'lonely_scalar']);
    expect(g.flatMap(groupChannelNames).sort())
      .toEqual(['brandnew.thing[0]', 'brandnew.thing[1]', 'lonely_scalar']);
  });
});
