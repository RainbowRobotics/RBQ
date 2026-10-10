import { useMemo, useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { BlackBoxPlayer } from '@/components/BlackBoxPlayer';
import { SysLogPlayer } from '@/components/SysLogPlayer';
import type { BlackboxSession, BbLogLine } from '@/lib/blackbox';
import type { LogLine } from '@/types/robot';
import { demoBtn } from '../Frame';
import type { WidgetCase } from '../types';

const TICK_MS = 10;
const FRAMES = 400;
const CH = ['imu.rpy.r', 'imu.rpy.p', 'imu.rpy.y', 'status.is_fall', 'status.gait_id',
  'pdu.bat.left.voltage', 'pdu.bat.right.voltage', 'cmd.vel_x', 'joy.l_rl', 'joy.l_ud'];

function demoSession(): BlackboxSession {
  const cols = new Map(CH.map((n, i) => [n, i]));
  const data = new Float32Array(FRAMES * CH.length);
  const put = (f: number, n: string, v: number) => { data[f * CH.length + cols.get(n)!] = v; };
  for (let f = 0; f < FRAMES; f++) {
    const t = (f * TICK_MS) / 1000;
    const fallen = f > 300;
    put(f, 'imu.rpy.r', fallen ? 62 : 4 * Math.sin(t * 6));
    put(f, 'imu.rpy.p', fallen ? -28 : 3 * Math.sin(t * 6 + 1.2));
    put(f, 'imu.rpy.y', t * 12);
    put(f, 'status.is_fall', fallen ? 1 : 0);
    put(f, 'status.gait_id', fallen ? 0 : 2);
    put(f, 'pdu.bat.left.voltage', 50.4 - t * 0.05);
    put(f, 'pdu.bat.right.voltage', 50.1 - t * 0.05);
    put(f, 'cmd.vel_x', fallen ? 0 : 0.6 + 0.1 * Math.sin(t * 2));
    put(f, 'joy.l_rl', fallen ? 0 : 0.3 * Math.sin(t * 1.5));
    put(f, 'joy.l_ud', fallen ? 0 : 0.7);
  }
  const start = Date.UTC(2026, 7, 28, 6, 30, 0);
  const hhmmss = (ms: number) => new Date(start + ms).toISOString().slice(11, 23);
  const logs: BbLogLine[] = [
    { epochMs: start + 200, ts: hhmmss(200), process: 'QuadWalk', level: 'INFO', msg: 'gait: TROT 진입' },
    { epochMs: start + 1500, ts: hhmmss(1500), process: 'Motion', level: 'INFO', msg: 'RT deadline miss x2' },
    { epochMs: start + 3010, ts: hhmmss(3010), process: 'Estimation', level: 'ERROR', msg: '낙상 감지 — roll 62deg' },
    { epochMs: start + 3120, ts: hhmmss(3120), process: 'QuadWalk', level: 'WARNING', msg: '보행 정지, 복구 대기' },
  ];
  return {
    tickMs: TICK_MS, frameCount: FRAMES, startEpochMs: start, cols, numCols: CH.length, data, logs,
    sync: { warn: false, reason: '' }, video: {},
  };
}

function BlackBoxDemo() {
  const { c } = useTheme();
  const sess = useMemo(demoSession, []);
  const [on, setOn] = useState(false);
  return (
    <View style={{ gap: 10 }}>
      <Tappable onPress={() => setOn((v) => !v)} style={[demoBtn, { backgroundColor: c.elev, borderWidth: 1, borderColor: c.line }]}>
        <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{on ? '닫기' : '가짜 세션 재생'}</Text>
      </Tappable>
      {on && (
        <View style={{ height: 420 }}>
          <BlackBoxPlayer ip="" date="2026-08-28" session="demo" preloaded={sess} />
        </View>
      )}
    </View>
  );
}

const DEMO_LOGS: LogLine[] = [
  { ts: '06:30:00.120', process: 'Network', level: 'INFO', msg: '클라이언트 접속 192.168.0.55' },
  { ts: '06:30:00.480', process: 'QuadWalk', level: 'INFO', msg: 'gait: TROT 진입' },
  { ts: '06:30:01.500', process: 'Motion', level: 'WARNING', msg: 'RT deadline miss x2 (12.4ms, CPU 71C)' },
  { ts: '06:30:02.240', process: 'Vision', level: 'INFO', msg: 'PTZ 프리셋 3 이동' },
  { ts: '06:30:03.010', process: 'Estimation', level: 'ERROR', msg: '낙상 감지 — roll 62deg' },
  { ts: '06:30:03.120', process: 'QuadWalk', level: 'WARNING', msg: '보행 정지, 복구 대기' },
  { ts: '06:30:05.900', process: 'QuadWalk', level: 'INFO', msg: '복구 완료 — STAND' },
];

function SysLogDemo() {
  const { c } = useTheme();
  const [on, setOn] = useState(false);
  return (
    <View style={{ gap: 10 }}>
      <Tappable onPress={() => setOn(true)} style={[demoBtn, { backgroundColor: c.elev, borderWidth: 1, borderColor: c.line }]}>
        <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>가짜 로그 재생</Text>
      </Tappable>
      {on && <SysLogPlayer date="2026-08-28" logs={DEMO_LOGS} onClose={() => setOn(false)} />}
    </View>
  );
}

export const playback: WidgetCase[] = [
  { name: 'BlackBoxPlayer',
    code: `import { BlackBoxPlayer } from '@/components/BlackBoxPlayer';

<BlackBoxPlayer ip={ip} date={date} session={session} />
<BlackBoxPlayer ip="" date={d} session="" preloaded={session} />  // zip 임포트·테스트
// preloaded 를 주면 REST 를 타지 않는다 — 로봇 없이 재생할 때 이 경로를 쓴다.`, from: '@/components/BlackBoxPlayer', Demo: BlackBoxDemo,
    size: 'wide', when: '사고 기록 재생 — 4초짜리 가짜 세션(3초에 낙상)이 들어 있다. preloaded 를 주면 REST 없이 재생하므로 zip 임포트도 같은 경로를 쓴다.' },
  { name: 'SysLogPlayer',
    code: `import { SysLogPlayer } from '@/components/SysLogPlayer';

<SysLogPlayer date="2026-08-28" logs={lines} onClose={close} />
// logs 는 LogLine[] — 출처를 가리지 않는다(실시간·파일·더미).`, from: '@/components/SysLogPlayer', Demo: SysLogDemo,
    size: 'wide', when: '시스템 로그를 시간축으로 되감는 재생기. logs 를 그대로 받으므로 어떤 출처의 로그든 물릴 수 있다.' },
];
