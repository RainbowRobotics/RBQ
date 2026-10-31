import { useRobot } from '@/store/robot';
import { useRobotSettings } from '@/store/robotSettings';
import { dcCommand } from '@/lib/commandBus';
import { rest } from '@/lib/rest';

async function fetchSerial(ip: string): Promise<string> {
  try {
    const r = await dcCommand('GET', '/api/robot/serial_number');
    const sn = String(r?.serial_number ?? '').trim();
    if (sn) return sn;
  } catch { }
  const r = await rest.serialNumber(ip);
  return String(r?.serial_number ?? '').trim();
}

let installed = false;

export function installRobotIdentity() {
  if (installed) return;
  installed = true;
  let seq = 0;
  const onConnected = async () => {
    const my = ++seq;
    const ip = useRobot.getState().ip;
    for (const wait of [0, 800, 2000, 5000]) {
      if (wait) await new Promise((r) => setTimeout(r, wait));
      if (my !== seq || useRobot.getState().conn !== 'connected') return;
      try {
        const sn = await fetchSerial(ip);
        if (sn && my === seq) { useRobotSettings.getState().setSerial(sn); return; }
      } catch { }
    }
    if (my === seq && useRobot.getState().conn === 'connected') {
      useRobot.getState().pushLog({ ts: '', process: 'App', level: 'WARNING',
        msg: '로봇 시리얼을 받지 못해 이 로봇의 설정값을 기기에 저장하지 않습니다(ROBOT_SERIAL_NUMBER 미설정?)' });
    }
  };
  useRobot.subscribe((s, prev) => {
    if (s.ip !== prev.ip) { seq++; useRobotSettings.getState().setSerial(''); }
    if (s.conn === 'connected' && prev.conn !== 'connected') void onConnected();
  });
  if (useRobot.getState().conn === 'connected') void onConnected();
}
