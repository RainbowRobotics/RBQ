import { useEffect } from 'react';
import { useRobot } from '@/store/robot';
import { rest } from './rest';
import { connection } from './connection';

export function useTripPoll(intervalMs = 5000) {
  const ip = useRobot((s) => s.ip);
  useEffect(() => {
    if (!ip) return;
    let alive = true;
    const tick = () => {
      if (Date.now() - connection.lastTripPushAt < 8000) return;
      rest.trip(ip).then((t) => { if (alive && t?.total) useRobot.getState().applyTrip(t); }).catch(() => {});
    };
    tick();
    const timer = setInterval(tick, intervalMs);
    return () => { alive = false; clearInterval(timer); };
  }, [ip, intervalMs]);
}
