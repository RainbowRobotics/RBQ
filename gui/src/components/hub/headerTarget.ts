import type { RobotProfile } from '@/store/robots';
import { remote } from '@/modules/registry';

export function headerTarget(rvId: string, cur: RobotProfile | null, via: RobotProfile['lastVia'] | undefined):
  { viaEff: RobotProfile['lastVia'] | undefined; otherId: string | null } {
  const rv = remote.robots;
  if (rvId && rv) return { viaEff: rv.lastVia, otherId: rvId === cur?.serial || (cur && rvId === rv.idOf(cur)) ? null : rvId };
  return { viaEff: via ?? (cur && rv?.has(cur) ? rv.lastVia : cur?.lastVia), otherId: null };
}
