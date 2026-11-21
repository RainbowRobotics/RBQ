import { useRobots } from '@/store/robots';
import { useFeatures } from '@/store/capability';

export const simulatedRobot = () =>
  useRobots.getState().currentSerial === 'sim' || useFeatures.getState().features?.simulation === true;

export function useSimulatedRobot() {
  const localSim = useRobots(s => s.currentSerial === 'sim');
  const remoteSim = useFeatures(s => s.features?.simulation === true);
  return localSim || remoteSim;
}
