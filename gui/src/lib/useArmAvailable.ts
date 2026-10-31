import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { armAvailable } from './armAvailability';
import { useSimulatedRobot } from './simulatedRobot';

export function useArmAvailable() {
  const connected = useRobot(s => s.conn === 'connected');
  const simulation = useSimulatedRobot();
  const available = useTelemetry(s => armAvailable(s.robot, s.pdu, simulation));
  return connected && available;
}
