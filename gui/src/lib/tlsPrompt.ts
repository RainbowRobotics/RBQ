import { useRobot } from '@/store/robot';
import { useRobots } from '@/store/robots';
import { disconnectByUser, reconnectCurrent } from '@/lib/connectNow';

export function rememberAgain(entryKey: string, fps: Record<number, string>, forget: number[]) {
  useRobots.getState().rememberTlsPins(entryKey, fps, forget);
  useRobot.getState().setTlsIssue(null);
  if (!reconnectCurrent()) void useRobots.getState().select(entryKey);
}

export function dismissTlsPrompt() {
  useRobot.getState().setTlsIssue(null);
  if (useRobot.getState().conn === 'disconnected') disconnectByUser();
}
