import { describe, expect, it, vi } from 'vitest';
import { create } from 'zustand';

vi.mock('@/store/robot', () => ({ useRobot: create(() => ({ conn: 'disconnected', ip: 'robot', pushLog: vi.fn() })) }));
vi.mock('@/store/robotSettings', () => ({ useRobotSettings: create<{ serial: string; setSerial: (serial: string) => void }>((set) => ({
  serial: 'A', setSerial: (serial) => set({ serial }),
})) }));
vi.mock('@/lib/commandBus', () => ({ dcCommand: vi.fn() }));
vi.mock('@/lib/rest', () => ({ rest: { serialNumber: vi.fn() } }));

import { useRobot } from '@/store/robot';
import { useRobotSettings } from '@/store/robotSettings';
import { dcCommand } from '@/lib/commandBus';
import { installRobotIdentity } from './robotIdentity';

describe('robot identity across connections', () => {
  it('clears the previous serial on reconnect and discards a disconnected response', async () => {
    let resolve!: (value: { serial_number: string }) => void;
    vi.mocked(dcCommand).mockImplementation(() => new Promise((r) => { resolve = r; }));
    installRobotIdentity();
    useRobot.setState({ conn: 'connected' });
    expect(useRobotSettings.getState().serial).toBe('');
    resolve({ serial_number: 'A' });
    await vi.waitFor(() => expect(useRobotSettings.getState().serial).toBe('A'));
    useRobot.setState({ conn: 'disconnected' });
    expect(useRobotSettings.getState().serial).toBe('A');
    useRobot.setState({ conn: 'connected' });
    expect(useRobotSettings.getState().serial).toBe('');
    const stale = resolve;
    useRobot.setState({ conn: 'disconnected' });
    useRobot.setState({ conn: 'connected' });
    stale({ serial_number: 'A' });
    await Promise.resolve();
    await Promise.resolve();
    expect(useRobotSettings.getState().serial).toBe('');
    resolve({ serial_number: 'B' });
    await vi.waitFor(() => expect(useRobotSettings.getState().serial).toBe('B'));
    useRobot.setState({ ip: 'another-robot' });
    expect(useRobotSettings.getState().serial).toBe('');
    resolve({ serial_number: 'C' });
    await vi.waitFor(() => expect(useRobotSettings.getState().serial).toBe('C'));
  });
});
