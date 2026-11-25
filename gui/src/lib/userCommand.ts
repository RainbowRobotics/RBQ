import { rest } from './rest';
import type { CommandOptions } from './commandBus';
import { askStandGate } from './standGate';

export function sendUserCommand(
  target: number, cmd: number,
  paraInt: number[] = [], paraChar: number[] = [], paraFloat: number[] = [],
  options?: CommandOptions, force = false,
) {
  rest.commandStruct('', target, cmd, {
    ...(paraInt.length ? { int: paraInt } : null),
    ...(paraChar.length ? { char: paraChar } : null),
    ...(paraFloat.length ? { float: paraFloat } : null),
    ...(force ? { force: true } : null),
  }, options).catch((e) => {
    if (!force && askStandGate(e, () => sendUserCommand(target, cmd, paraInt, paraChar, paraFloat, options, true))) return;
    console.warn(`[cmd] USER_COMMAND target=${target} cmd=${cmd} 실패:`, (e as any)?.message ?? e);
  });
}
