import { rest } from './rest';
import type { CommandOptions } from './commandBus';

export function sendUserCommand(
  target: number, cmd: number,
  paraInt: number[] = [], paraChar: number[] = [], paraFloat: number[] = [],
  options?: CommandOptions,
) {
  rest.commandStruct('', target, cmd, {
    ...(paraInt.length ? { int: paraInt } : null),
    ...(paraChar.length ? { char: paraChar } : null),
    ...(paraFloat.length ? { float: paraFloat } : null),
  }, options).catch((e) => {
    console.warn(`[cmd] USER_COMMAND target=${target} cmd=${cmd} 실패:`, (e as any)?.message ?? e);
  });
}
