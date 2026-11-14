export type RawInput = { pressed: Set<string>; armed: boolean };

export function startKeyboardListeners(_onArmChange: (armed: boolean) => void): () => void {
  return () => {};
}
export function getRawInput(): RawInput {
  return { pressed: new Set(), armed: false };
}
