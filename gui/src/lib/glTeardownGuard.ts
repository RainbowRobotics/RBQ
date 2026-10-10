
const GL_STACK_RE = /onFirstUse|renderObjects|renderScene|WebGLUniforms|_onSurfaceCreate|WebGLProgram/;
const GL_MSG_RE = /TypeError.*of undefined|Cannot read propert.*of undefined|Cannot set propert.*of undefined/;

let installed = false;

export function installGlTeardownGuard() {
  if (installed) return;
  installed = true;
  const eu = (globalThis as { ErrorUtils?: {
    getGlobalHandler: () => (e: Error, isFatal?: boolean) => void;
    setGlobalHandler: (h: (e: Error, isFatal?: boolean) => void) => void;
  } }).ErrorUtils;
  if (!eu) return;
  const prev = eu.getGlobalHandler();
  eu.setGlobalHandler((e, isFatal) => {
    const msg = `${e?.name ?? ''}: ${e?.message ?? ''}`;
    if (GL_MSG_RE.test(msg) && GL_STACK_RE.test(e?.stack ?? '')) {
      console.warn('[glTeardownGuard] 3D 해체 레이스 에러 무시:', msg);
      return;
    }
    prev(e, isFatal);
  });
}
