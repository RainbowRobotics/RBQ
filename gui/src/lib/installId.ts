
const KEY = 'rbq-install-id';
const CH = 'rbq-install-id';
const ASK_MS = 150;
let id = Math.random().toString(16).slice(2, 10);

export const installIdReady: Promise<void> = (async () => {
  try {
    const s = (globalThis as { sessionStorage?: Storage }).sessionStorage;
    if (s) {
      const v = s.getItem(KEY);
      if (v && !(await takenByOtherTab(v))) id = v; else s.setItem(KEY, id);
      answerTabs();
      return;
    }
    const AsyncStorage = (await import('@react-native-async-storage/async-storage')).default;
    const v = await AsyncStorage.getItem(KEY);
    if (v) id = v; else await AsyncStorage.setItem(KEY, id);
  } catch { }
})();

export const installId = () => id;


function takenByOtherTab(v: string): Promise<boolean> {
  const BC = (globalThis as { BroadcastChannel?: typeof BroadcastChannel }).BroadcastChannel;
  if (!BC) return Promise.resolve(false);
  return new Promise((done) => {
    const ch = new BC(CH);
    const t = setTimeout(() => { ch.close(); done(false); }, ASK_MS);
    ch.onmessage = (e) => { if (e.data?.mine === v) { clearTimeout(t); ch.close(); done(true); } };
    ch.postMessage({ ask: v });
  });
}

function answerTabs() {
  const BC = (globalThis as { BroadcastChannel?: typeof BroadcastChannel }).BroadcastChannel;
  if (!BC) return;
  const ch = new BC(CH);
  ch.onmessage = (e) => { if (e.data?.ask === id) ch.postMessage({ mine: id }); };
}
