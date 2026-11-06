export function openFileStream(uri: string): ReadableStreamDefaultReader<Uint8Array> {
  let inner: Promise<ReadableStreamDefaultReader<Uint8Array>> | null = null;
  const get = () => {
    if (!inner) {
      inner = fetch(uri).then((r) => {
        if (!r.ok || !r.body) throw new Error(`파일을 읽을 수 없습니다 (HTTP ${r.status})`);
        return r.body.getReader();
      });
    }
    return inner;
  };
  return {
    read: () => get().then((r) => r.read()),
    cancel: (reason?: unknown) => get().then((r) => r.cancel(reason)).catch(() => {}),
    releaseLock: () => { },
    get closed() { return get().then((r) => r.closed); },
  } as unknown as ReadableStreamDefaultReader<Uint8Array>;
}
