import { useEffect, useRef, useState } from 'react';
import { WebView } from 'react-native-webview';
import { simEngine } from '@/lib/simEngine';
import { simWorkerUrl, simWorkerDir } from '@/lib/simWorkerUrl';

export function SimHost() {
  const ref = useRef<WebView>(null);
  const [seq, setSeq] = useState(simEngine.reloadSeq);
  useEffect(() => simEngine.subscribe(() => setSeq(simEngine.reloadSeq)), []);

  useEffect(() => {
    const off = simEngine.attachHost((j) => {
      ref.current?.injectJavaScript(`window.__simCmd && window.__simCmd(${JSON.stringify(j)}); true;`);
    });
    return off;
  }, [seq]);

  return (
    <WebView
      key={seq}
      ref={ref}
      source={{ uri: simWorkerUrl() }}
      originWhitelist={['*']}
      allowFileAccess
      allowFileAccessFromFileURLs
      allowingReadAccessToURL={simWorkerDir()}
      allowUniversalAccessFromFileURLs
      javaScriptEnabled
      domStorageEnabled={false}
      onMessage={(e) => simEngine.onMessage(e.nativeEvent.data)}
      onError={(e) => simEngine.onMessage(JSON.stringify({ t: 'err', m: e.nativeEvent.description }))}
      style={{ position: 'absolute', width: 1, height: 1, opacity: 0 }}
      pointerEvents="none"
    />
  );
}
