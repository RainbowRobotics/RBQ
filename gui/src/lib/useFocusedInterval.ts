import { useCallback, useEffect, useRef, useState } from 'react';
import { AppState } from 'react-native';
import { useFocusEffect } from 'expo-router';

export function useFocusedInterval(cb: () => void, ms: number, enabled = true, onStop?: () => void) {
  const cbRef = useRef(cb);
  cbRef.current = cb;
  const stopRef = useRef(onStop);
  stopRef.current = onStop;
  const [active, setActive] = useState(AppState.currentState !== 'background');
  useEffect(() => {
    const sub = AppState.addEventListener('change', (s) => setActive(s !== 'background'));
    return () => sub.remove();
  }, []);
  useFocusEffect(useCallback(() => {
    if (!enabled || !active) return undefined;
    cbRef.current();
    const id = setInterval(() => cbRef.current(), ms);
    return () => { clearInterval(id); stopRef.current?.(); };
  }, [enabled, active, ms]));
}
