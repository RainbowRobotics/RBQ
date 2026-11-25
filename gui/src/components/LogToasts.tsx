import { useEffect, useRef, useState } from 'react';
import { View, StyleSheet, Animated, Pressable } from 'react-native';
import { useRouter } from 'expo-router';
import { RBSimpleToast } from '@/rb/components/RBToast';
import { useRobot, getLiveAlerts } from '@/store/robot';
import { useViewport } from '@/store/viewport';
import { useLogAck } from '@/store/logAck';
import type { LogLevel, LogLine } from '@/types/robot';
import { fitToasts, mergeToasts, type Toast } from '@/lib/logToast';

const SHOW_MS = 3000;
const FADE_MS = 260;
const MAX_VISIBLE = 3;
const MSG_MAX = 60;
const TOAST_LEVELS: LogLevel[] = ['ERROR', 'FATAL'];

export function LogToasts({ room }: { room?: number } = {}) {
  const router = useRouter();
  const alertSeq = useRobot((s) => s.alertSeq);
  const ackLen = useLogAck((s) => s.ackLen);
  const [toasts, setToasts] = useState<Toast[]>([]);
  const seenRef = useRef(getLiveAlerts().length);
  const idRef = useRef(0);
  const listRef = useRef<Toast[]>([]);
  const timers = useRef(new Map<number, ReturnType<typeof setTimeout>>());
  const [y0, setY0] = useState(0);
  const [hs, setHs] = useState<Record<number, number>>({});

  const clearTimer = (id: number) => {
    const tm = timers.current.get(id);
    if (tm) { clearTimeout(tm); timers.current.delete(id); }
  };

  const drop = (id: number) => {
    clearTimer(id);
    listRef.current = listRef.current.filter((x) => x.id !== id);
    setToasts(listRef.current);
  };

  const arm = (id: number) => {
    clearTimer(id);
    timers.current.set(id, setTimeout(() => drop(id), SHOW_MS));
  };

  useEffect(() => {
    const alerts = getLiveAlerts();
    if (seenRef.current > alerts.length) seenRef.current = alerts.length;
    const fresh = alerts.slice(seenRef.current).filter((l: LogLine) => TOAST_LEVELS.includes(l.level));
    seenRef.current = alerts.length;
    if (!fresh.length) return;

    const { next, bumped, dropped, lastId } = mergeToasts(listRef.current, fresh, idRef.current, MAX_VISIBLE);
    idRef.current = lastId;
    listRef.current = next;
    setToasts(next);
    dropped.forEach(clearTimer);
    bumped.filter((id) => !dropped.includes(id)).forEach(arm);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [alertSeq]);

  useEffect(() => {
    timers.current.forEach((tm) => clearTimeout(tm));
    timers.current.clear();
    listRef.current = [];
    setToasts([]);
  }, [ackLen]);

  useEffect(() => () => {
    timers.current.forEach((tm) => clearTimeout(tm));
    timers.current.clear();
  }, []);

  const setToastNeed = useViewport((s) => s.setToastNeed);
  const newest = toasts[toasts.length - 1];
  const needH = room != null && newest ? y0 + (hs[newest.id] ?? 0) : 0;
  const home = room != null;
  useEffect(() => { if (home) setToastNeed(needH); }, [home, needH, setToastNeed]);
  useEffect(() => () => { if (home) setToastNeed(0); }, [home, setToastNeed]);

  if (!toasts.length) return null;

  const jump = (toast: Toast) => {
    drop(toast.id);
    router.push({ pathname: '/log', params: { focusTs: toast.ts, focusMsg: toast.msg } });
  };

  const shown = room == null ? null : fitToasts(toasts.map((x) => x.id), hs, room - y0, 6);
  return (
    <View pointerEvents="box-none" style={styles.wrap} onLayout={(e) => setY0(e.nativeEvent.layout.y)}>
      {toasts.map((toast) => (
        <View key={toast.id} pointerEvents="box-none" style={shown && !shown.has(toast.id) ? styles.hidden : undefined}
          onLayout={(e) => { const h = Math.round(e.nativeEvent.layout.height); if (h > 0) setHs((m) => (m[toast.id] === h ? m : { ...m, [toast.id]: h })); }}>
          <ToastRow toast={toast} onPress={() => jump(toast)} />
        </View>
      ))}
    </View>
  );
}

function ToastRow({ toast, onPress }: { toast: Toast; onPress: () => void }) {
  const fade = useRef(new Animated.Value(0)).current;
  const [visible, setVisible] = useState(true);

  useEffect(() => {
    setVisible(true);
    Animated.timing(fade, { toValue: 1, duration: 140, useNativeDriver: false }).start();
    const tm = setTimeout(() => {
      Animated.timing(fade, { toValue: 0, duration: FADE_MS, useNativeDriver: false })
        .start(() => setVisible(false));
    }, SHOW_MS - FADE_MS);
    return () => clearTimeout(tm);
  }, [fade, toast.count]);

  const title = [toast.level, toast.process, toast.count > 1 ? `×${toast.count}` : ''].filter(Boolean).join(' · ');
  const msg = toast.msg.length > MSG_MAX ? `${toast.msg.slice(0, MSG_MAX - 1)}…` : toast.msg;
  return (
    <Animated.View style={{ opacity: fade }} pointerEvents={visible ? 'auto' : 'none'}>
      <Pressable onPress={onPress}>
        <RBSimpleToast type="alert" variant="danger" title={title} content={msg} />
      </Pressable>
    </Animated.View>
  );
}

const styles = StyleSheet.create({
  wrap: { alignItems: 'center', gap: 6 },
  hidden: { display: 'none' },
});
