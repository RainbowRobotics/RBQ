import { useTelemetry } from '@/store/telemetry';
import { useVisionToggles } from '@/store/visionToggles';
import { actions, WEBRTC_RES } from '@/lib/rest';
import { useRobot } from '@/store/robot';
import { onStreamerCommandReady } from '@/lib/commandBus';

export function projectorOnFrom(sensors: { projector: boolean; projectorOn: boolean }[] | undefined): boolean | undefined {
  const withProj = sensors?.filter((x) => x.projector) ?? [];
  if (withProj.length === 0) return undefined;
  return withProj.some((x) => x.projectorOn);
}

export function resIndexFrom(r: { width?: number; height?: number } | undefined): number | undefined {
  if (!r?.width || !r?.height) return undefined;
  const i = WEBRTC_RES.findIndex((x) => x.width === r.width && x.height === r.height);
  return i >= 0 ? i : undefined;
}

let installed = false;

export function installRobotSettingsSync() {
  if (installed) return;
  installed = true;

  useTelemetry.subscribe((s, prev) => {
    if (s.sensors === prev.sensors) return;
    const on = projectorOnFrom(s.sensors);
    if (on === undefined) return;
    const vt = useVisionToggles.getState();
    if (Date.now() < vt.irPendingUntil) return;
    if (vt.irProjector !== on) useVisionToggles.setState({ irProjector: on });
  });

  let seq = 0;
  onStreamerCommandReady(() => {
    const my = ++seq;
    void (async () => {
      for (const wait of [0, 1000, 3000]) {
        if (wait) await new Promise((r) => setTimeout(r, wait));
        if (my !== seq) return;
        try {
          const ip = useRobot.getState().ip;
          const i = resIndexFrom(await actions.visionWebrtcResolutionGet(ip));
          if (i !== undefined && my === seq) useVisionToggles.setState({ resIdx: i });
          return;
        } catch { }
      }
    })();
  });
}
