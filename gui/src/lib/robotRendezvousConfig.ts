
export type IniKey = { key: string; value: string };
export type IniSection = { name: string; keys: IniKey[] };

const SECTION = 'RENDEZVOUS';

export type RendezvousPatch = {
  enabled: boolean;
  url?: string;
  webrtcToken?: string;
  robotId?: string;
  rendezvousToken?: string;
};

export function applyRendezvousSection(all: IniSection[], patch: RendezvousPatch): IniSection[] {
  const next: Record<string, string> = { enabled: patch.enabled ? 'true' : 'false' };
  if (patch.url !== undefined) next.url = patch.url;
  if (patch.webrtcToken !== undefined) next.webrtc_token = patch.webrtcToken;
  if (patch.robotId !== undefined) next.robot_id = patch.robotId;
  if (patch.rendezvousToken !== undefined) next.token = patch.rendezvousToken;

  const out = all.map((s) => ({ name: s.name, keys: s.keys.map((k) => ({ ...k })) }));

  const target = out.find((s) => s.name === SECTION);
  if (!target) {
    out.push({ name: SECTION, keys: Object.entries(next).map(([key, value]) => ({ key, value })) });
    return out;
  }
  for (const [key, value] of Object.entries(next)) {
    const found = target.keys.find((k) => k.key === key);
    if (found) found.value = value;
    else target.keys.push({ key, value });
  }
  return out;
}

export function rendezvousPatchFromInputs(enabled: boolean, url: string): RendezvousPatch {
  if (!enabled) return { enabled: false };
  return { enabled: true, ...(url.trim() ? { url: url.trim() } : {}) };
}

export function readRendezvousKey(all: IniSection[], key: string): string {
  return all.find((s) => s.name === SECTION)?.keys.find((k) => k.key === key)?.value ?? '';
}
