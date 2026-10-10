
import { issueTicket, type Ticket } from './remoteLogin';
import { SB_URL, SB_KEY } from './logUploadCommon';
import { robotId as currentRobotId } from './remoteIce';

let session: { pin: string; robots: { robotId: string; serial: string }[] } | null = null;
let cached: (Ticket & { forRobotId: string }) | null = null;

export function setTicketSession(pin: string, robots: { robotId: string | null; robotSerial: string | null }[]): void {
  const usable = robots
    .filter((r) => !!r.robotId && !!r.robotSerial)
    .map((r) => ({ robotId: r.robotId as string, serial: r.robotSerial as string }));
  session = pin && usable.length > 0 ? { pin, robots: usable } : null;
  cached = null;
}

export function clearTicketSession(): void {
  session = null;
  cached = null;
}

const RENEW_MARGIN_MS = 30_000;

function usable(t: (Ticket & { forRobotId: string }) | null, wantRobotId: string, now: number): boolean {
  if (!t || t.forRobotId !== wantRobotId) return false;
  const exp = Date.parse(t.expiresAt);
  return Number.isFinite(exp) && exp - now > RENEW_MARGIN_MS;
}

export async function currentTicket(clientId: string, now: () => number = Date.now): Promise<string> {
  const want = currentRobotId();
  if (!want) return '';
  if (usable(cached, want, now())) return cached!.ticket;
  if (!session) return '';
  const hit = session.robots.find((r) => r.robotId === want);
  if (!hit) return '';
  const r = await issueTicket(session.pin, hit.serial, clientId, { url: SB_URL, anonKey: SB_KEY });
  if (!r.ok) return '';
  cached = { ...r.ticket, forRobotId: want };
  return r.ticket.ticket;
}

export function ticketExpiry(): string | null {
  return cached?.expiresAt ?? null;
}
