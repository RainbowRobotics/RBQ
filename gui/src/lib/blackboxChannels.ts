
export type ChannelMatrix = {
  kind: 'matrix';
  key: string;
  title: string;
  rows: string[];
  cols: string[];
  colLabels: string[];
  name: (row: string, col: string) => string | undefined;
};
export type ChannelScalars = { kind: 'scalars'; key: string; title: string; names: string[] };
export type ChannelGroup = ChannelMatrix | ChannelScalars;

const FAMILIES: ReadonlyArray<{ key: string; title: string; test: RegExp }> = [
  { key: 'wheel', title: '휠 · WHEEL', test: /^(wheel\.|can\.wheel\.)/ },
  { key: 'joint', title: '관절 · JOINT', test: /^(joint\.|motor\.|board\.|ref\.joint\.|can\.motor\.)/ },
  { key: 'can.ch', title: 'CAN 채널', test: /^can\.ch\./ },
  { key: 'cpu.core', title: 'CPU 코어', test: /^cpu\.core$/ },
  { key: 'status.attch', title: '부착 장치 · ATTCH', test: /^status\.attch$/ },
  { key: 'joy.btn', title: '조이스틱 버튼', test: /^joy\.btn$/ },
];

const NAMED = /^([a-z0-9_]+\.[a-z0-9_]+)\.([a-z0-9_]+)\.([a-z0-9_]+)$/;

const GROUP_TITLES: Record<string, string> = {
  'pdu.out': '전원 출력 포트', 'pdu.in': '입력 포트', 'pdu.bat': '배터리',
  'pdu.temp': 'PDU 온도', imu: 'IMU', cpu: 'CPU', mem: '메모리', swap: '스왑',
  status: '상태 워드 · STATUS', cmd: '외부 명령 · CMD', joy: '조이스틱',
  can: 'CAN', deadline_miss: 'RT 데드라인',
  'lan2can.connected': 'LAN2CAN', 'can.type': 'CAN 종류', process_time_ms: '처리 시간',
};
const title = (key: string) => GROUP_TITLES[key] ?? key;

function scalarTitle(key: string, names: string[]): string {
  const parts = names.map((n) => n.split('.'));
  let common = parts[0] ?? [];
  for (const p of parts) {
    let i = 0;
    while (i < common.length && i < p.length && common[i] === p[i]) i++;
    common = common.slice(0, i);
  }
  const prefix = common.join('.') || key;
  return GROUP_TITLES[prefix] ?? prefix;
}

function sortRows(rows: string[]): string[] {
  const allNum = rows.every((r) => /^\d+$/.test(r));
  return allNum ? [...rows].sort((a, b) => Number(a) - Number(b)) : rows;
}

function colLabel(family: string, stem: string): string {
  if (family === 'joint' || family === 'wheel') {
    return stem
      .replace(/^wheel\./, '')
      .replace(/^can\.(motor|wheel)\./, 'can.')
      .replace(/^ref\.joint\./, 'ref.')
      .replace(/^joint\./, '')
      .replace(/^motor\./, 'm.')
      .replace(/^board\./, 'b.');
  }
  if (family === 'can.ch') return stem.replace(/^can\.ch\./, '');
  return stem;
}

export function groupChannels(names: Iterable<string>): ChannelGroup[] {
  type Mat = { rows: string[]; cols: string[]; have: Set<string>; fmt: (r: string, c: string) => string };
  const mats = new Map<string, Mat>();
  const scal = new Map<string, string[]>();
  const order: string[] = [];

  const mat = (key: string, fmt: Mat['fmt']): Mat => {
    let m = mats.get(key);
    if (!m) { m = { rows: [], cols: [], have: new Set(), fmt }; mats.set(key, m); order.push(`m:${key}`); }
    return m;
  };
  const addCell = (key: string, row: string, col: string, fmt: Mat['fmt']) => {
    const m = mat(key, fmt);
    if (!m.rows.includes(row)) m.rows.push(row);
    if (!m.cols.includes(col)) m.cols.push(col);
    m.have.add(`${row} ${col}`);
  };
  const addScalar = (key: string, name: string) => {
    let s = scal.get(key);
    if (!s) { s = []; scal.set(key, s); order.push(`s:${key}`); }
    s.push(name);
  };

  for (const nameRaw of names) {
    const n = nameRaw.trim();
    if (!n) continue;

    const mIdx = /^(.*)\[(\d+)\]$/.exec(n);
    if (mIdx) {
      const stem = mIdx[1];
      const row = mIdx[2];
      const fam = FAMILIES.find((f) => f.test.test(stem + '.') || f.test.test(stem));
      const key = fam ? fam.key : stem;
      addCell(key, row, stem, (r, c) => `${c}[${r}]`);
      continue;
    }

    const mNamed = NAMED.exec(n);
    if (mNamed) {
      const [, prefix, row, col] = mNamed;
      addCell(prefix, row, col, (r, c) => `${prefix}.${r}.${c}`);
      continue;
    }

    addScalar(n.includes('.') ? n.slice(0, n.indexOf('.')) : n, n);
  }

  const out: ChannelGroup[] = [];
  for (const id of order) {
    const key = id.slice(2);
    const m = id.startsWith('m:') ? mats.get(key) : undefined;
    if (m) {
      const fam = FAMILIES.find((f) => f.key === key);
      const rows = sortRows(m.rows);
      out.push({
        kind: 'matrix', key, title: fam ? fam.title : title(key), rows, cols: m.cols,
        colLabels: m.cols.map((c) => colLabel(key, c)),
        name: (r, c) => (m.have.has(`${r} ${c}`) ? m.fmt(r, c) : undefined),
      });
      continue;
    }
    const s = id.startsWith('s:') ? scal.get(key) : undefined;
    if (s) out.push({ kind: 'scalars', key, title: scalarTitle(key, s), names: s });
  }
  return out;
}

export function groupChannelNames(g: ChannelGroup): string[] {
  if (g.kind === 'scalars') return [...g.names];
  const out: string[] = [];
  for (const r of g.rows) for (const c of g.cols) { const n = g.name(r, c); if (n) out.push(n); }
  return out;
}

export function filterGroups(groups: ChannelGroup[], q: string): ChannelGroup[] {
  const needle = q.trim().toLowerCase();
  if (!needle) return groups;
  const out: ChannelGroup[] = [];
  for (const g of groups) {
    if (g.title.toLowerCase().includes(needle) || g.key.toLowerCase().includes(needle)) { out.push(g); continue; }
    if (g.kind === 'scalars') {
      const names = g.names.filter((n) => n.toLowerCase().includes(needle));
      if (names.length) out.push({ ...g, names });
      continue;
    }
    const cols = g.cols.filter((c, i) => c.toLowerCase().includes(needle) || g.colLabels[i].toLowerCase().includes(needle));
    const rows = g.rows.filter((r) => r.toLowerCase().includes(needle));
    if (cols.length) out.push({ ...g, cols, colLabels: cols.map((c) => g.colLabels[g.cols.indexOf(c)]) });
    else if (rows.length) out.push({ ...g, rows });
  }
  return out;
}
