import { create } from 'zustand';
import { slamCmd, useSlamnav, type TopoNode } from './slamnav';
import { groundZ, headingYaw, linkState } from './slamnavWire';
import { actions } from './rest';
import { connection } from './connection';
import { useRobot } from '@/store/robot';

export type SlamTool = 'view' | 'edit' | 'route';
export const ADD_TYPES = ['ROUTE', 'GOAL', 'BASE', 'STAIR'] as const;

type Undo = () => Promise<boolean>;
type EditStore = {
  tool: SlamTool;
  addType: string;
  autoLink: boolean;
  selected?: string;
  prev?: string;
  anchor?: string;
  moving?: string;
  undo: Undo[];
};

export const useSlamEdit = create<EditStore>(() => ({ tool: 'view', addType: 'ROUTE', autoLink: true, undo: [] }));

const GAIT_STANDING = 1;

const topo = (): TopoNode[] => useSlamnav.getState().topo ?? [];
const nodeOf = (id?: string) => (id ? topo().find((n) => n.id === id) : undefined);
const pushUndo = (u: Undo) => useSlamEdit.setState((s) => ({ undo: [...s.undo, u].slice(-50) }));

function groundPose(n: TopoNode) {
  const st = useSlamnav.getState().state;
  return { x: n.pos[0], y: n.pos[1], z: n.pos[2] + (st ? st.robot.size.z[0] : 0), yaw: n.yaw };
}

function driveAssist(on: boolean) {
  const r = useRobot.getState();
  if (r.ip) actions.gamepadExternal(r.ip, on).catch(() => {});
  if (on && r.robot?.gait_id === GAIT_STANDING) connection.sendMotion('walk', true);
}

export async function slamAnnotSave(onSaved?: () => void): Promise<boolean> {
  if (!(await slamCmd('/api/annot/save', {}, onSaved))) return false;
  useSlamEdit.setState({ undo: [] });
  return slamCmd('/api/mode', { mode: 'DRIVE' });
}


export function setSlamTool(tool: SlamTool) {
  const mode = useSlamnav.getState().state?.mode;
  useSlamEdit.setState({ tool, selected: undefined, prev: undefined, moving: undefined });
  if (tool === 'edit' && mode !== 'ANNOTATION') slamCmd('/api/mode', { mode: 'ANNOTATION' });
  if (tool === 'route' && mode === 'ANNOTATION') slamAnnotSave();
}

export const editReady = () => useSlamnav.getState().state?.mode === 'ANNOTATION';


export function slamTapNode(id: string) {
  const s = useSlamEdit.getState();
  if (s.tool === 'view' || s.moving) return;
  useSlamEdit.setState({
    selected: id,
    prev: s.selected && s.selected !== id ? s.selected : s.prev !== id ? s.prev : undefined,
    anchor: s.tool === 'edit' ? id : s.anchor,
  });
}

export async function slamTapMap(x: number, y: number, z: number, yaw: number | null) {
  const s = useSlamEdit.getState();
  if (s.moving) return moveNode(s.moving, x, y, z, yaw);
  if (s.tool !== 'edit') { useSlamEdit.setState({ selected: undefined, prev: undefined }); return; }
  if (!editReady()) return;
  const anchor = s.autoLink ? nodeOf(s.anchor) : undefined;
  const yawAuto = anchor ? headingYaw([anchor.pos[0], anchor.pos[1]], [x, y]) : null;
  const pose = { x, y, z, yaw: yaw ?? yawAuto ?? useSlamnav.getState().state?.pose.yaw ?? 0 };
  let id = '';
  if (!(await slamCmd('/api/annot/node/add', { type: s.addType, ...pose }, (r) => { id = String(r?.id ?? ''); })) || !id) return;
  if (anchor) await slamCmd('/api/annot/link', { from: anchor.id, to: id, bidirectional: true });
  pushUndo(() => slamCmd('/api/annot/node/delete', { id }));
  useSlamEdit.setState({ anchor: id, selected: id, prev: undefined });
}

export function slamClearSelection() {
  useSlamEdit.setState({ selected: undefined, prev: undefined, moving: undefined });
}


export function slamStartMove(id: string) { useSlamEdit.setState({ moving: id }); }

async function moveNode(id: string, x: number, y: number, z: number, yaw: number | null) {
  const n = nodeOf(id);
  useSlamEdit.setState({ moving: undefined });
  if (!n) return;
  const back = groundPose(n);
  if (await slamCmd('/api/annot/node/move', { id, x, y, z, yaw: yaw ?? n.yaw })) pushUndo(() => slamCmd('/api/annot/node/move', { id, ...back }));
}

export async function slamSetType(id: string, type: string) {
  const n = nodeOf(id);
  if (!n || n.type === type) return;
  if (await slamCmd('/api/annot/node/type', { id, type })) pushUndo(() => slamCmd('/api/annot/node/type', { id, type: n.type }));
}

export async function slamDeleteNode(id: string) {
  const n = nodeOf(id);
  if (!n) return;
  const incoming = topo().filter((m) => m.linked.includes(id)).map((m) => m.id);
  const pose = groundPose(n);
  if (!(await slamCmd('/api/annot/node/delete', { id }))) return;
  useSlamEdit.setState((s) => ({
    selected: undefined, prev: s.prev === id ? undefined : s.prev, anchor: s.anchor === id ? undefined : s.anchor,
  }));
  pushUndo(async () => {
    let nid = '';
    if (!(await slamCmd('/api/annot/node/add', { type: n.type, name: n.name, ...pose }, (r) => { nid = String(r?.id ?? ''); })) || !nid) return false;
    for (const to of n.linked) await slamCmd('/api/annot/link', { from: nid, to, bidirectional: false });
    for (const from of incoming) await slamCmd('/api/annot/link', { from, to: nid, bidirectional: false });
    return true;
  });
}

export async function slamToggleLink(a: string, b: string) {
  const { ab, ba } = linkState(topo(), a, b);
  const send = async (from: string, to: string) => slamCmd('/api/annot/link', { from, to, bidirectional: false });
  if (ab || ba) {
    if (ab) await send(a, b);
    if (ba) await send(b, a);
    pushUndo(async () => (!ab || (await send(a, b))) && (!ba || (await send(b, a))));
  } else if (await slamCmd('/api/annot/link', { from: a, to: b, bidirectional: true })) {
    pushUndo(() => slamCmd('/api/annot/link', { from: a, to: b, bidirectional: true }));
  }
}

export async function slamUndo() {
  const list = useSlamEdit.getState().undo;
  const last = list[list.length - 1];
  if (!last) return;
  useSlamEdit.setState({ undo: list.slice(0, -1), selected: undefined, prev: undefined, moving: undefined });
  await last();
}


export async function slamGoTo(id: string) {
  if (await slamCmd('/api/drive/goal', { node: id })) driveAssist(true);
}

export const slamTaskAdd = (id: string) => slamCmd('/api/task/add', { node: id });
export const slamTaskRemove = (id: string) => slamCmd('/api/task/delete', { node: id });

export async function slamTaskClear() {
  for (const id of [...(useSlamnav.getState().state?.task.nodes ?? [])]) if (!(await slamTaskRemove(id))) return;
}

export async function slamTaskStart() {
  if (await slamCmd('/api/task/start')) driveAssist(true);
}

export function slamTaskPauseResume() {
  const paused = useSlamnav.getState().state?.ctrl.state === 'pause';
  if (paused) slamCmd('/api/task/resume', {}, () => driveAssist(true));
  else slamCmd('/api/task/pause');
}

export function slamTaskCancel() {
  slamCmd('/api/task/cancel');
  driveAssist(false);
}

export const tapGroundZ = () => groundZ(useSlamnav.getState().state);
