import { useMemo, useState } from 'react';
import { ScrollView, View, Text, StyleSheet, TextInput, Pressable } from 'react-native';
import { useTheme } from '@/theme';
import { chan, chanOpt, type BlackboxSession } from '@/lib/blackbox';
import { groupChannels, filterGroups, type ChannelGroup } from '@/lib/blackboxChannels';
import { t } from '@/lib/i18n';

const R2D = 180 / Math.PI;
const COL_W = 86;

const RAD_STEMS = new Set(['joint.pos', 'ref.joint.pos', 'wheel.joint.pos']);
const isRad = (stem: string) => RAD_STEMS.has(stem) || stem.startsWith('imu.rpy');

const isFault = (name: string) => /(\.err|err\.|jam|bus_off|overrun|_miss|throttled)/.test(name);

function fmt(v: number | undefined): string {
  if (v === undefined) return '';
  if (!Number.isFinite(v)) return '—';
  if (v === 0) return '0';
  const a = Math.abs(v);
  return a >= 1000 ? v.toFixed(0) : a >= 100 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : v.toFixed(3);
}

const secTitle = (title: string) => t(title).toUpperCase().replace(/^(.+) · \1$/, '$1');

function Section({ title, count, open, onToggle, children }: {
  title: string; count: number; open: boolean; onToggle: () => void; children: React.ReactNode;
}) {
  const { c } = useTheme();
  return (
    <View style={styles.sec}>
      <Pressable onPress={onToggle} style={styles.secHead}>
        <Text style={{ color: c.dim, fontSize: 11, width: 11 }}>{open ? '▾' : '▸'}</Text>
        <Text style={{ color: c.accent2, fontSize: 10.5, fontWeight: '800', letterSpacing: 0.7 }}>
          {secTitle(title)}
        </Text>
        <Text style={{ color: c.dim, fontSize: 9.5 }}>{count}ch</Text>
        <View style={[styles.secRule, { backgroundColor: c.line2 }]} />
      </Pressable>
      {open ? children : null}
    </View>
  );
}

function Matrix({ g, sess, frame }: { g: Extract<ChannelGroup, { kind: 'matrix' }>; sess: BlackboxSession; frame: number }) {
  const { c, fonts } = useTheme();
  const cell = (txt: string, w: number, color?: string, bold?: boolean) => (
    <Text numberOfLines={1} style={{
      width: w, color: color ?? c.text, fontFamily: fonts.mono, fontSize: 10.5,
      fontWeight: bold ? '700' : '400', textAlign: 'right', paddingHorizontal: 3,
    }}>{txt}</Text>
  );
  return (
    <ScrollView horizontal showsHorizontalScrollIndicator={false} contentContainerStyle={{ paddingBottom: 2 }}>
      <View>
        <View style={[styles.tr, { borderBottomWidth: 1, borderBottomColor: c.line2, paddingBottom: 2 }]}>
          {cell('', 34)}
          {g.cols.map((col, i) => (
            <Text key={col} numberOfLines={1} style={{
              width: COL_W, color: c.muted, fontFamily: fonts.mono, fontSize: 9,
              fontWeight: '700', textAlign: 'right', paddingHorizontal: 3,
            }}>{g.colLabels[i]}{isRad(col) ? '°' : ''}</Text>
          ))}
        </View>
        {g.rows.map((row) => (
          <View key={row} style={styles.tr}>
            {cell(row, 34, c.muted, true)}
            {g.cols.map((col) => {
              const name = g.name(row, col);
              if (!name) return cell('·', COL_W, c.line);
              const raw = chanOpt(sess, frame, name);
              const v = raw !== undefined && isRad(col) ? raw * R2D : raw;
              const bad = raw !== undefined && raw !== 0 && isFault(name);
              return cell(fmt(v), COL_W, bad ? c.redTx : undefined);
            })}
          </View>
        ))}
      </View>
    </ScrollView>
  );
}

function Chips({ g, sess, frame }: { g: Extract<ChannelGroup, { kind: 'matrix' }>; sess: BlackboxSession; frame: number }) {
  const { c, fonts } = useTheme();
  const col = g.cols[0];
  return (
    <View style={styles.grid}>
      {g.rows.map((row) => {
        const name = g.name(row, col);
        const v = name ? chanOpt(sess, frame, name) : undefined;
        const on = (v ?? 0) !== 0;
        return (
          <Text key={row} style={{
            color: on ? c.greenTx : c.dim, fontFamily: fonts.mono, fontSize: 10.5, minWidth: 62,
          }}>{row}:{fmt(v)}</Text>
        );
      })}
    </View>
  );
}

function Scalars({ g, sess, frame }: { g: Extract<ChannelGroup, { kind: 'scalars' }>; sess: BlackboxSession; frame: number }) {
  const { c, fonts } = useTheme();
  return (
    <View style={styles.grid}>
      {g.names.map((name) => {
        const raw = chanOpt(sess, frame, name);
        const v = raw !== undefined && isRad(name.replace(/\.[^.]+$/, '')) ? raw * R2D : raw;
        const bad = raw !== undefined && raw !== 0 && isFault(name);
        return (
          <View key={name} style={styles.kv}>
            <Text numberOfLines={1} style={{ color: c.muted, fontFamily: fonts.mono, fontSize: 10.5, flexShrink: 1 }}>{name}</Text>
            <Text style={{ color: bad ? c.redTx : c.text, fontFamily: fonts.mono, fontSize: 10.5, fontWeight: '600' }}>
              {fmt(v)}{isRad(name.replace(/\.[^.]+$/, '')) ? '°' : ''}
            </Text>
          </View>
        );
      })}
    </View>
  );
}

export function BlackBoxDataPanel({ sess, frame, topPad = true }: { sess: BlackboxSession; frame: number; topPad?: boolean }) {
  const { c, fonts, radius } = useTheme();
  const groups = useMemo(() => groupChannels(sess.cols.keys()), [sess]);
  const [q, setQ] = useState('');
  const [open, setOpen] = useState<Record<string, boolean>>({ joint: true });
  const shown = useMemo(() => filterGroups(groups, q), [groups, q]);
  const searching = q.trim().length > 0;

  const miss = chan(sess, frame, 'deadline_miss');
  const imuC = chanOpt(sess, frame, 'imu.connected');
  const total = useMemo(() => sess.cols.size, [sess]);

  return (
    <ScrollView style={[styles.wrap, { backgroundColor: c.panel2, borderColor: c.line, borderRadius: radius.md }]}
      contentContainerStyle={{ padding: 10, paddingTop: topPad ? 32 : 10 }} showsVerticalScrollIndicator={false}>
      <View style={{ flexDirection: 'row', gap: 12, marginBottom: 8, flexWrap: 'wrap', alignItems: 'center' }}>
        <Text style={{ color: chanOpt(sess, frame, 'deadline_miss') === undefined ? c.muted : miss > 0 ? c.redTx : c.greenTx,
                       fontFamily: fonts.mono, fontSize: 11 }}>miss {fmt(chanOpt(sess, frame, 'deadline_miss'))}</Text>
        <Text style={{ color: c.text, fontFamily: fonts.mono, fontSize: 11 }}>proc {fmt(chanOpt(sess, frame, 'process_time_ms'))}ms</Text>
        <Text style={{ color: imuC === undefined ? c.muted : imuC > 0 ? c.greenTx : c.redTx, fontFamily: fonts.mono, fontSize: 11 }}>
          IMU {imuC === undefined ? '-' : imuC > 0 ? 'OK' : '—'}
        </Text>
        <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 10 }}>{total}ch</Text>
      </View>

      <TextInput value={q} onChangeText={setQ} placeholder={t('채널 검색 (예: motor.cur, joint, can)')}
        placeholderTextColor={c.dim} autoCapitalize="none" autoCorrect={false}
        style={[styles.search, { color: c.text, backgroundColor: c.elev, borderColor: c.line, fontFamily: fonts.mono }]} />

      {shown.length === 0 ? (
        <Text style={{ color: c.muted, fontSize: 11, marginTop: 8 }}>{t('검색 결과 없음')}</Text>
      ) : shown.map((g) => {
        const count = g.kind === 'scalars' ? g.names.length : g.rows.length * g.cols.length;
        const isOpen = searching || open[g.key] === true;
        return (
          <Section key={g.key} title={g.title} count={count} open={isOpen}
            onToggle={() => setOpen((o) => ({ ...o, [g.key]: !isOpen }))}>
            {g.kind === 'scalars' ? <Scalars g={g} sess={sess} frame={frame} />
              : g.cols.length === 1 ? <Chips g={g} sess={sess} frame={frame} />
                : <Matrix g={g} sess={sess} frame={frame} />}
          </Section>
        );
      })}
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  wrap: { flex: 1, borderWidth: 1 },
  search: { borderWidth: 1, borderRadius: 7, paddingHorizontal: 8, paddingVertical: 4, fontSize: 11, marginBottom: 10 },
  sec: { marginBottom: 11 },
  secHead: { flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 4 },
  secRule: { flex: 1, height: 1 },
  tr: { flexDirection: 'row', alignItems: 'center' },
  grid: { flexDirection: 'row', flexWrap: 'wrap', rowGap: 2, columnGap: 16 },
  kv: { flexDirection: 'row', gap: 6, minWidth: 210, justifyContent: 'space-between' },
});
