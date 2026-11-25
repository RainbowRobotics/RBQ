#!/usr/bin/env python3
"""Generate include/zn/TypeInfoTable.hpp: the ROS 2 identity (DDS type name + REP-2011 type hash) of every
message struct in idl/{ros2,rbq,api}.

Run inside a ROS 2 Jazzy environment where rbq_msgs and api are built (scripts/gen_zn_type_info.bash does that);
the hashes are read from the type-description JSON rosidl installs next to each message, so they are exactly
what rmw_zenoh puts in its keys. A struct without a ROS counterpart (services, unitree types) is skipped and listed.

    gen_zn_type_info.py <idl root> <output header> <share dir>...
"""
import json
import re
import sys
from pathlib import Path

MODULE = re.compile(r'\bmodule\s+(\w+)\s*\{|\bstruct\s+(\w+)\s*\{([^{}]*)\}|(\{)|(\})')

# rosidl FieldType ids (type_description_interfaces/msg/FieldType): +48 fixed array, +144 unbounded sequence.
PRIMITIVE = {'int8': 2, 'uint8': 3, 'int16': 4, 'uint16': 5, 'int32': 6, 'uint32': 7, 'int64': 8, 'uint64': 9,
             'float': 10, 'double': 11, 'char': 13, 'boolean': 15, 'octet': 16, 'string': 17}
# Classic IDL spellings, longest first.
IDL_WORDS = [('unsigned long long', 'uint64'), ('long long', 'int64'), ('unsigned long', 'uint32'), ('long', 'int32'),
             ('unsigned short', 'uint16'), ('short', 'int16')]
# [sequence<ELEM[, BOUND]> | ELEM] name[ [N] ];   ELEM = type or string<N>
ELEM = r'[\w:]+(?:<\s*\d+\s*>)?'
MEMBER = re.compile(rf'^(?:sequence<\s*({ELEM})\s*(?:,\s*(\d+))?\s*>|({ELEM}))\s+(\w+)\s*(?:\[(\d+)\])?\s*;$')
STRING_BOUND = re.compile(r'^string<\s*(\d+)\s*>$')
CONSTANT = re.compile(r'\bconst\s+[\w\s]+?\s+([A-Z_][A-Z0-9_]*)\s*=\s*(\d+)\s*;')
CONSTANTS = {}   # IDL `const` names used as array bounds -> value, filled from every IDL file first


def structs(idl_text):
    """Yield (module path, struct name, body) for every struct, tracking nesting by braces."""
    stack = []  # entries: module name, or None for any other brace
    for m in MODULE.finditer(re.sub(r'//[^\n]*', '', idl_text)):
        mod, struct, body, opn, cls = m.groups()
        if mod:
            stack.append(mod)
        elif struct:
            yield [s for s in stack if s], struct, body
        elif opn:
            stack.append(None)
        elif cls and stack:
            stack.pop()


def idl_fields(body, pkg):
    """[(name, type_id, capacity, string_capacity, nested_type_name)] of an IDL struct body, as rosidl describes
    them."""
    out = []
    for line in (l.strip() for l in body.splitlines()):
        if not line or line.startswith('@'):
            continue
        for word, plain in IDL_WORDS:
            line = re.sub(rf'(^|<\s*){word}\b', rf'\g<1>{plain}', line)
        line = re.sub(r'\b(?:[\w]+::)*([A-Z_][A-Z0-9_]*)\b', lambda c: CONSTANTS.get(c.group(1), c.group(0)), line)
        m = MEMBER.match(line)
        if not m:
            raise ValueError(f'unparsed IDL member: {line}')
        seq_elem, seq_bound, plain, name, cap = m.groups()
        elem = seq_elem or plain
        str_cap = 0
        if STRING_BOUND.match(elem):
            tid, nested, str_cap = 21, '', int(STRING_BOUND.match(elem).group(1))   # BOUNDED_STRING
        elif elem in PRIMITIVE:
            tid, nested = PRIMITIVE[elem], ''
        else:
            parts = elem.split('::')   # pkg::msg::dds_::Name_, or Name_ inside its own module
            pkg_of = parts[0] if len(parts) > 1 else pkg
            tid, nested = 1, f'{pkg_of}/msg/{parts[-1].rstrip("_")}'
        if seq_elem:
            tid += 96 if seq_bound else 144
            cap = seq_bound
        elif cap:
            tid += 48
        out.append((name, tid, int(cap or 0), str_cap, nested))
    return out


ONE_BYTE_INTS = {2, 3, 13, 16}   # int8, uint8, char, byte: one byte each, identical CDR


def wire(field):
    """A field as CDR sees it: the 1-byte integer kinds collapse into one."""
    name, tid, cap, str_cap, nested = field
    base, container = tid % 48, tid - tid % 48
    if base in ONE_BYTE_INTS:
        base = 3
    return name, container + base, cap, str_cap, nested


def ros_fields(type_description):
    return [(f['name'], f['type']['type_id'], f['type']['capacity'], f['type']['string_capacity'],
             f['type']['nested_type_name']) for f in type_description['fields']]


def main():
    idl_root, out, shares = Path(sys.argv[1]), Path(sys.argv[2]), [Path(p) for p in sys.argv[3:]]
    entries, skipped, mismatches, notes = [], [], [], []
    seen = set()
    idls = [p for p in sorted(idl_root.glob('*/*.idl')) if p.parent.name in ('ros2', 'rbq', 'api')]
    for idl in idls:
        CONSTANTS.update({n: v for n, v in CONSTANT.findall(idl.read_text())})
    for idl in idls:
        for mods, name, body in structs(idl.read_text()):
            if len(mods) != 3 or mods[1] != 'msg' or mods[2] != 'dds_' or not name.endswith('_'):
                skipped.append(f'{"::".join(mods)}::{name}')
                continue
            pkg, ros = mods[0], name[:-1]
            if (pkg, ros) in seen:
                continue
            seen.add((pkg, ros))
            js = next((s / pkg / 'msg' / f'{ros}.json' for s in shares if (s / pkg / 'msg' / f'{ros}.json').exists()), None)
            if js is None:
                skipped.append(f'{pkg}/msg/{ros} (no type description)')
                continue
            data = json.loads(js.read_text())
            full = f'{pkg}/msg/{ros}'
            # The IDL and the .msg must describe the same fields, or the bytes stop matching while the hash still does.
            want = ros_fields(data['type_description_msg']['type_description'])
            have = idl_fields(body, pkg)
            if want != have and [wire(f) for f in want] == [wire(f) for f in have]:
                notes.append(f'{full}: a 1-byte field is spelled differently from the .msg (same bytes on the wire)')
            elif want != have:
                mismatches.append(f'{full} ({idl.relative_to(idl_root)}):\n    ros: {want}\n    idl: {have}')
            h = next(t['hash_string'] for t in data['type_hashes'] if t['type_name'] == full)
            entries.append((pkg, name, f'{pkg}::msg::dds_::{name}', h))

    if mismatches:
        print('IDL does not match the ROS message definition:', file=sys.stderr)
        for m in mismatches:
            print('  ' + m, file=sys.stderr)
        sys.exit(1)

    lines = [
        '#pragma once',
        '',
        '// GENERATED by scripts/gen_zn_type_info.bash (ROS 2 Jazzy type descriptions) — do not edit.',
        '// Regenerate whenever an IDL under idl/ changes or a type is added.',
        '',
        '#include "TypeInfo.hpp"',
        '',
    ]
    for pkg, name, _, _ in entries:
        lines.append(f'namespace {pkg}::msg::dds_ {{ class {name}; }}')
    lines.append('')
    for pkg, name, dds, h in entries:
        lines.append(f'RBQ_ZN_TYPE_INFO(::{dds}, "{dds}", "{h}")')
    out.write_text('\n'.join(lines) + '\n')
    print(f'{len(entries)} types -> {out}')
    for s in skipped:
        print(f'  skipped {s}')
    for n in notes:
        print(f'  note {n}')


if __name__ == '__main__':
    main()
