"""Make a Cyclone DDS 11 SDK describe its types the way the robot (Cyclone DDS 0.10) does.

Before they exchange data, DDS peers swap a description of each type (XTypes type information).
11 writes int8/uint8 members with the type codes INT8/UINT8. 0.10 cannot parse those codes, so the
robot rejects every topic that has such a member: RPC replies never arrive (3102), and LED, battery,
image and other topics stay silent. 0.10 itself describes uint8 as BYTE and int8 as CHAR8, so we do
the same:

1. build each struct's wire (de)serializers from the real int8/uint8 hints, so values stay ints,
2. only then swap the hints the XTypes builder reads to byte/char.

Step 2 alone breaks: Python 11 reads char as a 1-character str, and chr() raises on negative int8
values (gait_state -1 = CONTROL_OFF). The robot is not changed; ROS 2 Humble/Jazzy are still on 0.10.

This relies on cyclonedds 11.0.1 internals (`__idl__.populate`, `__idl_type_annotations__`), which is
why pyproject.toml pins the exact version. scripts/docker/interop.py (run by the PR check) breaks
if they move.
"""
from cyclonedds.idl import IdlStruct, IdlUnion, types
from cyclonedds.idl._type_normalize import get_extended_type_hints

_done = set()


def _walk(hint, found):
    """Return `hint` with int8/uint8 replaced by char/byte; collect the struct classes it uses."""
    if hint == types.uint8:
        return types.byte
    if hint == types.int8:
        return types.char
    if isinstance(hint, types.sequence):
        return types.sequence(_walk(hint.subtype, found), hint.max_length)
    if isinstance(hint, types.array):
        return types.array(_walk(hint.subtype, found), hint.length)
    if isinstance(hint, type) and issubclass(hint, (IdlStruct, IdlUnion)):
        found.append(hint)
    return hint


def advertise_like_robot(datatype):
    """Call before creating a Topic for `datatype`. Idempotent and covers nested structs.

    ponytail: a Topic created for one of these types *before* this call keeps the 11 type codes.
    The SDK creates all its topics in Channel.__init__, which calls this first.
    """
    todo, closure = [datatype], []
    while todo:  # phase 1: find every struct reachable from datatype, still with its real hints
        cls = todo.pop()
        if cls in _done or cls in closure:
            continue
        closure.append(cls)
        for hint in get_extended_type_hints(cls).values():
            _walk(hint, todo)
    for cls in closure:  # phase 2: build all wire machines before any hint changes
        cls.__idl__.populate()
    for cls in closure:  # phase 3: from now on only the type description sees byte/char
        hints = get_extended_type_hints(cls)
        for name, hint in hints.items():
            hints[name] = _walk(hint, [])
        _done.add(cls)
