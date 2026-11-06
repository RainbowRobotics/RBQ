"""SDK (Cyclone DDS 11) <-> robot (Cyclone DDS 0.10.2): every SDK message type, both directions.

    python3 interop.py robot   # cyclonedds 0.10.2, plain DDS Topics — stands in for the robot's Network
    python3 interop.py sdk     # cyclonedds 11, the SDK's own ChannelPublisher/ChannelSubscriber

interop.bash runs the two in separate containers on one Docker network. Each side writes one sample
of every type on interop/<side>/<type> and must read the other side's sample, with the same values,
before the timeout. int8 members carry -1 and uint8 members 255: a type-code mismatch makes the robot
side drop the topic (or crash, 0.10's Python), and a char/int mix-up changes or rejects the value.
Exit 0 = every type went both ways.
"""
import enum
import os
import pkgutil
import sys
import time
from importlib import import_module
from inspect import isclass

from cyclonedds.idl import IdlStruct, types
from cyclonedds.idl._type_normalize import get_extended_type_hints

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "example", "idl"))  # _rbq_idl: the examples' own types
import rbq_sdk_py.idl  # noqa: E402

ROLE = sys.argv[1]
PEER = {"robot": "sdk", "sdk": "robot"}[ROLE]
DOMAIN = int(os.environ.get("INTEROP_DOMAIN", "0"))
TIMEOUT = float(os.environ.get("INTEROP_TIMEOUT", "60"))


def all_types():
    mods = [import_module("_rbq_idl")]
    for m in pkgutil.walk_packages(rbq_sdk_py.idl.__path__, "rbq_sdk_py.idl."):
        mods.append(import_module(m.name))
    found = {}
    for mod in mods:
        for obj in vars(mod).values():
            if isclass(obj) and issubclass(obj, IdlStruct) and obj.__module__ == mod.__name__:
                found[obj.__idl_typename__ if hasattr(obj, "__idl_typename__") else obj.__name__] = obj
    return found


def sample(hint):
    """A sample with a recognizable value in every member, built from the declared (int8/uint8) hints."""
    if hint == types.int8:
        return -1
    if hint == types.uint8:
        return 255
    if isinstance(hint, types.sequence):
        return [sample(hint.subtype)] * min(2, hint.max_length or 2)
    if isinstance(hint, types.array):
        return [sample(hint.subtype)] * hint.length
    if isinstance(hint, types.bounded_str):
        return "a"
    if hint is str:
        return "rbq"
    if hint is bool:
        return True
    if isclass(hint) and issubclass(hint, enum.Enum):
        return list(hint)[-1]
    if isclass(hint) and issubclass(hint, IdlStruct):
        return hint(**{k: sample(v) for k, v in get_extended_type_hints(hint).items()})
    if hint in (types.float32, types.float64, float):
        return 1.5
    if hint in (int, types.int16, types.int32, types.int64, types.uint16, types.uint32, types.uint64):
        return 7
    raise TypeError(f"interop.py does not know how to fill {hint!r} — add it to sample()")


def plain(v):
    """Compare by value: bytes vs list, enum vs int and struct vs struct all differ between versions."""
    if isinstance(v, (bytes, bytearray)):
        return list(v)
    if isinstance(v, (list, tuple)):
        return [plain(x) for x in v]
    if isinstance(v, enum.Enum):
        return v.value
    if isinstance(v, IdlStruct):
        return {k: plain(getattr(v, k)) for k in get_extended_type_hints(type(v))}
    return v


def topic(side, name):
    return f"interop/{side}/{name.replace('.', '_').replace(':', '_')}"


def main():
    only = os.environ.get("INTEROP_ONLY", "")  # debugging: a substring of the type names to keep
    typemap = {n: t for n, t in all_types().items() if only in n}
    expected = {n: sample(t) for n, t in typemap.items()}  # before any channel swaps hints (dds_compat)
    want = {n: plain(s) for n, s in expected.items()}
    got, bad = {}, {}

    def check(name, s):
        if name in got:
            return
        if plain(s) == want[name]:
            got[name] = True
        else:
            bad[name] = plain(s)

    if ROLE == "sdk":
        from rbq_sdk_py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        ChannelFactoryInitialize(DOMAIN, os.environ.get("INTEROP_IFACE"))
        pubs, subs = {}, []
        for n, t in typemap.items():
            sub = ChannelSubscriber(topic(PEER, n), t)
            sub.Init(lambda s, n=n: check(n, s))
            subs.append(sub)
            pubs[n] = ChannelPublisher(topic(ROLE, n), t)
            pubs[n].Init()
        write = lambda n: pubs[n].Write(expected[n])  # noqa: E731
        poll = lambda: None  # noqa: E731
    else:
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.pub import DataWriter
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        iface = os.environ.get("INTEROP_IFACE")
        if iface:  # the robot's own setting: one interface, no multicast on lo
            from cyclonedds.domain import Domain
            from rbq_sdk_py.core.channel_config import ChannelConfigHasInterface as cfg
            # keep the reference: a collected Domain drops this config and the participant falls back to the default one
            domain = Domain(DOMAIN, cfg.replace("$__IF_NAME__$", iface).replace(  # noqa: F841
                "$__IF_MULTICAST__$", "false" if iface == "lo" else "default"))
        dp = DomainParticipant(DOMAIN)
        writers, readers = {}, {}
        for n, t in typemap.items():
            readers[n] = DataReader(dp, Topic(dp, topic(PEER, n), t))
            writers[n] = DataWriter(dp, Topic(dp, topic(ROLE, n), t))
        write = lambda n: writers[n].write(expected[n])  # noqa: E731

        def poll():
            for n, r in readers.items():
                for s in r.take(N=4):
                    check(n, s)

    print(f"{ROLE}: {len(typemap)} types, domain {DOMAIN}", flush=True)
    t0, done_at = time.time(), None
    while time.time() - t0 < TIMEOUT:
        for n in typemap:
            write(n)
        poll()
        if len(got) == len(typemap) and done_at is None:
            done_at = time.time()
            print(f"{ROLE}: received all {len(got)} after {done_at - t0:.1f} s", flush=True)
        if done_at and time.time() - done_at > 5:  # keep writing a little so the peer can finish too
            break
        time.sleep(0.2)

    missing = sorted(set(typemap) - set(got))
    for n in sorted(bad):
        print(f"{ROLE}: WRONG VALUE {n}: {bad[n]}  expected {want[n]}", flush=True)
    for n in missing:
        print(f"{ROLE}: NOT RECEIVED {n}", flush=True)
    print(f"{ROLE}: {len(got)}/{len(typemap)} types ok", flush=True)
    sys.exit(0 if not missing else 1)


main()
