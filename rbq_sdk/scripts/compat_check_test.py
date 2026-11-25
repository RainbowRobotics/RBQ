#!/usr/bin/env python3
"""Self-test of compat_check.py's comparisons — `python3 rbq_sdk/scripts/compat_check_test.py`, no framework."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compat_check as c  # noqa: E402

J, M, W = ("rbq_msgs", "JointRef_"), ("rbq_msgs", "MotionRef_"), ("rbq_msgs", "Whole_")
OLD = [("pos", "float32"), ("torque", "float32")]
NEW = [("pos", "float32"), ("vel", "float32"), ("torque", "float32")]


def broken(base, now):
    return c.compare(base, now, c.idl_refs(now))


# FINAL: a field added anywhere breaks; APPENDABLE: added at the end passes, in the middle breaks
assert J in broken({J: ("final", OLD)}, {J: ("final", NEW)})
assert "added vel (float32) as field 2 of 3" in broken({J: ("final", OLD)}, {J: ("final", NEW)})[J]
assert not broken({J: ("appendable", OLD)}, {J: ("appendable", OLD + [("vel", "float32")])})
assert J in broken({J: ("appendable", OLD)}, {J: ("appendable", NEW)})
assert J in broken({J: ("final", OLD)}, {J: ("appendable", OLD)})  # extensibility changed
assert J in broken({J: ("final", OLD)}, {})  # removed

# a struct holding a broken one breaks too — through an array, a sequence and two levels down
holder = {M: ("final", [("time", "float64"), ("leg_joint", "JointRef_[12]")]),
          W: ("final", [("refs", "sequence<MotionRef_,4>")])}
got = broken({J: ("final", OLD), **holder}, {J: ("final", NEW), **holder})
assert got[M] == "via JointRef_ (leg_joint)", got
assert got[W] == "via MotionRef_ (refs)", got
assert not broken({J: ("final", OLD), **holder}, {J: ("final", OLD), **holder})

# a release topic missing now breaks; one now built at run time ("..._" + i) or by a prefix still counts
base = {"rt/rbq/old": {"a.cpp"}, "rt/rbq/ref/motion/_20": {"b.py"}, "rt/api/*": {"Client.hpp"}}
now = {"rt/rbq/ref/motion/_*": {"DdsBridge.cpp"}, "rt/api/sport/request": {"DdsBridge.cpp"}}
assert list(c.topic_breaks(base, now)) == ["topic:rt/rbq/old"], c.topic_breaks(base, now)

# allow list: a reason is required
assert c.parse_allow(["# comment", "", "idl:a::B_  # #1 why"]) == {"idl:a::B_": "#1 why"}
for bad in (["idl:a::B_"], ["idl:a::B_  #"], ["idl:a::B_ idl:a::C_  # #1 two ids"]):
    try:
        c.parse_allow(bad)
    except ValueError:
        continue
    raise AssertionError(f"accepted {bad}")

# the tree against itself: every parser reads the real files and finds nothing
assert c.find_breaks(c.ROOT, c.ROOT)[0] == {}

print("compat_check_test.py: all passed")
