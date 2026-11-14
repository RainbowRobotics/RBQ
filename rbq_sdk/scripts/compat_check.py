#!/usr/bin/env python3
"""Does a program built on the last released SDK still work against the code as it is now?

    python3 rbq_sdk/scripts/compat_check.py                         # base = latest v* tag merged into HEAD
    python3 rbq_sdk/scripts/compat_check.py --base v1.2609.0 --json
    exit 0 no break outside the allow list / 1 at least one BREAK / 2 could not read or parse (git, IDL, allow list)

Why: on 11-12 develop 9a58ab383 put `float vel` into rbq_msgs::msg::dds_::JointRef_, an @extensibility(FINAL)
struct held by MotionRef_. A low-level program built with the v1.2609.0 SDK then no longer matches a new robot
on rt/rbq/ref/motion/_20.._39 (#1333) — DDS drops the pair without a word. idl_check.py and docker/interop.bash
compare the SDK and the robot of one commit, so they could not see it. This compares the release tag's rbq_sdk/
(read with `git archive`, nothing is checked out) with the working tree. The nightly runs it; it gates no PR.

What it reports, one stable id per finding:
  idl:<module>::<struct>  rbq_sdk/cpp/rbq_sdk_cpp/idl/**/*.idl, parsed by rbq_sdk/python/scripts/idl_check.py
        (<module> is its package name: IDL module `api` is rbq_api). Broken: the struct is gone, its
        extensibility changed, FINAL with any field different (name, order, type, array length, sequence
        bound), APPENDABLE whose release fields are no longer the leading fields, MUTABLE with a release field
        gone or retyped. A struct holding a broken struct — directly, in an array or a sequence, at any depth —
        is broken too ("via JointRef_ (leg_joint, ...)"). The detail names the topics that carry it, from
        Publisher<T>/Subscriber<T> in src/Network/*.cpp (best effort, "topics: unknown" when none is found).
  ros2:<pkg>/<msg|srv>/<name>  rbq_sdk/ros2/src/*/{msg,srv}/: any field different (rmw_cyclonedds makes them
        FINAL), and a type holding a broken one. Constants are not on the wire and are not compared.
  topic:<name>  an "rt/..." string in the release's rbq_sdk/ code (C++, examples, Python) that is in neither
        src/Network nor rbq_sdk/ now — a topic removed or renamed. A string built at run time
        ("rt/api/" + name, f"..._{i}") counts as its prefix and is written <prefix>*.
  api:<NAME>  a service API number constant (*API_ID_*) in the C++ headers or in rbq_sdk_py: gone or revalued.
  func:cpp:<Class>[::<method>]  a *Client class in rbq_sdk/cpp/rbq_sdk_cpp/include, or a public method of it, gone.
  func:py:<module>[.<Class>[.<method>]]  a public module (file) of rbq_sdk_py, a *Client class in it or a public
        method of that class, gone. [[deprecated]] or a FutureWarning still counts as there — deprecate first.

Allow list rbq_sdk/compat-allow.txt: one id per line, then "#" and the reason with its issue number
(`idl:rbq_msgs::JointRef_  # #1333 — ...`); a line starting with "#" is a comment. A line without a reason is an
error (exit 2). A listed break prints ALLOWED and passes. A listed id that no longer breaks prints STALE —
delete that line; it does not change the exit code. List only breaks the owner decided on; once a release ships
one, that release becomes the base, the break stops showing and the line turns STALE.

Not seen here: behaviour — same names and types with another meaning (gait numbers, response codes, units, what
a command does) is the sim scenarios' job. Neither are: overloads (one overload left passes), a method moved to
a base class (reported as gone), C++ headers and free functions outside *Client classes, QoS, topics only used
outside rbq_sdk/ and src/Network (e.g. vision topics from HAL/Streamer). Standard library only (no venv or PyYAML
on the servers).
"""
import argparse
import ast
import functools
import importlib.util
import io
import json
import os
import re
import subprocess
import sys
import tarfile
import tempfile

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
IDL = "rbq_sdk/cpp/rbq_sdk_cpp/idl"
ROS2 = "rbq_sdk/ros2/src"
CPP_INC = "rbq_sdk/cpp/rbq_sdk_cpp/include"
PY_PKG = "rbq_sdk/python/rbq_sdk_py"
NETWORK = "src/Network"
SELF = "rbq_sdk/scripts/"  # this check and its test: their own "rt/..." strings are not SDK topics
ALLOW = "rbq_sdk/compat-allow.txt"
CODE = (".h", ".hpp", ".cpp", ".cc", ".py")


@functools.lru_cache(maxsize=None)
def idl_check():
    path = os.path.join(ROOT, "rbq_sdk", "python", "scripts", "idl_check.py")
    spec = importlib.util.spec_from_file_location("idl_check", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def git(*args):
    r = subprocess.run(["git", *args], cwd=ROOT, capture_output=True)
    if r.returncode:
        raise ValueError(f"git {' '.join(args)}: {r.stderr.decode(errors='replace').strip()}")
    return r.stdout


def latest_tag():
    tags = git("tag", "--merged", "HEAD", "--sort=-creatordate", "v[0-9]*").decode().split()
    if not tags:
        raise ValueError("no v* tag merged into HEAD (a clone without tags? fetch them, or pass --base)")
    return tags[0]


def extract(rev, dest):
    """rbq_sdk/ of rev into dest — only the kinds of file this check reads."""
    tar = tarfile.open(fileobj=io.BytesIO(git("archive", rev, "rbq_sdk")))
    for m in tar.getmembers():
        if m.isfile() and m.name.endswith(CODE + (".idl", ".msg", ".srv")):
            path = os.path.join(dest, m.name)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "wb") as f:
                f.write(tar.extractfile(m).read())


def files(root, rel, exts):
    """[(path relative to root, text)] of the files under root/rel ending in exts, sorted."""
    out = []
    for d, dirs, names in os.walk(os.path.join(root, rel)):
        dirs.sort()
        for n in sorted(names):
            r = os.path.relpath(os.path.join(d, n), root).replace(os.sep, "/")
            if n.endswith(exts) and not r.startswith(SELF):
                out.append((r, open(os.path.join(d, n), encoding="utf-8", errors="replace").read()))
    return out


CPP_TOK = re.compile(r'"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'|//[^\n]*|/\*.*?\*/', re.S)


def strip_comments(text):
    return CPP_TOK.sub(lambda m: m.group(0) if m.group(0)[0] in "\"'" else " ", text)


def mask_strings(text):
    """String and char literals as "___" of the same length, so braces in them don't count and offsets hold."""
    return CPP_TOK.sub(lambda m: m.group(0)[0] + "_" * (len(m.group(0)) - 2) + m.group(0)[-1], text)


# ── types: IDL structs and ROS 2 messages ───────────────────────────────────
def idl_structs(root):
    """(package, struct) -> (extensibility, [(field, type)]) — idl_check.py's parser run on root's IDL."""
    chk = idl_check()
    chk.IDL_DIR = os.path.join(root, IDL)
    return chk.parse_idl()


def idl_refs(structs):
    """key -> [(field, key of a struct the field holds)]. idl_check keeps only the last name of a scoped type,
    so a name is looked up in the holder's package first, then everywhere (no name is in two packages today)."""
    by_name = {}
    for k in structs:
        by_name.setdefault(k[1], []).append(k)
    refs = {}
    for key, (_, fields) in structs.items():
        refs[key] = []
        for field, t in fields:
            for word in re.findall(r"[A-Za-z_]\w*", t):
                cands = by_name.get(word, [])
                refs[key] += [(field, c) for c in [c for c in cands if c[0] == key[0]] or cands]
    return refs


def ros2_types(root):
    """((pkg, msg|srv, name) -> ("final", [(field, type)]), refs as in idl_refs). srv response fields are response.<f>."""
    types, refs = {}, {}
    for rel, text in files(root, ROS2, (".msg", ".srv")):
        m = re.search(r"([^/]+)/(msg|srv)/([^/]+)\.(?:msg|srv)$", rel)
        if not m:
            continue
        key, fields, prefix = m.groups(), [], ""
        for line in text.splitlines():
            line = line.split("#")[0].strip()
            if line == "---":
                prefix = "response."
                continue
            words = line.split(None, 2)
            if len(words) < 2 or "=" in line.split(None, 1)[1]:  # blank, or a constant (TYPE NAME=value)
                continue
            fields.append((prefix + words[1], words[0]))
        types[key] = ("final", fields)
        refs[key] = []
        for field, t in fields:
            base = re.sub(r"\[.*?\]|<=\d+", "", t)
            pkg, _, name = base.rpartition("/")
            refs[key].append((field, (pkg or key[0], "msg", name)))
    return types, refs


def field_diff(fb, fn):
    b, n = dict(fb), dict(fn)
    out = [f"added {f} ({t}) as field {fn.index((f, t)) + 1} of {len(fn)}" for f, t in fn if f not in b]
    out += [f"removed {f} ({t})" for f, t in fb if f not in n]
    out += [f"{f}: {b[f]} -> {n[f]}" for f, _ in fb if f in n and b[f] != n[f]]
    if [f for f, _ in fb if f in n] != [f for f, _ in fn if f in b]:
        out.append("field order changed")
    return "; ".join(out)


def type_break(base, now):
    """Why a type no longer matches its release self, None if it does. base, now = (extensibility, fields)."""
    (eb, fb), (en, fn) = base, now
    if eb != en:
        return f"extensibility {eb} -> {en}"
    if eb == "appendable":
        bad = fn[:len(fb)] != fb
    elif eb == "mutable":
        bad = any(dict(fn).get(f) != t for f, t in fb)
    else:
        bad = fb != fn
    return f"{eb.upper()}: {field_diff(fb, fn)}" if bad else None


def compare(base, now, refs):
    """key -> why, for every release type a release program can no longer exchange with the code now."""
    broken = {}
    for key in sorted(base):
        why = "removed" if key not in now else type_break(base[key], now[key])
        if why:
            broken[key] = why
    grew = True
    while grew:  # a type holding a broken type is broken too, at any depth
        grew = False
        for key in sorted(base):
            if key in broken or key not in now:
                continue
            via = {}
            for field, r in refs.get(key, []):
                if r in broken:
                    via.setdefault(r, []).append(field)
            if via:
                broken[key] = "; ".join(f"via {r[-1]} ({', '.join(fs)})" for r, fs in via.items())
                grew = True
    return broken


def carried(key, refs, broken, direct):
    """Topics carrying key: its own (direct(key)) and those of every broken type that holds it."""
    holders = {}
    for k, rs in refs.items():
        for _, r in rs:
            holders.setdefault(r, set()).add(k)
    out, todo, seen = set(), [key], set()
    while todo:
        k = todo.pop()
        if k not in seen:
            seen.add(k)
            out |= direct(k)
            todo += [h for h in holders.get(k, ()) if h in broken]
    return out


def network_topics(root):
    """(package or None, struct) -> {"<topic> (Network pub|sub)"} from Publisher<T>/Subscriber<T> in src/Network."""
    pkg = idl_check().PKG
    out = {}
    for _, text in files(root, NETWORK, (".cpp",)):
        text = strip_comments(text)
        masked = mask_strings(text)
        alias = dict(re.findall(r"\b(?:namespace|using)\s+(\w+)\s*=\s*:*([\w:]+)\s*;", text))
        consts = dict(re.findall(r'\b(\w+)\s*=\s*"((?:rt/|/rb[qh]/)[^"]*)"\s*;', text))
        pat = r'"((?:rt/|/rb[qh]/)[^"]*)"(\s*\+)?' + "".join(f"|\\b({c})\\b" for c in consts)
        for h in re.finditer(r"\b(Publisher|Subscriber)\s*<\s*:*([\w:]+)\s*>", masked):
            depth, end = 0, len(text)
            for j in range(h.end(), len(masked)):  # the rest of the statement: the topic is a constructor argument
                c = masked[j]
                depth += c in "([{"
                depth -= c in ")]}"
                if (c == ";" and depth <= 0) or depth < 0:
                    end = j
                    break
            m = re.search(pat, text[h.end():end])
            if not m:
                continue
            topic = m.group(1) + ("*" if m.group(2) else "") if m.group(1) else consts[next(g for g in m.groups()[2:] if g)]
            t = alias.get(h.group(2), h.group(2))
            head, _, rest = t.partition("::")
            if rest and head in alias:
                t = alias[head] + "::" + rest
            parts = t.split("::")
            key = (pkg.get(parts[0], parts[0]) if len(parts) > 1 else None, parts[-1])
            out.setdefault(key, set()).add(f"{topic} (Network {'pub' if h.group(1) == 'Publisher' else 'sub'})")
    return out


# ── names: topics, API ids, functions ───────────────────────────────────────
TOPIC = re.compile(r"""(["'])(rt/[^"'\n]*)\1(\s*[+%])?""")


def topic_strings(root, dirs):
    """topic -> {file}; a string built at run time is its prefix + "*"."""
    out = {}
    for d in dirs:
        for rel, text in files(root, d, CODE):
            if not rel.endswith(".py"):
                text = strip_comments(text)
            for _, s, op in TOPIC.findall(text):
                cut = re.split(r"[{%]", s)[0]
                out.setdefault(cut + "*" if op or cut != s else s, set()).add(rel)
    return out


def topic_breaks(base, now):
    """base, now: topic -> files. A release topic is gone when no string now equals it, covers it (a prefix*)
    or, for a release prefix*, starts with it."""
    prefixes = [t[:-1] for t in now if t.endswith("*")]
    out = {}
    for t, where in sorted(base.items()):
        p = t.rstrip("*")
        if t in now or any(p.startswith(q) for q in prefixes) or (t.endswith("*") and any(n.startswith(p) for n in now)):
            continue
        out[f"topic:{t}"] = (f"used by {', '.join(sorted(where)[:3])} at the release; "
                             f"no such topic string in {NETWORK} or rbq_sdk/ now (removed or renamed)")
    return out


def api_ids(root):
    """NAME -> {value: file} for the *API_ID_* constants of the C++ headers and rbq_sdk_py."""
    out = {}
    for rel, text in files(root, CPP_INC, (".h", ".hpp")) + files(root, PY_PKG, (".py",)):
        text = re.sub(r"#[^\n]*", "", text) if rel.endswith(".py") else strip_comments(text)
        for name, value in re.findall(r"\b(\w*API_ID_\w+)\s*=\s*(\d+)\b", text):
            out.setdefault(name, {})[value] = rel
    return out


def api_breaks(base, now):
    out = {}
    for name, vals in sorted(base.items()):
        was = ", ".join(f"{v} in {f}" for v, f in sorted(vals.items()))
        if name not in now:
            out[f"api:{name}"] = f"removed (was {was})"
        elif set(vals) != set(now[name]):
            out[f"api:{name}"] = f"value changed: {'/'.join(sorted(vals))} -> {'/'.join(sorted(now[name]))}"
    return out


def cpp_clients(root):
    """*Client class -> its public method names, from the SDK's C++ headers (constructors left out)."""
    out = {}
    for rel, text in files(root, CPP_INC, (".h", ".hpp")):
        if "/nlohmann/" in rel:
            continue
        text = re.sub(r"\[\[.*?\]\]", " ", mask_strings(strip_comments(text)), flags=re.S)
        for m in re.finditer(r"\b(class|struct)\s+(\w*Client)\b[^;{]*\{", text):
            name, depth, top = m.group(2), 0, []
            for c in text[m.end():]:  # the class body with every nested body cut to ";"
                if c == "{":
                    top.append(";" if depth == 0 else "")
                    depth += 1
                elif c == "}":
                    if depth == 0:
                        break
                    depth -= 1
                elif depth == 0:
                    top.append(c)
            access, methods = "private" if m.group(1) == "class" else "public", set()
            for i, chunk in enumerate(re.split(r"\b(public|private|protected)\s*:(?!:)", "".join(top))):
                if i % 2:
                    access = chunk
                    continue
                for stmt in chunk.split(";") if access == "public" else ():
                    f = re.search(r"(?:^|[\s*&>:])(~?\w+)\s*\(", stmt)
                    if f and f.group(1) != name and not f.group(1).startswith("~"):
                        methods.add(f.group(1))
            out[name] = methods
    return out


def py_modules(root):
    """public module of rbq_sdk_py -> {*Client class: public method names}."""
    out = {}
    for rel, text in files(root, PY_PKG, (".py",)):
        parts = rel[len(os.path.dirname(PY_PKG)) + 1:-3].split("/")
        if parts[-1] == "__init__":
            parts = parts[:-1]
        if any(p.startswith("_") for p in parts):
            continue
        tree = ast.parse(text, rel)
        out[".".join(parts)] = {
            c.name: {f.name for f in c.body
                     if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef)) and not f.name.startswith("_")}
            for c in tree.body if isinstance(c, ast.ClassDef) and c.name.endswith("Client")}
    return out


def func_breaks(base_cpp, now_cpp, base_py, now_py):
    out = {}
    for cls, methods in sorted(base_cpp.items()):
        if cls not in now_cpp:
            out[f"func:cpp:{cls}"] = "C++ class removed"
        for m in sorted(methods - now_cpp.get(cls, methods)):
            out[f"func:cpp:{cls}::{m}"] = "public method removed"
    for mod, classes in sorted(base_py.items()):
        if mod not in now_py:
            out[f"func:py:{mod}"] = "public module removed"
            continue
        for cls, methods in sorted(classes.items()):
            now = now_py[mod].get(cls)
            if now is None:
                out[f"func:py:{mod}.{cls}"] = "class removed from the module"
            for m in sorted(methods - (methods if now is None else now)):
                out[f"func:py:{mod}.{cls}.{m}"] = "public method removed"
    return out


# ── all together ────────────────────────────────────────────────────────────
def find_breaks(base_root, now_root):
    """(id -> detail for every break, what was compared). base_root needs only rbq_sdk/."""
    pkg = idl_check().PKG
    b_idl, n_idl = idl_structs(base_root), idl_structs(now_root)
    i_refs = idl_refs(n_idl)
    i_broken = compare(b_idl, n_idl, i_refs)
    net = network_topics(now_root)
    i_direct = lambda k: net.get(k, set()) | net.get((None, k[1]), set())
    out = {}
    for k, why in i_broken.items():
        out[f"idl:{k[0]}::{k[1]}"] = f"{why}; topics: {', '.join(sorted(carried(k, i_refs, i_broken, i_direct))) or 'unknown'}"

    (b_ros, _), (n_ros, r_refs) = ros2_types(base_root), ros2_types(now_root)
    r_broken = compare(b_ros, n_ros, r_refs)
    r_direct = lambda k: i_direct((pkg.get(k[0], k[0]), k[2] + "_")) if k[1] == "msg" else set()
    for k, why in r_broken.items():
        out[f"ros2:{'/'.join(k)}"] = f"{why}; topics: {', '.join(sorted(carried(k, r_refs, r_broken, r_direct))) or 'unknown'}"

    b_top = topic_strings(base_root, ["rbq_sdk"])
    out.update(topic_breaks(b_top, topic_strings(now_root, [NETWORK, "rbq_sdk"])))
    b_api = api_ids(base_root)
    out.update(api_breaks(b_api, api_ids(now_root)))
    b_cpp, b_py = cpp_clients(base_root), py_modules(base_root)
    out.update(func_breaks(b_cpp, cpp_clients(now_root), b_py, py_modules(now_root)))

    n_func = sum(1 + len(m) for m in b_cpp.values()) + sum(1 + sum(1 + len(m) for m in c.values()) for c in b_py.values())
    compared = (f"{len(b_idl)} IDL structs, {len(b_ros)} ROS 2 types, {len(b_top)} topics, {len(b_api)} API ids, "
                f"{n_func} classes/methods/modules")
    return out, compared


def parse_allow(lines, where=ALLOW):
    """id -> reason. A line is "<id>  # <reason>"; a line starting with # is a comment."""
    allow = {}
    for n, line in enumerate(lines, 1):
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        ident, sep, reason = (x.strip() for x in s.partition("#"))
        if not sep or not reason:
            raise ValueError(f"{where}:{n}: {ident} has no reason — write '<id>  # <why, with the issue number>'")
        if len(ident.split()) != 1:
            raise ValueError(f"{where}:{n}: one id per line, got '{ident}'")
        allow[ident] = reason
    return allow


def main(argv=None):
    ap = argparse.ArgumentParser(description="Compare the last SDK release with the code now (see the docstring).")
    ap.add_argument("--base", help="tag or revision to compare with (default: latest v* tag merged into HEAD)")
    ap.add_argument("--json", action="store_true", help="the same report as JSON")
    args = ap.parse_args(argv)
    try:
        base = args.base or latest_tag()
        head = git("rev-parse", "--short", "HEAD").decode().strip()
        path = os.path.join(ROOT, ALLOW)
        allow = parse_allow(open(path, encoding="utf-8").read().splitlines()) if os.path.exists(path) else {}
        with tempfile.TemporaryDirectory() as tmp:
            extract(base, tmp)
            breaks, compared = find_breaks(tmp, ROOT)
    except Exception as e:  # a file this cannot read is a broken check, not "compatible" — exit 2
        msg = f"compat_check.py could not compare: {type(e).__name__}: {e}"
        print(json.dumps({"error": msg}) if args.json else msg)
        return 2

    bad = {i: d for i, d in sorted(breaks.items()) if i not in allow}
    ok = {i: d for i, d in sorted(breaks.items()) if i in allow}
    stale = [i for i in allow if i not in breaks]
    summary = (f"{base} -> working tree (HEAD {head}): {len(bad)} break(s), {len(ok)} allowed, {len(stale)} stale; "
               f"compared {compared}")
    if args.json:
        print(json.dumps({"base": base, "head": head,
                          "breaks": [{"id": i, "detail": d} for i, d in bad.items()],
                          "allowed": [{"id": i, "reason": allow[i], "detail": d} for i, d in ok.items()],
                          "stale": [{"id": i, "reason": allow[i]} for i in stale],
                          "summary": summary}, ensure_ascii=False, indent=2))
    else:
        for i, d in bad.items():
            print(f"BREAK {i} — {d}")
        for i in ok:
            print(f"ALLOWED {i} — {allow[i]}")
        for i in stale:
            print(f"STALE {i} — no longer breaks; delete it from {ALLOW}")
        print(summary)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
