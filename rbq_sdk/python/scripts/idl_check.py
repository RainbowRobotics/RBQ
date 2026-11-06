#!/usr/bin/env python3
"""Do the Python SDK's message types still match the robot's IDL?

    python3 rbq_sdk/python/scripts/idl_check.py      # exit 0 all match / 1 one line per mismatch / 2 could not parse

The robot (Network) and the C++ SDK are built from rbq_sdk/cpp/rbq_sdk_cpp/idl/*.idl. The Python SDK keeps
its own copy (rbq_sdk_py/idl/, example/idl/_rbq_idl.py) and nothing regenerates it, so a field changed on one
side only goes unnoticed until customers see it. That happened with Response_.binary (robot int8, SDK uint8):
DDS dropped every reply and each SportClient call ended in 3104 (#1215).

For every Python struct this compares, by package + struct name, with the IDL struct:
  - extensibility (final / appendable / mutable),
  - field names, order and types — int8 vs uint8, array lengths and sequence bounds included.
IDL structs that the Python SDK does not carry are not errors (the SDK exposes a subset).
Standard library only: the files are parsed, not imported, so no cyclonedds is needed. Run by nightly.yml.
"""
import ast
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
IDL_DIR = os.path.join(ROOT, "rbq_sdk", "cpp", "rbq_sdk_cpp", "idl")
PY_DIRS = [os.path.join(ROOT, "rbq_sdk", "python", "rbq_sdk_py", "idl"),
           os.path.join(ROOT, "rbq_sdk", "python", "example", "idl")]
PKG = {"api": "rbq_api"}  # IDL root module -> Python package, where they differ

IDL_PRIM = {"octet": "uint8", "boolean": "bool", "float": "float32", "double": "float64",
            "long": "int32", "unsigned long": "uint32", "short": "int16", "unsigned short": "uint16",
            "long long": "int64", "unsigned long long": "uint64"}
PY_PRIM = {"str": "string", "bool": "bool", "float": "float64", "int": "int64"}


# ── IDL side ────────────────────────────────────────────────────────────────
def idl_type(t, consts):
    t = " ".join(t.split())
    m = re.fullmatch(r"sequence<\s*(.+?)\s*(?:,\s*(\w+)\s*)?>", t)
    if m:
        bound = f",{consts.get(m.group(2), m.group(2))}" if m.group(2) else ""
        return f"sequence<{idl_type(m.group(1), consts)}{bound}>"
    m = re.fullmatch(r"string\s*<\s*(\w+)\s*>", t)
    if m:
        return f"string<{consts.get(m.group(1), m.group(1))}>"
    return IDL_PRIM.get(t, t.split("::")[-1])


def parse_idl():
    text = ""
    for d, _, files in os.walk(IDL_DIR):
        for f in sorted(files):
            if f.endswith(".idl"):
                text += open(os.path.join(d, f), encoding="utf-8").read() + "\n"
    text = re.sub(r"//[^\n]*|/\*.*?\*/", "", text, flags=re.S)
    consts = {n: v for n, v in re.findall(r"const\s+[\w ]+?\s+(\w+)\s*=\s*(\d+)\s*;", text)}
    structs, stack, ext = {}, [], "final"
    for tok in re.finditer(r"module\s+(\w+)\s*\{|@extensibility\((\w+)\)|struct\s+(\w+)\s*\{(.*?)\}\s*;|\}\s*;", text, re.S):
        if tok.group(1):
            stack.append(tok.group(1))
        elif tok.group(2):
            ext = tok.group(2).lower()
        elif tok.group(3):
            fields = []
            for decl in tok.group(4).split(";"):
                decl = re.sub(r"@\w+\s*(\([^)]*\))?", "", decl).strip()
                if not decl:
                    continue
                m = re.fullmatch(r"(.+?)\s+(\w+)\s*((?:\[\s*\w+\s*\])*)", decl, re.S)
                t = idl_type(m.group(1), consts)
                for dim in reversed(re.findall(r"\[\s*(\w+)\s*\]", m.group(3))):
                    t = f"{t}[{consts.get(dim, dim)}]"
                fields.append((m.group(2), t))
            key = (PKG.get(stack[0], stack[0]), tok.group(3))
            if key in structs and structs[key] != (ext, fields):  # which one is the robot's? Don't pick silently
                raise ValueError(f"{key[0]}/{key[1]} is defined twice in the IDL with different fields")
            structs[key] = (ext, fields)
            ext = "final"
        elif stack:
            stack.pop()
    return structs


# ── Python side ─────────────────────────────────────────────────────────────
def py_type(node, consts):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):  # 'pkg.msg.dds_.Name_' forward ref
        return node.value.split(".")[-1]
    if isinstance(node, ast.Name):
        return PY_PRIM.get(node.id, node.id)
    if isinstance(node, ast.Attribute):  # types.int8
        return node.attr
    if isinstance(node, ast.Subscript):
        kind = py_type(node.value, consts)
        args = node.slice.elts if isinstance(node.slice, ast.Tuple) else [node.slice]
        if kind == "sequence":
            bound = f",{args[1].value}" if len(args) > 1 else ""
            return f"sequence<{py_type(args[0], consts)}{bound}>"
        if kind == "array":
            n = args[1]
            return f"{py_type(args[0], consts)}[{consts[n.id] if isinstance(n, ast.Name) else n.value}]"
        if kind == "bounded_str":
            return f"string<{args[0].value}>"
    raise ValueError(ast.unparse(node))


def parse_py():
    structs = {}
    for top in PY_DIRS:
        for d, _, files in os.walk(top):
            for f in sorted(files):
                if not f.endswith(".py"):
                    continue
                path = os.path.join(d, f)
                tree = ast.parse(open(path, encoding="utf-8").read())
                consts = {t.id: st.value.value for st in tree.body if isinstance(st, ast.Assign)
                          for t in st.targets if isinstance(t, ast.Name) and isinstance(st.value, ast.Constant)}
                for cls in ast.walk(tree):
                    if not isinstance(cls, ast.ClassDef):
                        continue
                    typename = next((k.value.value for k in cls.keywords if k.arg == "typename"), None)
                    if typename is None:
                        continue
                    ext = "final"
                    for dec in cls.decorator_list:
                        name = dec.attr if isinstance(dec, ast.Attribute) else getattr(dec, "id", "")
                        if name in ("final", "appendable", "mutable"):
                            ext = name
                    fields = [(st.target.id, py_type(st.annotation, consts)) for st in cls.body if isinstance(st, ast.AnnAssign)]
                    rel = os.path.relpath(path, ROOT)
                    structs.setdefault((typename.split(".")[0], cls.name), []).append((rel, ext, fields))
    return structs


def main():
    idl, py = parse_idl(), parse_py()
    problems = []
    for key, copies in sorted(py.items()):
        for rel, ext, fields in copies:
            where = f"{key[0]}/{key[1]} ({rel})"
            if key not in idl:
                problems.append(f"{where}: not in the robot IDL")
                continue
            iext, ifields = idl[key]
            if ext != iext:
                problems.append(f"{where}: extensibility {ext}, robot IDL {iext}")
            if fields != ifields:
                pf, rf = dict(fields), dict(ifields)
                for n, t in ifields:
                    if n not in pf:
                        problems.append(f"{where}: missing field {n} ({t})")
                    elif pf[n] != t:
                        problems.append(f"{where}: field {n} is {pf[n]}, robot IDL {t}")
                for n, t in fields:
                    if n not in rf:
                        problems.append(f"{where}: extra field {n} ({t})")
                if [n for n, _ in fields if n in rf] != [n for n, _ in ifields if n in pf]:
                    problems.append(f"{where}: field order differs from the robot IDL")
    checked = sum(len(c) for c in py.values())
    for p in problems:
        print(p)
    print(f"{checked} Python SDK types checked against {len(idl)} IDL structs: "
          f"{'all match' if not problems else f'{len(problems)} mismatch(es)'}")
    return 1 if problems else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:  # a file this parser cannot read is a broken check, not "all match" — exit 2
        print(f"idl_check.py could not compare: {type(e).__name__}: {e}")
        sys.exit(2)
