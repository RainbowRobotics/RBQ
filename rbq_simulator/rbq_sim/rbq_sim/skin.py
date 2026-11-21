"""RBQ10 skin — the textured look MuJoCo shows (resources/model/rbq/rbq.xml), for the Isaac viewport.

The articulation comes from rbq_lab's URDF, whose visuals are untextured STL in flat grey. Left alone
so the RL trainer's asset is untouched, rbq_sim hides each link's `visuals` and adds a `skin` beside
it: the same-frame OBJ (UV-mapped) with the *_skin.png MuJoCo uses. Placement comes from the URDF
<visual> origin, so the skin follows the link exactly. Visual only — no collision, mass or sensor change.
"""

import math
import os
import xml.etree.ElementTree as ET

from PIL import Image
from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

# Mesh stem -> texture; None = no UVs in the OBJ (hips), shaded with the texture's mean colour like MuJoCo.
_TEXTURE = {"trunk": "trunk_skin.png", "calf": "calf_skin.png", "hip2": None, "hip3": None}
_HIP_TEXTURE = "hip_skin.png"


def _thigh_texture(link: str) -> str:
    return "thigh_right_skin.png" if link[1] == "R" else "thigh_left_skin.png"  # HR_/FR_ = right side


def _parse_obj(path: str):
    """OBJ -> (points, face counts, face point indices, faceVarying st or None, faceVarying normals or None)."""
    v, vt, vn = [], [], []
    counts, idx, st, nrm = [], [], [], []
    with open(path) as f:
        for line in f:
            tag, _, rest = line.partition(" ")
            if tag == "v":
                v.append(Gf.Vec3f(*map(float, rest.split()[:3])))
            elif tag == "vt":
                vt.append(Gf.Vec2f(*map(float, rest.split()[:2])))
            elif tag == "vn":
                vn.append(Gf.Vec3f(*map(float, rest.split()[:3])))
            elif tag == "f":
                corners = rest.split()
                counts.append(len(corners))
                for c in corners:
                    parts = c.split("/")
                    idx.append(int(parts[0]) - 1)
                    if len(parts) > 1 and parts[1]:
                        st.append(vt[int(parts[1]) - 1])
                    if len(parts) > 2 and parts[2]:
                        nrm.append(vn[int(parts[2]) - 1])
    st = st if len(st) == len(idx) else None
    nrm = nrm if len(nrm) == len(idx) else None
    return v, counts, idx, st, nrm


def _urdf_visuals(urdf_path: str):
    """link -> (mesh path without extension, xyz, rpy) from each <visual> with a mesh."""
    out = {}
    for link in ET.parse(urdf_path).getroot().iter("link"):
        vis = link.find("visual")
        mesh = vis.find("geometry/mesh") if vis is not None else None
        if mesh is None:
            continue
        org = vis.find("origin")
        xyz = [float(x) for x in (org.get("xyz", "0 0 0") if org is not None else "0 0 0").split()]
        rpy = [float(x) for x in (org.get("rpy", "0 0 0") if org is not None else "0 0 0").split()]
        stl = os.path.normpath(os.path.join(os.path.dirname(urdf_path), mesh.get("filename")))
        out[link.get("name")] = (os.path.splitext(stl)[0], xyz, rpy)
    return out


def _material(stage, path: str, texture: str | None, color):
    # OmniPBR (MDL): the RTX renderer's native material; it reads the mesh's `st` primvar as UV.
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.SetSourceAsset("OmniPBR.mdl", "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(0.55)
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    if texture:
        sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(texture)
    else:
        sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(out)
    mat.CreateVolumeOutput("mdl").ConnectToSource(out)
    mat.CreateDisplacementOutput("mdl").ConnectToSource(out)
    return mat


def apply_skin(stage, robot_path: str, urdf_path: str) -> int:
    """Hide the URDF visuals under robot_path and add the textured skin. Returns links skinned."""
    visuals_by_link = _urdf_visuals(urdf_path)
    if not visuals_by_link:
        return 0
    mesh_dir = os.path.dirname(next(iter(visuals_by_link.values()))[0])  # skins sit next to the URDF meshes
    hip_rgb = Image.open(os.path.join(mesh_dir, _HIP_TEXTURE)).convert("RGB").resize((1, 1)).getpixel((0, 0))
    hip_color = tuple(c / 255.0 for c in hip_rgb)
    looks = f"{robot_path}/SkinLooks"
    materials, meshes = {}, {}
    n = 0
    for link, (base, xyz, rpy) in visuals_by_link.items():
        stem = os.path.basename(base)
        link_prim = stage.GetPrimAtPath(f"{robot_path}/{link}")
        obj = base + ".obj"
        if not link_prim.IsValid() or not os.path.isfile(obj):
            continue
        texture = _thigh_texture(link) if stem == "thigh" else _TEXTURE.get(stem)
        key = texture or "hip"
        if key not in materials:
            materials[key] = _material(stage, f"{looks}/{os.path.splitext(key)[0]}",
                                       os.path.join(mesh_dir, texture) if texture else None, hip_color)
        if stem not in meshes:
            meshes[stem] = _parse_obj(obj)
        points, counts, idx, st, nrm = meshes[stem]

        visuals = stage.GetPrimAtPath(f"{robot_path}/{link}/visuals")
        if visuals.IsValid():
            UsdGeom.Imageable(visuals).MakeInvisible()

        xf = UsdGeom.Xform.Define(stage, f"{robot_path}/{link}/skin")
        xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*(math.degrees(a) for a in rpy)))  # URDF rpy = fixed-axis X, Y, Z
        mesh = UsdGeom.Mesh.Define(stage, f"{robot_path}/{link}/skin/mesh")
        mesh.CreatePointsAttr(Vt.Vec3fArray(points))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        if nrm is not None:
            mesh.CreateNormalsAttr(Vt.Vec3fArray(nrm))
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        if st is not None:
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(Vt.Vec2fArray(st))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[key])
        n += 1
    return n
