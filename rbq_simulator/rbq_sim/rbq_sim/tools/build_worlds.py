"""Build rbq_sim's worlds: the rbq_mujoco test field, and stairs / ramps / multi-floor / rough terrain
for SLAM and vision.

The research worlds are authored from primitives by this script — no third-party assets, so the layouts
are free to change. Geometry is textured convex meshes with static colliders (friction 0.8, like the flat
ground), plus procedurally generated textures so cameras and visual SLAM have features to track.
rbq_test copies rbq_mujoco's environment (its docking station and grating meshes, ArUco boards and
dimensions), so the same scenarios run in all three simulators.

    source rbq_simulator/rbq_sim/scripts/activate.bash
    python -m rbq_sim.tools.build_worlds                  # all worlds
    python -m rbq_sim.tools.build_worlds stairs_course    # one world

Output: rbq_simulator/rbq_sim/worlds/<world>.usda + worlds/textures/*.png (registered in worlds.py).
Customize by editing the build_* functions below and re-running, or open the .usda in Isaac Sim and
edit it there (re-running this script overwrites such edits).
"""

import math
import os
import shutil
import sys

import numpy as np
from PIL import Image

WORLDS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "worlds")
TEXTURE_DIR = os.path.join(WORLDS_DIR, "textures")
# rbq_mujoco's environment assets (resources/model/env/environment.xml uses the same files).
MJ_ENV_DIR = os.path.join(os.path.dirname(os.path.dirname(WORLDS_DIR)), "rbq_mujoco", "resources", "meshes", "env")
TEX_SIZE = 512
FRICTION = 0.8  # matches scene.py flat ground


# ---------------------------------------------------------------------------------------------------
# Procedural textures (seeded, so re-running reproduces the same images)
# ---------------------------------------------------------------------------------------------------


def _noise(rng, size, octaves=5):
    """Tileable value noise in [0, 1] (the lattice wraps, so repeated textures show no seams)."""
    out = np.zeros((size, size))
    amp, total = 1.0, 0.0
    for o in range(octaves):
        cells = 4 * (2 ** o)
        grid = np.pad(rng.random((cells, cells)), 2, mode="wrap")
        step = size // cells
        big = Image.fromarray((grid * 255).astype(np.uint8)).resize(((cells + 4) * step,) * 2, Image.BICUBIC)
        out += amp * (np.asarray(big)[2 * step:2 * step + size, 2 * step:2 * step + size] / 255.0)
        total += amp
        amp *= 0.5
    return out / total


def _save(name, rgb):
    Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8)).save(os.path.join(TEXTURE_DIR, name))


def build_textures():
    os.makedirs(TEXTURE_DIR, exist_ok=True)
    rng = np.random.default_rng(7)
    s = TEX_SIZE
    yy, xx = np.mgrid[0:s, 0:s]

    n = _noise(rng, s)
    _save("concrete.png", np.stack([150 + 70 * n] * 3, -1) + rng.normal(0, 6, (s, s, 1)))

    tile = 128
    grout = ((xx % tile) < 4) | ((yy % tile) < 4)
    shade = rng.uniform(-18, 18, (s // tile, s // tile))[yy // tile, xx // tile]
    base = np.stack([200 + shade, 196 + shade, 184 + shade], -1)
    _save("tiles.png", np.where(grout[..., None], 90, base) + rng.normal(0, 4, (s, s, 1)))

    bh, bw = 64, 128
    row = yy // bh
    off = (row % 2) * (bw // 2)
    mortar = ((yy % bh) < 6) | (((xx + off) % bw) < 6)
    brick_id = row * 10 + (xx + off) // bw
    tone = (np.sin(brick_id * 12.9898) * 43758.5453) % 1.0
    brick = np.stack([140 + 50 * tone, 60 + 25 * tone, 45 + 15 * tone], -1) + 25 * _noise(rng, s)[..., None]
    _save("brick.png", np.where(mortar[..., None], 175, brick))

    # Planks: 8 boards per repeat with offset seams, each with its own tone and slow grain.
    plank = yy // (s // 8)
    seam = ((yy % (s // 8)) < 3) | (((xx + plank * 97) % s) < 3)
    grain = 0.5 + 0.5 * np.sin(xx / 22.0 + 5 * _noise(rng, s, octaves=3) + plank * 1.7)
    tone = ((np.sin(plank * 7.13) * 43758.5453) % 1.0)[..., None] * 30
    wood = np.stack([130 + 35 * grain, 90 + 25 * grain, 55 + 15 * grain], -1) + tone
    _save("wood.png", np.where(seam[..., None], 60, wood))

    for name, rgb in (("plaster_blue.png", (120, 150, 190)), ("plaster_green.png", (135, 175, 130)),
                      ("plaster_white.png", (215, 212, 205))):
        pn = _noise(rng, s)
        _save(name, np.stack([c - 25 + 50 * pn for c in rgb], -1))

    stripes = (((xx + yy) // 48) % 2).astype(bool)
    _save("hazard.png", np.where(stripes[..., None], np.array([230, 190, 20]), np.array([30, 30, 30])))

    g = _noise(rng, s)
    d = _noise(rng, s, octaves=3)
    dirt = d[..., None] > 0.6
    _save("grass.png", np.where(dirt, np.stack([120 + 40 * g, 95 + 30 * g, 60 + 20 * g], -1),
                                np.stack([55 + 40 * g, 110 + 60 * g, 40 + 30 * g], -1)))

    bark = _noise(rng, s, octaves=4)
    _save("bark.png", np.stack([70 + 50 * bark, 50 + 35 * bark, 35 + 20 * bark], -1))

    # Marker posters: distinct random binary grids (one per poster) — strong corners for visual SLAM.
    for k in range(6):
        cells = rng.integers(0, 2, (8, 8))
        cells[0, :] = cells[-1, :] = cells[:, 0] = cells[:, -1] = 0
        img = np.kron(cells, np.ones((s // 8, s // 8)))
        hue = np.array([[255, 255, 255], [255, 230, 200], [200, 230, 255]])[k % 3]
        _save(f"marker_{k}.png", np.where(img[..., None] > 0, hue, 15))

    # rbq_test: MuJoCo's blue checker floor, and the docking ArUco boards copied from rbq_mujoco.
    cell = s // 2
    check = ((xx // cell + yy // cell) % 2).astype(bool)
    edge = ((xx % cell) < 3) | ((yy % cell) < 3)
    floor = np.where(check[..., None], np.array([51, 77, 102]), np.array([26, 51, 77]))
    _save("checker_blue.png", np.where(edge[..., None], np.array([90, 120, 150]), floor))
    for png in ("bottom_id.png", "rear_id.png"):
        shutil.copyfile(os.path.join(MJ_ENV_DIR, png), os.path.join(TEXTURE_DIR, png))


# ---------------------------------------------------------------------------------------------------
# Mesh files and MuJoCo frames
# ---------------------------------------------------------------------------------------------------


def load_obj(path):
    """Triangle OBJ -> (points (N, 3), triangles (M, 3)); only v / f lines are read."""
    pts, tris = [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("v "):
                pts.append([float(t) for t in line.split()[1:4]])
            elif line.startswith("f "):
                tris.append([int(t.split("/")[0]) - 1 for t in line.split()[1:4]])
    return np.array(pts), np.array(tris)


def load_stl(path):
    """Binary STL -> (points, triangles) with shared vertices."""
    data = open(path, "rb").read()
    n = int(np.frombuffer(data[80:84], "<u4")[0])
    rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    v = np.frombuffer(data[84:84 + n * 50], rec)["v"].reshape(-1, 3).astype(float)
    pts, inv = np.unique(np.round(v, 6), axis=0, return_inverse=True)
    return pts, inv.reshape(-1, 3)


def mj_euler(rx, ry, rz):
    """MuJoCo euler (default eulerseq 'xyz', rotating axes) -> rotation matrix."""
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rot_x @ rot_y @ rot_z


# ---------------------------------------------------------------------------------------------------
# USD authoring
# ---------------------------------------------------------------------------------------------------


class WorldBuilder:
    """Stage wrapper: textured convex meshes baked in world coordinates, static colliders, lights."""

    def __init__(self, name):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

        self.Gf, self.Sdf, self.UsdGeom, self.UsdPhysics, self.UsdShade = Gf, Sdf, UsdGeom, UsdPhysics, UsdShade
        self.path = os.path.join(WORLDS_DIR, f"{name}.usda")
        self.stage = Usd.Stage.CreateNew(self.path)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        root = UsdGeom.Xform.Define(self.stage, "/Root")
        self.stage.SetDefaultPrim(root.GetPrim())
        self.stage.SetMetadata("comment", f"rbq_sim world '{name}' — generated by rbq_sim/tools/build_worlds.py")
        self._materials = {}
        self._count = {}

        phys = UsdShade.Material.Define(self.stage, "/Root/Looks/Physics")
        api = UsdPhysics.MaterialAPI.Apply(phys.GetPrim())
        api.CreateStaticFrictionAttr(FRICTION)
        api.CreateDynamicFrictionAttr(FRICTION)
        api.CreateRestitutionAttr(0.0)
        self.physics_material = phys

    def material(self, texture, tile=1.0, roughness=0.85):
        """UsdPreviewSurface with a repeating texture; `tile` = metres per texture repeat."""
        key = (texture, tile)
        if key in self._materials:
            return self._materials[key]
        Sdf, UsdShade = self.Sdf, self.UsdShade
        name = f"{os.path.splitext(texture)[0]}_{str(tile).replace('.', 'p')}"
        mat = UsdShade.Material.Define(self.stage, f"/Root/Looks/{name}")
        shader = UsdShade.Shader.Define(self.stage, f"/Root/Looks/{name}/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        reader = UsdShade.Shader.Define(self.stage, f"/Root/Looks/{name}/St")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(self.stage, f"/Root/Looks/{name}/Texture")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./textures/{texture}")
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        self._materials[key] = (mat, tile)
        return self._materials[key]

    def color(self, name, rgb, roughness=0.6):
        """Untextured UsdPreviewSurface; same (material, tile) shape as material(). `rgb` is a display
        (sRGB) colour like MuJoCo's rgba; diffuseColor is linear, so it is converted."""
        key = ("color", name)
        if key in self._materials:
            return self._materials[key]
        mat = self.UsdShade.Material.Define(self.stage, f"/Root/Looks/{name}")
        shader = self.UsdShade.Shader.Define(self.stage, f"/Root/Looks/{name}/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        linear = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb]
        shader.CreateInput("diffuseColor", self.Sdf.ValueTypeNames.Color3f).Set(self.Gf.Vec3f(*linear))
        shader.CreateInput("roughness", self.Sdf.ValueTypeNames.Float).Set(roughness)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        self._materials[key] = (mat, 1.0)
        return self._materials[key]

    def friction_material(self, name, friction):
        """Physics material with its own friction; 'min' combine so the feet do not average it back up."""
        from pxr import PhysxSchema

        mat = self.UsdShade.Material.Define(self.stage, f"/Root/Looks/{name}")
        api = self.UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(friction)
        api.CreateDynamicFrictionAttr(friction)
        api.CreateRestitutionAttr(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr().Set("min")
        return mat

    def _name(self, group, base):
        k = f"{group}/{base}"
        self._count[k] = self._count.get(k, 0) + 1
        return f"/Root/{group}/{base}_{self._count[k]:03d}"

    def mesh(self, group, base, points, faces, material, collide=True, approximation="convexHull",
             physics=None, visible=True):
        """Polygon mesh from world-space points; UVs are planar per face (textures align across meshes)."""
        Gf, UsdGeom, UsdPhysics, UsdShade = self.Gf, self.UsdGeom, self.UsdPhysics, self.UsdShade
        mat, tile = material
        pts = np.asarray(points, dtype=float)
        counts, indices, st, normals = [], [], [], []
        for face in faces:
            p = pts[face]
            nrm = np.cross(p[1] - p[0], p[2] - p[0]) + (np.cross(p[2] - p[0], p[3] - p[0]) if len(face) > 3 else 0.0)
            if np.linalg.norm(nrm) < 1e-9:
                continue  # zero-height side of a ramp
            nrm = nrm / np.linalg.norm(nrm)
            axis = int(np.argmax(np.abs(nrm)))
            u, v = [(1, 2), (0, 2), (0, 1)][axis]
            counts.append(len(face))
            indices.extend(face)
            st.extend((q[u] / tile, q[v] / tile) for q in p)
            normals.extend([tuple(nrm)] * len(face))
        prim = UsdGeom.Mesh.Define(self.stage, self._name(group, base))
        prim.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in pts])
        prim.CreateFaceVertexCountsAttr(counts)
        prim.CreateFaceVertexIndicesAttr(indices)
        prim.CreateNormalsAttr([Gf.Vec3f(*n) for n in normals])
        prim.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        prim.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        lo, hi = pts.min(0), pts.max(0)
        prim.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
        pv = UsdGeom.PrimvarsAPI(prim).CreatePrimvar("st", self.Sdf.ValueTypeNames.TexCoord2fArray,
                                                     UsdGeom.Tokens.faceVarying)
        pv.Set([Gf.Vec2f(float(a), float(b)) for a, b in st])
        binding = UsdShade.MaterialBindingAPI.Apply(prim.GetPrim())
        binding.Bind(mat)
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(prim.GetPrim()).CreateApproximationAttr(approximation)
            binding.Bind(physics or self.physics_material, UsdShade.Tokens.weakerThanDescendants, "physics")
        if not visible:
            prim.CreatePurposeAttr(UsdGeom.Tokens.guide)  # collider only: guides are not rendered
        return prim

    def box(self, group, center, size, material, yaw=0.0, collide=True, rot=None, physics=None, visible=True):
        """Box; center/size in metres, turned by `yaw` about z or by the 3x3 rotation `rot`."""
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.asarray(rot, dtype=float) if rot is not None else np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        hx, hy, hz = (v * 0.5 for v in size)
        corners = []
        for z in (-hz, hz):
            for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
                corners.append(np.asarray(center, dtype=float) + rot @ (x, y, z))
        faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        return self.mesh(group, "box", corners, faces, material, collide, physics=physics, visible=visible)

    def block(self, group, x0, x1, y0, y1, z0, z1, material, collide=True, physics=None, visible=True):
        return self.box(group, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                        (abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)), material, collide=collide,
                        physics=physics, visible=visible)

    def raw_mesh(self, path, points, triangles, material, prim=None):
        """Visual-only triangle mesh with shared vertices (no UVs / baked normals): keeps big meshes small."""
        Gf, UsdGeom = self.Gf, self.UsdGeom
        mesh = UsdGeom.Mesh(prim) if prim is not None else UsdGeom.Mesh.Define(self.stage, path)
        pts = np.asarray(points, dtype=float)
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in np.round(pts, 5)])
        mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
        mesh.CreateFaceVertexIndicesAttr([int(i) for i in np.asarray(triangles).ravel()])
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))])
        self.UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material[0])
        return mesh

    def prototype(self, name, points, triangles, material):
        """raw_mesh as a class prim: not drawn itself, drawn wherever instance() references it."""
        prim = self.stage.CreateClassPrim(f"/{name}")
        prim.SetTypeName("Mesh")
        self.raw_mesh(None, points, triangles, material, prim=prim)
        return prim.GetPath()

    def instance(self, group, proto_path, translate, scale=(1.0, 1.0, 1.0)):
        xf = self.UsdGeom.Xform.Define(self.stage, self._name(group, "inst"))
        xf.AddTranslateOp().Set(self.Gf.Vec3d(*translate))
        xf.AddScaleOp().Set(self.Gf.Vec3f(*scale))
        child = self.stage.DefinePrim(xf.GetPath().AppendChild("mesh"))
        child.GetReferences().AddInternalReference(proto_path)

    def marker(self, center, rot, texture, half=0.05, thick=0.001):
        """ArUco board as rbq_mujoco draws it: a thin white plate with the image on its local +z face,
        image right = local +x, image up = local +y."""
        Gf, UsdGeom, UsdShade = self.Gf, self.UsdGeom, self.UsdShade
        rot = np.asarray(rot, dtype=float)
        self.box("Markers", center, (2 * half, 2 * half, 2 * thick), self.color("white", (1.0, 1.0, 1.0)),
                 rot=rot, collide=False)
        z = thick + 0.0005  # just off the plate face
        corners = [np.asarray(center) + rot @ p for p in ((-half, half, z), (half, half, z), (half, -half, z), (-half, -half, z))]
        quad = UsdGeom.Mesh.Define(self.stage, self._name("Markers", "board"))
        quad.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in corners])
        quad.CreateFaceVertexCountsAttr([4])
        quad.CreateFaceVertexIndicesAttr([3, 2, 1, 0] if np.dot(np.cross(corners[1] - corners[0], corners[3] - corners[0]), rot[:, 2]) < 0 else [0, 1, 2, 3])
        quad.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        quad.CreateExtentAttr([Gf.Vec3f(*map(float, np.min(corners, 0))), Gf.Vec3f(*map(float, np.max(corners, 0)))])
        st = {0: (0, 1), 1: (1, 1), 2: (1, 0), 3: (0, 0)}  # TL, TR, BR, BL of the image
        pv = UsdGeom.PrimvarsAPI(quad).CreatePrimvar("st", self.Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set([Gf.Vec2f(*st[i]) for i in range(4)])
        UsdShade.MaterialBindingAPI.Apply(quad.GetPrim()).Bind(self.material(texture, 1.0, roughness=0.5)[0])

    def rigid(self, name, center, material, mass, box=None, radius=None):
        """Free rigid body (a box of size `box`, or a sphere), like MuJoCo's free-joint bodies."""
        from pxr import PhysxSchema

        Gf, UsdGeom, UsdPhysics, UsdShade = self.Gf, self.UsdGeom, self.UsdPhysics, self.UsdShade
        xf = UsdGeom.Xform.Define(self.stage, f"/Root/Movable/{name}")
        xf.AddTranslateOp().Set(Gf.Vec3d(*center))
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(mass)
        # The MuJoCo pile starts interpenetrating: let PhysX separate it gently instead of flinging it.
        PhysxSchema.PhysxRigidBodyAPI.Apply(xf.GetPrim()).CreateMaxDepenetrationVelocityAttr(0.5)
        if radius is not None:
            geom = UsdGeom.Sphere.Define(self.stage, xf.GetPath().AppendChild("geom"))
            geom.CreateRadiusAttr(radius)
        else:
            geom = UsdGeom.Cube.Define(self.stage, xf.GetPath().AppendChild("geom"))
            geom.CreateSizeAttr(1.0)
            geom.AddScaleOp().Set(Gf.Vec3f(*box))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        binding = UsdShade.MaterialBindingAPI.Apply(geom.GetPrim())
        binding.Bind(material[0])
        binding.Bind(self.physics_material, UsdShade.Tokens.weakerThanDescendants, "physics")

    def sun(self, intensity=500.0, tilt=(25.0, -30.0, 0.0)):
        from pxr import UsdLux

        light = UsdLux.DistantLight.Define(self.stage, "/Root/Lights/sun")
        light.CreateIntensityAttr(intensity)
        light.CreateAngleAttr(1.0)
        light.AddRotateXYZOp().Set(self.Gf.Vec3f(*tilt))

    def wedge(self, group, x0, x1, y0, y1, z_low, z_high, material, rising="+x", base=0.0):
        """Solid ramp from z_low to z_high across [x0,x1] (rising +x/-x) or [y0,y1] (+y/-y)."""
        if rising in ("+x", "-x"):
            lo_x, hi_x = (x0, x1) if rising == "+x" else (x1, x0)
            pts = [(lo_x, y0, base), (hi_x, y0, base), (hi_x, y1, base), (lo_x, y1, base),
                   (lo_x, y0, z_low), (hi_x, y0, z_high), (hi_x, y1, z_high), (lo_x, y1, z_low)]
        else:
            lo_y, hi_y = (y0, y1) if rising == "+y" else (y1, y0)
            pts = [(x0, lo_y, base), (x0, hi_y, base), (x1, hi_y, base), (x1, lo_y, base),
                   (x0, lo_y, z_low), (x0, hi_y, z_high), (x1, hi_y, z_high), (x1, lo_y, z_low)]
        # Orient faces outward regardless of the rising direction / axis handedness.
        centre = np.mean(np.asarray(pts, dtype=float), axis=0)
        faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        oriented = []
        p = np.asarray(pts, dtype=float)
        for f in faces:
            nrm = np.cross(p[f[1]] - p[f[0]], p[f[2]] - p[f[0]])
            oriented.append(f if np.dot(nrm, p[f].mean(0) - centre) > 0 else f[::-1])
        return self.mesh(group, "ramp", pts, oriented, material)

    def frustum(self, group, cx, cy, top_half, bottom_half, height, material):
        """Square truncated pyramid (a hill with four slopes and a flat top), one convex collider."""
        b, t = bottom_half, top_half
        pts = [(cx - b, cy - b, 0.0), (cx + b, cy - b, 0.0), (cx + b, cy + b, 0.0), (cx - b, cy + b, 0.0),
               (cx - t, cy - t, height), (cx + t, cy - t, height), (cx + t, cy + t, height), (cx - t, cy + t, height)]
        faces = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
        return self.mesh(group, "hill", pts, faces, material)

    def heightfield(self, group, x0, y0, size_x, size_y, cell, heights, material):
        """Static triangle-mesh terrain patch; heights[i, j] at (x0 + i*cell, y0 + j*cell)."""
        nx, ny = heights.shape
        pts = [(x0 + i * cell, y0 + j * cell, float(heights[i, j])) for i in range(nx) for j in range(ny)]
        faces = []
        for i in range(nx - 1):
            for j in range(ny - 1):
                a, b, c, d = i * ny + j, (i + 1) * ny + j, (i + 1) * ny + j + 1, i * ny + j + 1
                faces.extend([[a, b, c], [a, c, d]])
        return self.mesh(group, "terrain", pts, faces, material, approximation="none")

    def stairs(self, group, x0, y0, y1, rise, run, steps, base_z, riser_mat, tread_mat, direction=+1):
        """Solid straight flight along x starting at x0 (direction +1/-1); returns (x_end, top_z)."""
        for i in range(steps):
            xa = x0 + direction * i * run
            xb = x0 + direction * (i + 1) * run
            top = base_z + (i + 1) * rise
            self.block(group, min(xa, xb), max(xa, xb), y0, y1, 0.0, top - 0.02, riser_mat)
            self.block(group, min(xa, xb), max(xa, xb), y0, y1, top - 0.02, top, tread_mat)
        return x0 + direction * steps * run, base_z + steps * rise

    def sphere_light(self, center, intensity=400000.0, radius=0.25):
        from pxr import UsdLux

        light = UsdLux.SphereLight.Define(self.stage, self._name("Lights", "lamp"))
        light.CreateIntensityAttr(intensity)
        light.CreateRadiusAttr(radius)
        light.AddTranslateOp().Set(self.Gf.Vec3d(*center))

    def poster(self, center, yaw, index, size=(0.9, 0.9)):
        """Marker poster on a wall (visual only), facing +x rotated by yaw."""
        self.box("Posters", center, (0.02, size[0], size[1]), self.material(f"marker_{index % 6}.png", size[0]),
                 yaw=yaw, collide=False)

    def save(self):
        self.stage.GetRootLayer().Save()
        print(f"[build_worlds] wrote {os.path.relpath(self.path)}")


def walls_with_doors(b, axis, fixed, start, end, doors, z0, z1, mat, thickness=0.2, door_w=1.4):
    """Straight wall along `axis` ('x' or 'y') at coordinate `fixed`, with door gaps centred at `doors`."""
    cuts = sorted(doors)
    edges = [start] + [e for d in cuts for e in (d - door_w / 2, d + door_w / 2)] + [end]
    for a, c in zip(edges[0::2], edges[1::2]):
        if c - a < 1e-3:
            continue
        if axis == "x":
            b.block("Walls", a, c, fixed - thickness / 2, fixed + thickness / 2, z0, z1, mat)
        else:
            b.block("Walls", fixed - thickness / 2, fixed + thickness / 2, a, c, z0, z1, mat)


# ---------------------------------------------------------------------------------------------------
# World 0 — rbq_test: rbq_mujoco's test field (resources/model/env/environment.xml)
# ---------------------------------------------------------------------------------------------------


def build_rbq_test():
    """rbq_mujoco's environment, so a scenario runs the same in MuJoCo, Gazebo and Isaac Sim. Robot at the
    origin facing +x. Positions and sizes are MuJoCo's: pyramid stairs (0.15 m x 6) at x=5; 20 / 30 deg
    slopes at y=-3.5 / -7 / -12; slippery strips at y=+3 / +5; obstacle bars (0.10 / 0.15 / 0.20 m) at
    x=-2.2..-3.4; the docking station with its ArUco boards at (-1.5, -6); grating stairs at (-4, -9);
    a ball and a pile of bricks at (2, 6) / (0, 6)."""
    b = WorldBuilder("rbq_test")
    b.block("Floor", -50, 50, -50, 50, -0.2, 0.0, b.material("checker_blue.png", 0.5))

    # Pyramid stairs: six 0.15 m slabs (MuJoCo nests boxes; slabs give the same surfaces without z-fighting).
    for k, (hx, shade) in enumerate(zip((2.1, 1.8, 1.5, 1.2, 0.9, 0.6), (0.5, 0.4, 0.3, 0.2, 0.15, 0.1))):
        b.block("Stairs", 5 - hx, 5 + hx, -1.5, 1.5, 0.15 * k, 0.15 * (k + 1), b.color(f"stair_{k}", (shade,) * 3))

    blue = b.color("obstacle_blue", (0.4, 0.4, 0.8))
    for y, h in ((0.0, 0.10), (-2.0, 0.15), (-4.0, 0.20)):
        for x in (-2.2, -2.6, -3.0, -3.4):
            b.block("Obstacles", x - 0.1, x + 0.1, y - 0.75, y + 0.75, 0.0, h, blue)

    # Slopes: up / flat / down plates, 3 x 3 x 0.05 m, tilted about y.
    grey = b.color("slope_grey", (0.8, 0.8, 0.8))

    def rot_y(deg):
        a = math.radians(deg)
        return [[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]

    for y, deg, flat_x, flat_len, flat_z, down_x in ((-3.5, 20, 5.52, 1.68, 0.56, 7.747), (-12.0, 20, 5.52, 1.68, 0.56, None),
                                                     (-7.0, 30, 5.10, 1.02, 0.795, 6.898)):
        b.box("Slopes", (3.3, y, 0.05), (3.0, 3.0, 0.05), grey, rot=rot_y(-deg))
        b.box("Slopes", (flat_x, y, flat_z), (flat_len, 3.0, 0.05), grey)
        if down_x is not None:
            b.box("Slopes", (down_x, y, 0.05), (3.0, 3.0, 0.05), grey, rot=rot_y(deg))

    for y, mu, rgb in ((3.0, 0.2, (0.7, 0.4, 0.9)), (5.0, 0.1, (0.1, 0.5, 0.8))):
        tag = str(mu).replace(".", "p")  # USD prim names take no dots
        b.block("Slippery", 0.5, 5.5, y - 0.8, y + 0.8, 0.0, 0.055, b.color(f"slippery_{tag}", rgb),
                physics=b.friction_material(f"SlipperyPhysics_{tag}", mu))

    # Docking station: MuJoCo body pos (-1.5, -6, 0.005), quat (0 1 0 0) = 180 deg about x. Visual only,
    # as in MuJoCo (contype 0). ArUco boards: MuJoCo geoms bottom_id / rear_id, 0.1 m.
    rot_dock, pos_dock = np.diag([1.0, -1.0, -1.0]), np.array([-1.5, -6.0, 0.005])
    pts, tris = load_obj(os.path.join(MJ_ENV_DIR, "Docking_station.obj"))
    b.raw_mesh("/Root/Dock/station", pts @ rot_dock.T + pos_dock, tris, b.color("dock_grey", (0.7, 0.7, 0.7)))
    for gpos, euler, png in (((-0.1, -0.08, -0.035), (-3.141592, 0, 1.5708), "bottom_id.png"),
                             ((-0.4, -0.08, -0.50), (-1.5708, 1.5708, 0), "rear_id.png")):
        b.marker(pos_dock + rot_dock @ np.array(gpos), rot_dock @ mj_euler(*euler), png)

    # Grating stairs at (-4, -9): nine 0.5 x 1.5 m treads up to 0.75 m and down; Grating.stl drawn,
    # a 0.06 m box collides (MuJoCo's visual / collision geoms).
    gpts, gtris = load_stl(os.path.join(MJ_ENV_DIR, "Grating.stl"))
    proto = b.prototype("GratingMesh", gpts, gtris, b.color("grating_blue", (0.7, 0.7, 1.0)))
    for k in range(9):
        x, z = -4.0 + 0.4 * k, 0.15 * min(k + 1, 9 - k)
        b.instance("GratingStair", proto, (x - 0.25, -9.0 + 0.75, z), scale=(0.01, 0.03, 0.01))
        b.block("GratingStair", x - 0.25, x + 0.25, -9.75, -8.25, z - 0.03, z + 0.03, blue, visible=False)

    b.rigid("ball", (2.0, 6.0, 0.5), b.color("ball_green", (0.0, 1.0, 0.0)), 0.5, radius=0.2)
    red = b.color("brick_red", (1.0, 0.0, 0.0))
    for k, c in enumerate(((0.007, 6.170, 0.114), (0.023, 6.162, 0.104), (0.011, 6.050, 0.112), (0.039, 6.109, 0.085),
                           (0.001, 6.183, 0.116), (0.006, 6.034, 0.105), (0.033, 6.075, 0.114), (0.025, 6.193, 0.102),
                           (0.014, 6.078, 0.086), (0.009, 6.114, 0.084))):
        b.rigid(f"brick_{k + 1}", c, red, 1.0, box=(0.2, 0.08, 0.06))
    b.sun()
    b.save()


# ---------------------------------------------------------------------------------------------------
# World 1 — stairs_course: stair / ramp / step test hall
# ---------------------------------------------------------------------------------------------------


def build_stairs_course():
    """32 m x 16 m hall, robot spawns at the origin facing +x. Three lanes ahead:
    y=+5 gentle stairs (0.10 m rise), y=0 standard stairs (0.15 m) then a 15 deg ramp down,
    y=-5 ramps (10 deg up / 20 deg down); past x=16 a rough / stepping-stone / curb zone."""
    b = WorldBuilder("stairs_course")
    concrete, tiles, wood = b.material("concrete.png", 2.0), b.material("tiles.png", 1.0), b.material("wood.png", 0.6)
    brick, hazard = b.material("brick.png", 1.5), b.material("hazard.png", 0.5)

    b.block("Floor", -8, 24, -8, 8, -0.2, 0.0, tiles)
    for y in (-8.1, 8.1):
        b.block("Walls", -8.2, 24.2, y - 0.1, y + 0.1, 0.0, 3.0, brick)
    for x in (-8.1, 24.1):
        b.block("Walls", x - 0.1, x + 0.1, -8.2, 8.2, 0.0, 3.0, brick)

    # Lane y=+5: 6 x 0.10 m up, 1.5 m landing, 6 down.
    x, top = b.stairs("LaneGentle", 3.0, 4.2, 5.8, 0.10, 0.30, 6, 0.0, concrete, wood)
    b.block("LaneGentle", x, x + 1.5, 4.2, 5.8, 0.0, top, concrete)
    b.stairs("LaneGentle", x + 1.5 + 6 * 0.30, 4.2, 5.8, 0.10, 0.30, 6, 0.0, concrete, wood, direction=-1)
    b.block("LaneGentle", x + 1.5 + 6 * 0.30 + 0.3, x + 1.5 + 6 * 0.30 + 0.5, 4.2, 5.8, 0.0, 0.01, hazard)

    # Lane y=0: 6 x 0.15 m up (0.90 m), 2 m landing, 15 deg ramp down.
    x, top = b.stairs("LaneStandard", 3.0, -0.8, 0.8, 0.15, 0.28, 6, 0.0, concrete, wood)
    b.block("LaneStandard", x, x + 2.0, -0.8, 0.8, 0.0, top, concrete)
    run = top / math.tan(math.radians(15))
    b.wedge("LaneStandard", x + 2.0, x + 2.0 + run, -0.8, 0.8, 0.0, top, concrete, rising="-x")

    # Lane y=-5: 10 deg up to 0.6 m, 2 m plateau, 20 deg down.
    h = 0.6
    up = h / math.tan(math.radians(10))
    down = h / math.tan(math.radians(20))
    b.wedge("LaneRamps", 3.0, 3.0 + up, -5.8, -4.2, 0.0, h, concrete, rising="+x")
    b.block("LaneRamps", 3.0 + up, 5.0 + up, -5.8, -4.2, 0.0, h, concrete)
    b.wedge("LaneRamps", 5.0 + up, 5.0 + up + down, -5.8, -4.2, 0.0, h, concrete, rising="-x")

    # Rough zone x in [16, 23]: random low blocks (0.02-0.08 m), stepping stones, curbs.
    rng = np.random.default_rng(3)
    for i in range(14):
        for j in range(10):
            hgt = rng.uniform(0.02, 0.08)
            b.block("Rough", 16 + i * 0.5, 16.5 + i * 0.5, 2.0 + j * 0.5, 2.5 + j * 0.5, 0.0, hgt, concrete)
    for i in range(7):
        for j in range(5):
            hgt = rng.uniform(0.05, 0.15)
            b.block("SteppingStones", 16.1 + i * 0.95, 16.8 + i * 0.95, -2.0 + j * 0.8, -1.4 + j * 0.8, 0.0, hgt, wood)
    for i in range(5):
        b.block("Curbs", 16.5 + i * 1.3, 16.7 + i * 1.3, -7.0, -3.5, 0.0, 0.08 + 0.02 * i, hazard)

    for k, (px, py, yaw) in enumerate([(-7.99, 0, 0), (23.99, 3, math.pi), (23.99, -3, math.pi),
                                        (6, 7.99, -math.pi / 2), (14, -7.99, math.pi / 2), (0, -7.99, math.pi / 2)]):
        b.poster((px, py, 1.6), yaw, k, size=(1.2, 1.2))
    for x_, y_ in ((12.5, 2.8), (12.5, -2.8), (-4, 4), (-4, -4)):
        b.block("Pillars", x_ - 0.25, x_ + 0.25, y_ - 0.25, y_ + 0.25, 0.0, 3.0, brick)
    b.save()


# ---------------------------------------------------------------------------------------------------
# World 2 — two_floor_office: two storeys joined by a switchback staircase (multi-floor SLAM)
# ---------------------------------------------------------------------------------------------------


def build_two_floor_office():
    """20 m x 14 m building. Ground floor: four rooms in a door loop + a stair hall; switchback stairs
    (0.15 m x 20 steps, landing at 1.5 m) up to a second floor at 3.0 m with its own rooms and
    guard rails. Robot spawns in the ground-floor hall at (-1.5, 5.5) facing +x (worlds.py)."""
    b = WorldBuilder("two_floor_office")
    tiles, wood, concrete = b.material("tiles.png", 1.0), b.material("wood.png", 1.6), b.material("concrete.png", 2.0)
    brick, blue = b.material("brick.png", 1.5), b.material("plaster_blue.png", 2.0)
    green, white, hazard = b.material("plaster_green.png", 2.0), b.material("plaster_white.png", 2.0), b.material("hazard.png", 0.5)

    F1, SLAB, F2, TOP = 0.0, 0.2, 3.0, 5.6  # floor-2 slab occupies z in [F2 - SLAB, F2]
    b.block("Floor1", -10, 10, -7, 7, -0.2, F1, tiles)

    # Outer walls, both storeys.
    for y in (-7.1, 7.1):
        b.block("Walls", -10.2, 10.2, y - 0.1, y + 0.1, F1, TOP, brick)
    for x in (-10.1, 10.1):
        b.block("Walls", x - 0.1, x + 0.1, -7.2, 7.2, F1, TOP, brick)

    # Ground floor rooms: A (-10..-3, 0..7), B hall (-3..10, 0..7), C (-10..2, -7..0), stair hall (2..10, -7..0).
    walls_with_doors(b, "x", 0.0, -10, 10, [-6.5, 0.0, 6.0], F1, F2 - SLAB, white)
    walls_with_doors(b, "y", -3.0, 0, 7, [4.0], F1, F2 - SLAB, blue)
    walls_with_doors(b, "y", 2.0, -7, 0, [-1.5], F1, F2 - SLAB, green)
    for x_, y_ in ((2.0, 3.5), (6.0, 3.5)):
        b.block("Pillars", x_ - 0.2, x_ + 0.2, y_ - 0.2, y_ + 0.2, F1, F2 - SLAB, concrete)
    b.block("Furniture", -8.5, -6.5, 2.0, 3.0, F1, 0.75, wood)       # tables / crates for clutter
    b.block("Furniture", -8.0, -5.0, -5.5, -4.8, F1, 1.8, wood)
    b.block("Furniture", 7.5, 9.5, 5.5, 6.5, F1, 1.0, concrete)

    # Switchback stairs in the stair hall: flight 1 +x along y in [-6.8,-5.4], landing, flight 2 -x along y in [-5.0,-3.6].
    rise, run, n = 0.15, 0.30, 10
    x1, z1 = b.stairs("Stairs", 3.4, -6.8, -5.4, rise, run, n, F1, concrete, wood)
    b.block("Stairs", x1, x1 + 1.6, -6.8, -3.6, F1, z1, concrete)
    b.block("Stairs", x1, x1 + 1.6, -6.8, -3.6, z1 - 0.02, z1, hazard)
    x2, z2 = b.stairs("Stairs", x1, -5.0, -3.6, rise, run, n, z1, concrete, wood, direction=-1)
    b.block("Stairs", 3.4, x1, -5.4, -5.0, F1, F2 + 1.0, white)       # core wall between flights, doubles as rail

    # Floor-2 slab with the stairwell opening x in [3.4, 8.0], y in [-6.8, -3.6].
    ox0, ox1, oy0, oy1 = 3.4, x1 + 1.6, -6.8, -3.6
    b.block("Floor2", -10, ox0, -7, 7, F2 - SLAB, F2, wood)
    b.block("Floor2", ox1, 10, -7, 7, F2 - SLAB, F2, wood)
    b.block("Floor2", ox0, ox1, oy1, 7, F2 - SLAB, F2, wood)
    b.block("Floor2", ox0, ox1, -7, oy0, F2 - SLAB, F2, wood)
    rail = 1.0
    b.block("Rails", ox0 - 0.1, ox0, oy0, -5.4, F2, F2 + rail, hazard)    # above flight-1 start (flight 2 exits at y > -5.0)
    b.block("Rails", ox0, ox1, oy1, oy1 + 0.1, F2, F2 + rail, hazard)     # along the hall side
    b.block("Rails", ox1, ox1 + 0.1, oy0, oy1, F2, F2 + rail, hazard)     # above the landing

    # Second floor rooms.
    walls_with_doors(b, "x", 0.0, -10, 10, [-5.0, 5.0], F2, TOP, green)
    walls_with_doors(b, "y", 0.0, 0, 7, [3.5], F2, TOP, white)
    walls_with_doors(b, "y", -4.0, -7, 0, [-3.5], F2, TOP, blue)
    b.block("Furniture", 2.0, 4.0, 4.5, 6.5, F2, F2 + 0.75, concrete)
    stage_h = 0.3
    b.block("Stage", -9.5, -7.0, 2.0, 6.0, F2, F2 + stage_h, wood)
    b.wedge("Stage", -7.0, -7.0 + stage_h / math.tan(math.radians(10)), 2.0, 6.0, F2, F2 + stage_h, concrete,
            rising="-x", base=F2)

    posters = [((-9.99, 3.5, 1.6), 0.0), ((9.99, 3.0, 1.6), math.pi), ((-2.0, -6.99, 1.6), math.pi / 2),
               ((5.0, 6.99, 1.6), -math.pi / 2), ((-9.99, -3.0, F2 + 1.6), 0.0), ((9.99, 3.0, F2 + 1.6), math.pi)]
    for k, (c, yaw) in enumerate(posters):
        b.poster(c, yaw, k)
    for x_, y_ in ((-6.5, 3.5), (3.5, 3.5), (-4.0, -3.5), (6.0, -2.0)):
        b.sphere_light((x_, y_, F2 - SLAB - 0.3))
    for x_, y_ in ((-5.0, 3.5), (5.0, 3.5), (-5.0, -3.5), (5.0, -1.5)):
        b.sphere_light((x_, y_, TOP - 0.3))
    b.save()


# ---------------------------------------------------------------------------------------------------
# World 3 — terrain_park: outdoor rough terrain with stairs / slopes / stepping stones
# ---------------------------------------------------------------------------------------------------


def build_terrain_park():
    """40 m x 40 m outdoor park, robot at the origin facing +x. Pyramid stairs (+x), rough heightfield
    (-x), 15 deg hill (+y), stepping stones and curbs (-y), a raised plaza with stairs and a ramp,
    trees / rocks / fence as landmarks."""
    b = WorldBuilder("terrain_park")
    grass, concrete, wood = b.material("grass.png", 6.0), b.material("concrete.png", 2.0), b.material("wood.png", 1.2)
    bark, brick, hazard = b.material("bark.png", 1.0), b.material("brick.png", 1.5), b.material("hazard.png", 0.5)
    rng = np.random.default_rng(11)

    b.block("Ground", -20, 20, -20, 20, -0.3, 0.0, grass)

    # Pyramid stairs centred at (9, 0): 6 levels, 0.10 m rise, 0.5 m tread.
    for k in range(6):
        half = 3.0 - k * 0.5
        b.block("PyramidStairs", 9 - half, 9 + half, -half, half, 0.0, 0.10 * (k + 1), concrete)

    # Rough heightfield at x in [-14, -4], y in [-5, 5]: smooth bumps up to ~0.08 m, edges blended to 0.
    cell, nx, ny = 0.2, 51, 51
    field = sum(np.sin(np.linspace(0, f * math.pi, nx))[:, None] * np.cos(np.linspace(p, p + f * math.pi, ny))[None, :]
                for f, p in ((3, 0.3), (7, 1.1), (13, 2.0)))
    field = (field - field.min()) / (np.ptp(field) or 1.0) * 0.08 + rng.uniform(0, 0.015, (nx, ny))
    ramp = np.minimum(1.0, np.minimum.outer(np.minimum(np.arange(nx), np.arange(nx)[::-1]),
                                            np.minimum(np.arange(ny), np.arange(ny)[::-1])) / 5.0)
    b.heightfield("Rough", -14.0, -5.0, 10.0, 10.0, cell, field * ramp + 0.001, grass)

    # 15 deg hill at (0, 11): 3 m x 3 m plateau at 0.8 m, four ramps.
    h, cx, cy, p = 0.8, 0.0, 11.0, 1.5
    r = h / math.tan(math.radians(15))
    b.frustum("Hill", cx, cy, p, p + r, h, grass)

    # Stepping stones at y in [-13, -8], curbs further out.
    for i in range(8):
        for j in range(5):
            hgt = rng.uniform(0.04, 0.14)
            b.block("SteppingStones", -4 + i * 0.9, -3.4 + i * 0.9, -13 + j * 1.0, -12.4 + j * 1.0, 0.0, hgt, concrete)
    for i in range(5):
        b.block("Curbs", 6 + i * 1.4, 6.25 + i * 1.4, -14, -9, 0.0, 0.10 + 0.025 * i, hazard)

    # Raised plaza at (-12, 13): 1.2 m platform, stairs (0.15 m x 8) on +x side, 12 deg ramp on -y side.
    px0, px1, py0, py1, ph = -16.0, -10.0, 10.0, 16.0, 1.2
    b.block("Plaza", px0, px1, py0, py1, 0.0, ph, brick)
    b.stairs("Plaza", px1 + 8 * 0.3, 11.5, 14.5, 0.15, 0.30, 8, 0.0, concrete, wood, direction=-1)
    rr = ph / math.tan(math.radians(12))
    b.wedge("Plaza", px0, px0 + 2.5, py0 - rr, py0, 0.0, ph, concrete, rising="+y")
    b.block("Plaza", px0, px1, py1 - 0.2, py1, ph, ph + 1.0, brick)                # back wall on the plaza

    # Landmarks: trees (trunk collides, canopy visual), rocks, perimeter fence with gaps. Kept off the
    # spawn area and the terrain features above (rects incl. ramps, 1 m margin).
    keep_out = [(-4, 4, -4, 4), (5, 13, -4, 4), (-15, -3, -6, 6), (-5.5, 5.5, 5.5, 16.5), (-5.5, 4.5, -14.5, -6.5),
                (4.5, 13.5, -15, -8), (-17, -5, 3, 17)]

    def free(x_, y_):
        return not any(a <= x_ <= b_ and c <= y_ <= d for a, b_, c, d in keep_out)

    for _ in range(18):
        tx, ty = rng.uniform(-18, 18, 2)
        if not free(tx, ty):
            continue
        b.block("Trees", tx - 0.15, tx + 0.15, ty - 0.15, ty + 0.15, 0.0, 3.0, bark)
        b.box("Trees", (tx, ty, 3.4), (1.6, 1.6, 1.2), grass, yaw=rng.uniform(0, math.pi), collide=False)
    for _ in range(14):
        rx, ry = rng.uniform(-18, 18, 2)
        if not free(rx, ry):
            continue
        sx, sy, sz = rng.uniform(0.3, 1.0, 3)
        b.box("Rocks", (rx, ry, sz * 0.4), (sx, sy, sz), concrete, yaw=rng.uniform(0, math.pi))
    for s in np.arange(-19.5, 19.6, 3.0):
        for (x_, y_, along_x) in ((s, -19.9, True), (s, 19.9, True), (-19.9, s, False), (19.9, s, False)):
            if along_x:
                b.block("Fence", x_ - 1.2, x_ + 1.2, y_ - 0.05, y_ + 0.05, 0.0, 1.2, wood)
            else:
                b.block("Fence", x_ - 0.05, x_ + 0.05, y_ - 1.2, y_ + 1.2, 0.0, 1.2, wood)
    for k, (c, yaw) in enumerate([((-13.0, 15.79, ph + 0.5), -math.pi / 2), ((19.84, 4.5, 0.65), math.pi),
                                   ((4.5, 19.84, 0.65), -math.pi / 2)]):
        b.poster(c, yaw, k)
    b.save()


WORLD_BUILDERS = {
    "rbq_test": build_rbq_test,
    "stairs_course": build_stairs_course,
    "two_floor_office": build_two_floor_office,
    "terrain_park": build_terrain_park,
}


def main():
    names = sys.argv[1:] or list(WORLD_BUILDERS)
    unknown = [n for n in names if n not in WORLD_BUILDERS]
    if unknown:
        raise SystemExit(f"Unknown world(s) {unknown}. Options: {list(WORLD_BUILDERS)}")

    # pxr ships inside Isaac Sim: boot a headless kit just to get the USD libraries.
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        build_textures()
        for name in names:
            path = os.path.join(WORLDS_DIR, f"{name}.usda")
            if os.path.exists(path):
                os.remove(path)
            WORLD_BUILDERS[name]()
    finally:
        app.close()


if __name__ == "__main__":
    main()
