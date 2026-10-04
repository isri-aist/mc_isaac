"""Convert a URDF into a single-file USD usable by mc_isaac (Isaac Sim URDF importer).

Run with the Isaac Sim python (package:// URIs must be resolvable or replaced by absolute paths):
  /isaacsim/python.sh mc_isaac_urdf_to_usd.py robot.urdf robot.usd

- fixed joints are not merged, so link/joint names match the mc_rtc robot module
- joint drives are force drives (the importer default "acceleration" scales the gains by the joint inertia):
  set the real gains in the mc_isaac robot yaml (drives)
- URDF links without <inertial> (sensor frames) stay (almost) massless instead of the importer default 1 kg
- URDF mimic joints become regular driven joints (no gains: set them in the yaml), mc_rtc commands them
- the importer layers are flattened into one file; the mesh textures are copied to <usd folder>/textures and
  referenced relatively (list them in the description extra_files so mc_isaac uploads them)
"""
import os
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

from isaacsim import SimulationApp

if len(sys.argv) != 3:
    sys.exit(__doc__)
urdf, out = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
from isaacsim.asset.importer.urdf import _urdf  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdPhysics  # noqa: E402

_, config = omni.kit.commands.execute("URDFCreateImportConfig")
config.merge_fixed_joints = False
config.fix_base = False
config.make_default_prim = True
config.self_collision = False
config.create_physics_scene = False
config.import_inertia_tensor = True
config.convex_decomp = False
config.collision_from_visuals = False
config.distance_scale = 1.0
config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION

tmp = os.path.join(tempfile.mkdtemp(), os.path.basename(out))
omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf, import_config=config, dest_path=tmp)
os.makedirs(os.path.dirname(out), exist_ok=True)
Usd.Stage.Open(tmp).Export(out)

# the importer writes the textures in its temporary folder and references them with absolute paths
layer = Sdf.Layer.FindOrOpen(out)
textures, missing = set(), set()


def relocate_texture(path):
    spec = layer.GetAttributeAtPath(path) if path.IsPropertyPath() else None
    if spec is None or spec.typeName != Sdf.ValueTypeNames.Asset or not spec.default:
        return
    src = spec.default.path
    if not os.path.isabs(src):
        return
    if not os.path.isfile(src):
        missing.add(src)
        return
    name = os.path.basename(src)
    os.makedirs(os.path.join(os.path.dirname(out), "textures"), exist_ok=True)
    shutil.copyfile(src, os.path.join(os.path.dirname(out), "textures", name))
    spec.default = Sdf.AssetPath(f"textures/{name}")
    textures.add(name)


layer.Traverse(Sdf.Path.absoluteRootPath, relocate_texture)
layer.Save()

stage = Usd.Stage.Open(out)
drives = 0
# URDF links without <inertial> (sensor frames...) are massless for mc_rtc, the importer gives them 1 kg
massless = {link.get("name") for link in ET.parse(urdf).getroot().findall("link") if link.find("inertial") is None}
lightened = mimics = 0
for prim in stage.Traverse():
    # mc_rtc commands the URDF mimic joints itself: make them regular driven joints (the importer PhysX mimic
    # constraints left some followers swinging)
    mimic_schemas = [s for s in prim.GetAppliedSchemas() if s.startswith("PhysxMimicJointAPI")]
    if mimic_schemas:
        for schema in mimic_schemas:
            prim.RemoveAppliedSchema(schema)
        for attr in prim.GetAttributes():
            if attr.GetName().startswith("physxMimicJoint:"):
                prim.RemoveProperty(attr.GetName())
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateDampingAttr().Set(0.0)
        mimics += 1
    for kind in ("angular", "linear"):
        if prim.HasAPI(UsdPhysics.DriveAPI, kind):
            UsdPhysics.DriveAPI(prim, kind).CreateTypeAttr().Set("force")
            drives += 1
    if prim.GetName() in massless and prim.HasAPI(UsdPhysics.RigidBodyAPI):
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr().Set(1e-4)
        mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1e-8, 1e-8, 1e-8))
        mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0, 0, 0))
        lightened += 1
stage.GetRootLayer().Save()
prims = list(stage.Traverse(Usd.TraverseInstanceProxies()))
print(f"[mc_isaac_urdf_to_usd] {out}: default prim {stage.GetDefaultPrim().GetPath()}, "
      f"{sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims)} bodies, {drives} force drives, {lightened} massless links, {mimics} mimic joints made driven, "
      f"{sum(p.IsA(UsdPhysics.FixedJoint) for p in prims)} fixed joints, "
      f"{sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims)} collision prims", flush=True)
if textures:
    print(f"[mc_isaac_urdf_to_usd] {len(textures)} textures, description entry:\n"
          f"extra_files: [{', '.join(f'textures/{t}' for t in sorted(textures))}]", flush=True)
for path in sorted(missing):
    print(f"[mc_isaac_urdf_to_usd] WARNING: texture not found: {path}", flush=True)
app.close()
