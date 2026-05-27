#!/usr/bin/env python3
"""General Blender scene exporter for RayRai examples."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Output path. Defaults next to the .blend file.")
    parser.add_argument(
        "--format",
        choices=("obj", "glb", "gltf"),
        default="glb",
        help="Export format.",
    )
    parser.add_argument(
        "--material-diffuse",
        action="append",
        default=[],
        metavar="MATERIAL=TEXTURE",
        help="OBJ-only MTL map_Kd override. May be repeated.",
    )
    parser.add_argument(
        "--lights-only",
        action="store_true",
        help="Only write RayRai light sidecar metadata for an existing glTF/GLB export.",
    )
    parser.add_argument(
        "--probes-only",
        action="store_true",
        help="Only write RayRai reflection probe sidecar metadata for an existing glTF/GLB export.",
    )
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []
    return parser.parse_args(argv)


def renderable_objects() -> list[bpy.types.Object]:
    return [
        obj
        for obj in bpy.context.scene.objects
        if obj.type in {"MESH", "CURVE", "SURFACE", "FONT", "META"}
        and not obj.hide_get()
        and not obj.hide_render
    ]


def default_output_path(fmt: str) -> Path:
    blend = Path(bpy.data.filepath)
    suffix = ".obj" if fmt == "obj" else f".{fmt}"
    return blend.with_suffix(suffix)


def export_obj(output: Path, material_diffuse: list[str]) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    for obj in renderable_objects():
        obj.select_set(True)
    bpy.context.view_layer.objects.active = next((obj for obj in renderable_objects()), None)
    bpy.ops.wm.obj_export(
        filepath=str(output),
        export_selected_objects=True,
        export_materials=True,
        export_uv=True,
        export_normals=True,
        export_triangulated_mesh=True,
        apply_modifiers=True,
        path_mode="RELATIVE",
        forward_axis="Y",
        up_axis="Z",
    )
    if material_diffuse:
        patch_mtl(output.with_suffix(".mtl"), material_diffuse)


def patch_mtl(path: Path, overrides: list[str]) -> None:
    if not path.exists():
        return
    mapping: dict[str, str] = {}
    for item in overrides:
        name, sep, texture = item.partition("=")
        if sep:
            mapping[name.strip()] = texture.strip()
    if not mapping:
        return

    lines = path.read_text(encoding="utf-8").splitlines()
    out: list[str] = []
    current: str | None = None
    wrote_map = False
    for line in lines:
        if line.startswith("newmtl "):
            if current in mapping and not wrote_map:
                out.append(f"map_Kd {mapping[current]}")
            current = line.removeprefix("newmtl ").strip()
            wrote_map = False
            out.append(line)
            continue
        if current in mapping and line.startswith("map_Kd "):
            if not wrote_map:
                out.append(f"map_Kd {mapping[current]}")
                wrote_map = True
            continue
        out.append(line)
    if current in mapping and not wrote_map:
        out.append(f"map_Kd {mapping[current]}")
    path.write_text("\n".join(out) + "\n", encoding="utf-8")


def export_gltf(output: Path, fmt: str) -> None:
    try:
        bpy.ops.export_scene.gltf(
            filepath=str(output),
            export_format="GLB" if fmt == "glb" else "GLTF_SEPARATE",
            export_yup=False,
            export_apply=True,
            export_lights=True,
            export_cameras=False,
            export_materials="EXPORT",
            export_image_format="AUTO",
        )
    finally:
        export_rayrai_light_sidecar(output)
        export_rayrai_probe_sidecar(output)


def vec3(value: Vector) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def custom_property(obj: bpy.types.ID, name: str, default: object = None) -> object:
    try:
        return obj.get(name, default)
    except AttributeError:
        return default


def custom_bool(obj: bpy.types.ID, name: str, default: bool = False) -> bool:
    value = custom_property(obj, name, default)
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)


def custom_float(obj: bpy.types.ID, name: str, default: float) -> float:
    value = custom_property(obj, name, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def custom_int(obj: bpy.types.ID, name: str, default: int, minimum: int, maximum: int) -> int:
    value = custom_property(obj, name, default)
    try:
        return max(minimum, min(maximum, int(value)))
    except (TypeError, ValueError):
        return default


def custom_vec3(obj: bpy.types.ID, name: str) -> list[float] | None:
    value = custom_property(obj, name)
    if value is None or isinstance(value, (str, bytes)):
        return None
    try:
        values = [float(value[i]) for i in range(3)]
    except (TypeError, ValueError, IndexError):
        return None
    return values


def rayrai_probe_objects() -> list[bpy.types.Object]:
    probes: list[bpy.types.Object] = []
    for obj in bpy.context.scene.objects:
        if obj.hide_get() or obj.hide_render:
            continue
        if custom_bool(obj, "rayrai_reflection_probe", False) or obj.name.startswith("RAYRAI_PROBE"):
            probes.append(obj)
    return probes


def object_world_aabb(obj: bpy.types.Object) -> tuple[list[float], list[float]] | None:
    corners: list[Vector] = []
    if getattr(obj, "type", None) == "EMPTY":
        corners = [
            obj.matrix_world @ Vector((x, y, z))
            for x in (-0.5, 0.5)
            for y in (-0.5, 0.5)
            for z in (-0.5, 0.5)
        ]
    elif getattr(obj, "bound_box", None):
        corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    if not corners:
        return None

    min_corner = Vector((min(c.x for c in corners), min(c.y for c in corners), min(c.z for c in corners)))
    max_corner = Vector((max(c.x for c in corners), max(c.y for c in corners), max(c.z for c in corners)))
    if min((max_corner - min_corner).x, (max_corner - min_corner).y, (max_corner - min_corner).z) <= 1.0e-4:
        return None
    return vec3(min_corner), vec3(max_corner)


def probe_filter_settings(obj: bpy.types.Object) -> dict[str, object] | None:
    if not custom_bool(obj, "rayrai_probe_filter", False):
        return None
    return {
        "irradiance_resolution": custom_int(obj, "rayrai_probe_irradiance_resolution", 16, 0, 128),
        "irradiance_samples": custom_int(obj, "rayrai_probe_irradiance_samples", 64, 0, 4096),
        "prefiltered_resolution": custom_int(obj, "rayrai_probe_prefiltered_resolution", 64, 0, 512),
        "prefiltered_mip_levels": custom_int(obj, "rayrai_probe_prefiltered_mip_levels", 5, 0, 16),
        "prefiltered_samples": custom_int(obj, "rayrai_probe_prefiltered_samples", 64, 0, 4096),
        "brdf_lut_size": custom_int(obj, "rayrai_probe_brdf_lut_size", 128, 0, 512),
        "brdf_lut_samples": custom_int(obj, "rayrai_probe_brdf_lut_samples", 128, 0, 4096),
    }


def export_rayrai_probe_sidecar(output: Path) -> None:
    """Write explicit static reflection probe authoring metadata.

    Add a Blender object with custom property ``rayrai_reflection_probe=true``.
    Empty cube objects are useful for box-projected room probes; their world-space
    bounding box is exported when ``rayrai_probe_box_projection=true``.
    """

    probes: list[dict[str, object]] = []
    for obj in rayrai_probe_objects():
        probe: dict[str, object] = {
            "name": obj.name,
            "position": vec3(obj.matrix_world.translation),
            "radius": max(0.0, custom_float(obj, "rayrai_probe_radius", 5.0)),
            "strength": max(0.0, custom_float(obj, "rayrai_probe_strength", 1.0)),
            "capture": {
                "resolution": custom_int(obj, "rayrai_probe_resolution", 64, 16, 512),
                "visualization_objects": custom_bool(obj, "rayrai_probe_visualization_objects", True),
                "coordinate_frames": custom_bool(obj, "rayrai_probe_coordinate_frames", False),
                "point_clouds": custom_bool(obj, "rayrai_probe_point_clouds", False),
                "shadows": custom_bool(obj, "rayrai_probe_shadows", False),
                "environment_background": custom_bool(obj, "rayrai_probe_environment_background", False),
                "environment_exposure": max(
                    0.0, custom_float(obj, "rayrai_probe_environment_exposure", 1.0)
                ),
            },
        }

        box_projection = custom_bool(obj, "rayrai_probe_box_projection", False)
        box_min = custom_vec3(obj, "rayrai_probe_box_min")
        box_max = custom_vec3(obj, "rayrai_probe_box_max")
        if box_projection and (box_min is None or box_max is None):
            aabb = object_world_aabb(obj)
            if aabb is not None:
                box_min, box_max = aabb
        if box_projection and box_min is not None and box_max is not None:
            if min(box_max[i] - box_min[i] for i in range(3)) > 1.0e-4:
                probe["box_projection"] = True
                probe["box_min"] = box_min
                probe["box_max"] = box_max

        filter_settings = probe_filter_settings(obj)
        if filter_settings is not None:
            probe["filter"] = filter_settings
        probes.append(probe)

    sidecar = output.with_suffix(output.suffix + ".rayrai_probes.json")
    if not probes:
        if sidecar.exists():
            sidecar.unlink()
        return

    quality: dict[str, object] = {}
    scene = bpy.context.scene
    if custom_property(scene, "rayrai_probe_max_reflection_probes_per_selection") is not None:
        quality["max_reflection_probes_per_selection"] = custom_int(
            scene, "rayrai_probe_max_reflection_probes_per_selection", 1, 1, 2
        )
    if custom_property(scene, "rayrai_probe_min_reflection_probe_weight") is not None:
        quality["min_reflection_probe_weight"] = max(
            0.0, custom_float(scene, "rayrai_probe_min_reflection_probe_weight", 0.0)
        )
    if custom_property(scene, "rayrai_probe_filtering_enabled") is not None:
        quality["filtering_enabled"] = custom_bool(scene, "rayrai_probe_filtering_enabled", False)

    document: dict[str, object] = {
        "version": 1,
        "coordinate_system": "z_up",
        "probes": probes,
    }
    if quality:
        document["quality"] = quality
    sidecar.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    print(f"Exported RayRai reflection probe sidecar: {sidecar}")


def export_rayrai_light_sidecar(output: Path) -> None:
    """Preserve Blender lights that glTF cannot represent directly.

    The standard KHR_lights_punctual extension only supports directional, point,
    and spot lights. Blender area lights are important for authored indoor scenes,
    so RayRai stores them in a small sidecar next to the exported glTF/GLB.
    """

    lights: list[dict[str, object]] = []
    for obj in bpy.context.scene.objects:
        if obj.type != "LIGHT" or obj.hide_get() or obj.hide_render:
            continue
        light = obj.data
        if light.type != "AREA":
            continue

        matrix = obj.matrix_world
        quat = matrix.to_quaternion()
        direction = quat @ Vector((0.0, 0.0, -1.0))
        right = quat @ Vector((1.0, 0.0, 0.0))
        up = quat @ Vector((0.0, 1.0, 0.0))
        size_x = float(getattr(light, "size", 1.0))
        size_y = float(getattr(light, "size_y", size_x))
        if getattr(light, "shape", "SQUARE") in {"SQUARE", "DISK"}:
            size_y = size_x

        lights.append(
            {
                "name": obj.name,
                "type": "area",
                "position": vec3(matrix.translation),
                "direction": vec3(direction.normalized()),
                "right": vec3(right.normalized()),
                "up": vec3(up.normalized()),
                "size": [size_x, size_y],
                "energy": float(getattr(light, "energy", 0.0)),
                "color": [float(c) for c in getattr(light, "color", (1.0, 1.0, 1.0))],
                "shape": getattr(light, "shape", "RECTANGLE"),
            }
        )

    sidecar = output.with_suffix(output.suffix + ".rayrai_lights.json")
    if not lights:
        if sidecar.exists():
            sidecar.unlink()
        return
    sidecar.write_text(
        json.dumps({"version": 1, "coordinate_system": "z_up", "lights": lights}, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Exported RayRai light sidecar: {sidecar}")


def main() -> None:
    args = parse_args()
    output = args.output or default_output_path(args.format)
    output.parent.mkdir(parents=True, exist_ok=True)
    wrote_scene = not args.lights_only and not args.probes_only
    if args.lights_only:
        export_rayrai_light_sidecar(output)
    elif args.probes_only:
        export_rayrai_probe_sidecar(output)
    elif args.format == "obj":
        export_obj(output, args.material_diffuse)
    else:
        export_gltf(output, args.format)
    if wrote_scene:
        print(f"Exported {args.format}: {output}")


if __name__ == "__main__":
    main()
