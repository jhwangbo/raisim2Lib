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


def vec3(value: Vector) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


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
    wrote_scene = not args.lights_only
    if args.lights_only:
        export_rayrai_light_sidecar(output)
    elif args.format == "obj":
        export_obj(output, args.material_diffuse)
    else:
        export_gltf(output, args.format)
    if wrote_scene:
        print(f"Exported {args.format}: {output}")


if __name__ == "__main__":
    main()
