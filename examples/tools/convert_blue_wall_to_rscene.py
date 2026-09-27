#!/usr/bin/env python3
"""Convert the Blue Wall GLB to a lossless glTF payload and Engine .rscene."""

import argparse
import json
import math
import shutil
import struct
from pathlib import Path


RSCENE = """# Poly Haven Blue Wall, CC0. Mesh materials and textures are in blue_wall.gltf/bin.
raisim_engine_scene 1
time_step 0.002
asset_root .
environment 0.04705882 0.05490196 0.06666667 0.055 0.055 0.055 0 true false 10 hamburg_canal_1k.hdr backgroundMode=hdr gamma=2.2 exposure=0.88 bloomIntensity=0.10 bloomThreshold=1.35 bloomRadius=5 pbrEnvironmentIntensity=0.24 colorMode=unreal_preview ssao=true
rayrai_render preset=ultra custom=true colorMode=unreal_preview shadowBias=0.00045 shadowStrength=0.34 shadowPcfRadius=1.6 highFidelityPbr=true pbrToneMapping=true pbrExposure=0.88 depthOfField=false
object /World/BlueWall mesh 0 0 0 1 0 0 0 1 1 1 0.5 1 0 default engine_default true true false blue_wall.gltf visual_only false 1 18446744073709551615 id=blue_wall renderMeshPath=blue_wall.gltf visualUseMeshColor=false visualTwoSided=false
camera /World/Cameras/Main -0.10 -3.20 1.30 0.6652281657524527 -0.024271589933430954 0.02164561308873813 0.7459315282556056 52 0.05 50 1280 800 rgb true id=blue_wall_camera projection=perspective
"""


def light_records(document: dict, sidecar: dict) -> str:
    lights = document.get("extensions", {}).get("KHR_lights_punctual", {}).get("lights", [])
    records = []
    for index, node in enumerate(document.get("nodes", [])):
        reference = node.get("extensions", {}).get("KHR_lights_punctual")
        if reference is None:
            continue
        light = lights[reference["light"]]
        x, y, z, w = node.get("rotation", [0, 0, 0, 1])
        # A glTF punctual light points along its local -Z axis.
        direction = (-(2 * (x * z + w * y)),
                     -(2 * (y * z - w * x)),
                     -(1 - 2 * (x * x + y * y)))
        position = node.get("translation", [0, 0, 0])
        color = light.get("color", [1, 1, 1])
        spot = light.get("spot", {})
        records.append(
            f"light /World/Lights/Imported{index} "
            f"{' '.join(str(value) for value in direction)} "
            f"{light['intensity'] * 0.0022} id=imported_light_{index} "
            f"type={light['type']} position={','.join(map(str, position))} "
            f"color={','.join(map(str, color))} "
            f"coneAngle={math.degrees(spot.get('outerConeAngle', math.pi / 6))} "
            f"innerConeAngle={math.degrees(spot.get('innerConeAngle', 0))} shadows=true"
        )
    for index, light in enumerate(sidecar.get("lights", [])):
        records.append(
            f"light /World/Lights/Area{index} "
            f"{' '.join(map(str, light['direction']))} "
            f"{light['energy'] * 0.0022} id=area_light_{index} type=area "
            f"position={','.join(map(str, light['position']))} "
            f"color={','.join(map(str, light['color']))} "
            f"areaSize={light['size'][0]},{light['size'][1]},0 "
            f"areaRight={','.join(map(str, light['right']))} "
            f"areaUp={','.join(map(str, light['up']))} shadows=true"
        )
    return "\n".join(records) + "\n"


def convert(source: Path, output: Path) -> None:
    data = source.read_bytes()
    if len(data) < 28 or data[:4] != b"glTF":
        raise ValueError("source must be a glTF 2.0 GLB")
    version, total = struct.unpack_from("<II", data, 4)
    if version != 2 or total != len(data):
        raise ValueError("invalid GLB version or length")
    offset = 12
    chunks = []
    while offset < len(data):
        length, kind = struct.unpack_from("<I4s", data, offset)
        offset += 8
        chunks.append((kind, data[offset:offset + length]))
        offset += length
    if offset != len(data) or [kind for kind, _ in chunks] != [b"JSON", b"BIN\0"]:
        raise ValueError("expected one JSON chunk and one BIN chunk")
    document = json.loads(chunks[0][1])
    binary = chunks[1][1]
    if len(document.get("buffers", [])) != 1 or document["buffers"][0]["byteLength"] != len(binary):
        raise ValueError("unexpected GLB buffer layout")
    if any("uri" in image for image in document.get("images", [])):
        raise ValueError("expected all Blue Wall images to be embedded")
    document["buffers"][0]["uri"] = "blue_wall.bin"

    output.mkdir(parents=True, exist_ok=True)
    (output / "blue_wall.gltf").write_text(
        json.dumps(document, separators=(",", ":"), ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (output / "blue_wall.bin").write_bytes(binary)
    sidecar_path = source.with_name(source.name + ".rayrai_lights.json")
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    (output / "blue_wall.rscene").write_text(
        RSCENE + light_records(document, sidecar), encoding="utf-8")
    shutil.copyfile(sidecar_path,
                    output / "blue_wall.gltf.rayrai_lights.json")
    shutil.copyfile(source.parent / "textures" / "hamburg_canal_1k.hdr",
                    output / "hamburg_canal_1k.hdr")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="input blue_wall.glb")
    parser.add_argument("output", type=Path, help="destination directory")
    args = parser.parse_args()
    convert(args.source, args.output)
