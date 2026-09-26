#!/usr/bin/env python3
"""Smoke-check the default glass paths and detect a blank refraction pass."""
import re
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path


def png_rgb(path):
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    offset = 8
    chunks = []
    while offset < len(data):
        length = struct.unpack_from(">I", data, offset)[0]
        kind = data[offset + 4:offset + 8]
        payload = data[offset + 8:offset + 8 + length]
        offset += length + 12
        if kind == b"IHDR":
            width, height, depth, color, _, _, _ = struct.unpack(">IIBBBBB", payload)
            assert (width, height, depth, color) == (320, 200, 8, 6)
        elif kind == b"IDAT":
            chunks.append(payload)
    packed = zlib.decompress(b"".join(chunks))
    stride = width * 4
    previous = bytearray(stride)
    rgb = bytearray()
    for row_index in range(height):
        start = row_index * (stride + 1)
        filtering = packed[start]
        row = bytearray(packed[start + 1:start + 1 + stride])
        for i in range(stride):
            left = row[i - 4] if i >= 4 else 0
            above = previous[i]
            upper_left = previous[i - 4] if i >= 4 else 0
            if filtering == 1:
                predictor = left
            elif filtering == 2:
                predictor = above
            elif filtering == 3:
                predictor = (left + above) // 2
            elif filtering == 4:
                estimate = left + above - upper_left
                distances = (abs(estimate - left), abs(estimate - above), abs(estimate - upper_left))
                predictor = (left, above, upper_left)[distances.index(min(distances))]
            else:
                assert filtering == 0
                predictor = 0
            row[i] = (row[i] + predictor) & 255
        for i in range(0, stride, 4):
            rgb.extend(row[i:i + 3])
        previous = row
    return rgb


def render(executable, output, extra, expected_volumes):
    result = subprocess.run([executable, "--benchmark", "--width=320", "--height=200",
                             "--frames=1", "--warmup=0", f"--out={output}", *extra],
                            capture_output=True, text=True, timeout=90, check=True)
    assert re.search(r"backend=(portable|vulkan) status=", result.stdout), result.stdout
    assert "samples_per_frame=10" in result.stdout, result.stdout
    assert "bounces=10" in result.stdout, result.stdout
    assert re.search(rf"volumes={expected_volumes}", result.stdout), result.stdout
    return png_rgb(output)


def main():
    with tempfile.TemporaryDirectory(prefix="rayrai-nested-glass-") as directory:
        path = Path(directory)
        glass = render(sys.argv[1], path / "glass.png", [], "[1-9][0-9]*")
        opaque = render(sys.argv[1], path / "opaque.png", ["--opaque-only"], "0")
        changed = sum(any(abs(a - b) > 12 for a, b in zip(glass[i:i + 3], opaque[i:i + 3]))
                      for i in range(0, len(glass), 3))
        assert changed > 320 * 200 // 200, f"Glass pass changed only {changed} pixels"
        print(f"Nested glass changed {changed} pixels compared with opaque-only control")


if __name__ == "__main__":
    main()
