"""Exercise the compiled examples, including diagnostics and native XML export."""
import pathlib
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


def main():
    executables = sys.argv[1:4]
    key = sys.argv[4:]
    for executable in executables:
        help_result = subprocess.run([executable, "--help"], capture_output=True, text=True)
        assert help_result.returncode == 0 and "--headless" in help_result.stdout
        for args in [["--steps=-1"], ["--steps=2oops"], ["--port=65536"], ["--unknown"], ["--export"]]:
            result = subprocess.run([executable, *args], capture_output=True, text=True)
            assert result.returncode != 0 and result.stderr, (executable, args)
    with tempfile.TemporaryDirectory(prefix="raisim-tendon-examples-") as temporary:
        for executable, expected in zip(executables, [(2, 0, 0), (4, 0, 2), (3, 1, 1)]):
            export = pathlib.Path(temporary) / pathlib.Path(executable).stem / "tendon world.xml"
            result = subprocess.run([executable, "--headless", "--steps=1200", "--export", str(export), *key],
                                    capture_output=True, text=True)
            assert result.returncode == 0, result.stdout + result.stderr
            assert "max_motion=" in result.stdout and "steps=1200" in result.stdout
            world = ET.parse(export).getroot()
            assert world.tag == "raisim"
            models = world.findall(".//articulatedSystem")
            assert len(models) == expected[2]
            for model in models:
                urdf = pathlib.Path(model.get("urdf_path"))
                assert urdf.parent == export.parent
                assert urdf.is_file() and ET.parse(urdf).getroot().tag == "robot"
            tendons = world.findall(".//tendon")
            couplings = world.findall(".//tendon_coupling")
            assert (len(tendons), len(couplings)) == expected[:2]
            assert len({t.get("name") for t in tendons}) == expected[0]
            if pathlib.Path(executable).stem == "tendon_coupling":
                cables = {t.get("name") for t in tendons if t.get("type") == "spatial"}
                assert cables == {"joint_a", "joint_b"}
                assert couplings[0].get("first") == "joint_b" and couplings[0].get("second") == "joint_a"
    print("CLI validation, finite simulation, and XML export checks passed for all three examples.")


if __name__ == "__main__":
    main()
