import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
CMAKE_LISTS_TEXT = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
VERSION_MATCH = re.search(
    r"^set\(_raisim_expected_version ([^)]+)\)",
    CMAKE_LISTS_TEXT,
    flags=re.MULTILINE,
)
if VERSION_MATCH is None:
    raise RuntimeError("Could not read _raisim_expected_version from CMakeLists.txt")
RAISIM_VERSION = VERSION_MATCH.group(1)


@unittest.skipIf(os.name == "nt", "The OpenUSD alias is used by Unix release packages")
class CMakeInstallTest(unittest.TestCase):
    def _write_file(self, path: Path, contents: str = "fixture\n") -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")

    def _create_fixture(self, root: Path, openusd_layout: str) -> Path:
        source_dir = root / "source"
        source_dir.mkdir()
        shutil.copy2(REPO_ROOT / "CMakeLists.txt", source_dir / "CMakeLists.txt")
        shutil.copy2(REPO_ROOT / "package.xml", source_dir / "package.xml")

        self._write_file(
            source_dir / "raisim/lib/cmake/raisim/raisimConfig-version.cmake",
            f'set(PACKAGE_VERSION "{RAISIM_VERSION}")\n',
        )
        self._write_file(source_dir / "raisim/lib/cmake/raisim/raisimConfig.cmake")
        self._write_file(source_dir / "raisim/lib/libraisim.fixture")
        self._write_file(source_dir / "raisim/lib/openusd/raisim-openusd.fixture")
        self._write_file(source_dir / "raisim/include/raisim/fixture.hpp")

        self._write_file(source_dir / "rayrai/lib/cmake/rayrai/rayraiConfig.cmake")
        self._write_file(source_dir / "rayrai/lib/librayrai.fixture")
        self._write_file(source_dir / "rayrai/include/rayrai/fixture.hpp")

        rayrai_openusd = source_dir / "rayrai/lib/openusd"
        if openusd_layout == "shared-symlink":
            rayrai_openusd.symlink_to("../../raisim/lib/openusd", target_is_directory=True)
        elif openusd_layout == "distinct-directory":
            self._write_file(rayrai_openusd / "rayrai-openusd.fixture")
        elif openusd_layout == "foreign-symlink":
            self._write_file(source_dir / "foreign/openusd/foreign-openusd.fixture")
            rayrai_openusd.symlink_to("../../foreign/openusd", target_is_directory=True)
        else:
            self.fail(f"Unknown OpenUSD fixture layout: {openusd_layout}")

        return source_dir

    def _configure(self, source_dir: Path, build_dir: Path, configured_prefix: Path):
        return subprocess.run(
            [
                "cmake",
                "-S",
                str(source_dir),
                "-B",
                str(build_dir),
                "-DRAISIM_EXAMPLE=OFF",
                "-DRAISIM_PY=OFF",
                "-DRAISIM_DOC=OFF",
                f"-DCMAKE_INSTALL_PREFIX={configured_prefix}",
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

    def _install_twice(self, build_dir: Path, install_prefix: Path) -> None:
        command = ["cmake", "--install", str(build_dir), "--prefix", str(install_prefix)]
        for _ in range(2):
            result = subprocess.run(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            self.assertEqual(result.returncode, 0, result.stdout)

    def test_shared_openusd_symlink_is_not_reinstalled(self) -> None:
        with tempfile.TemporaryDirectory(prefix="raisim2lib-install-test.") as temp_dir:
            root = Path(temp_dir)
            source_dir = self._create_fixture(root, "shared-symlink")
            build_dir = root / "build"
            configured_prefix = root / "configured-prefix"
            install_prefix = root / "install-prefix"

            result = self._configure(source_dir, build_dir, configured_prefix)
            self.assertEqual(result.returncode, 0, result.stdout)
            self._install_twice(build_dir, install_prefix)

            installed_openusd = install_prefix / "lib/openusd"
            self.assertTrue(installed_openusd.is_dir())
            self.assertFalse(installed_openusd.is_symlink())
            self.assertTrue((installed_openusd / "raisim-openusd.fixture").is_file())
            self.assertTrue((install_prefix / "lib/librayrai.fixture").is_file())
            self.assertFalse(configured_prefix.exists())

    def test_distinct_rayrai_openusd_runtime_is_installed(self) -> None:
        with tempfile.TemporaryDirectory(prefix="raisim2lib-install-test.") as temp_dir:
            root = Path(temp_dir)
            source_dir = self._create_fixture(root, "distinct-directory")
            build_dir = root / "build"
            configured_prefix = root / "configured-prefix"
            install_prefix = root / "install-prefix"

            result = self._configure(source_dir, build_dir, configured_prefix)
            self.assertEqual(result.returncode, 0, result.stdout)
            self._install_twice(build_dir, install_prefix)

            installed_openusd = install_prefix / "lib/openusd"
            self.assertTrue((installed_openusd / "raisim-openusd.fixture").is_file())
            self.assertTrue((installed_openusd / "rayrai-openusd.fixture").is_file())

    def test_foreign_openusd_symlink_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory(prefix="raisim2lib-install-test.") as temp_dir:
            root = Path(temp_dir)
            source_dir = self._create_fixture(root, "foreign-symlink")
            result = self._configure(
                source_dir,
                root / "build",
                root / "configured-prefix",
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("rayrai OpenUSD runtime symlink must resolve to", result.stdout)


if __name__ == "__main__":
    unittest.main()
