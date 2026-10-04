"""Compiler selection must preserve symlink names that select driver modes."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location("build_tool", Path(__file__).resolve().parents[1] / "tools/build.py")
build = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(build)


class CompilerPathTests(unittest.TestCase):
    def test_path_lookup_keeps_cxx_name(self):
        path = os.path.join("toolchain", "clang++")
        with patch.object(build.shutil, "which", return_value=path), \
             patch.object(Path, "resolve", side_effect=AssertionError("must not resolve compiler symlinks")):
            self.assertEqual(build.compiler_path("clang++"), os.path.abspath(path))

    def test_cxx_environment_keeps_wrapper_name(self):
        path = os.path.join("compiler tools", "c++")
        with patch.dict(os.environ, {"CXX": "c++"}), patch.object(build.shutil, "which", return_value=path):
            self.assertEqual(build.compiler_path(None), os.path.abspath(path))
            build.shutil.which.assert_called_once_with("c++")

    def test_actual_symlink_keeps_cxx_driver(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "clang"
            target.write_text("compiler fixture", encoding="utf-8")
            target.chmod(0o755)
            alias = Path(directory) / "clang++"
            try:
                alias.symlink_to(target.name)
            except (OSError, NotImplementedError):
                self.skipTest("symlinks unavailable to this user")
            with patch.object(build.shutil, "which", return_value=str(alias)):
                selected = build.compiler_path("clang++")
            self.assertEqual(selected, str(alias.absolute()))
            self.assertNotEqual(selected, str(target.resolve()))

    def test_missing_explicit_compiler_is_an_error(self):
        with patch.object(build.shutil, "which", return_value=None):
            with self.assertRaises(RuntimeError):
                build.compiler_path("missing-c++")


if __name__ == "__main__":
    unittest.main()
