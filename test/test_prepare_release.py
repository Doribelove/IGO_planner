#!/usr/bin/env python3
"""Exercise source preparation using only synthetic repository metadata."""

import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/prepare_release.py"
CORE_FILES = {
    "include/igo_local_planner/core.h": b"// synthetic core header\n",
    "src/core.cpp": b"// synthetic core implementation\n",
    "test/scenarios.cpp": b"// synthetic core scenarios\n",
}
IGNORED_NAMES = {".git", ".hg", ".svn", "build", "install", "log", "__pycache__"}


def snapshot(root):
    """Record all fixture bytes and directories, including fake VCS metadata."""
    return {
        str(path.relative_to(root)): path.read_bytes() if path.is_file() else None
        for path in root.rglob("*")
    }


class PrepareReleaseTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.repository = self.root / "repository"
        self.output = self.root / "source-bundle"

    def make_fixture(self, embedded_core):
        package = self.repository if embedded_core else self.repository / "nav2"
        core = package / "core" if embedded_core else self.repository
        (package / "scripts").mkdir(parents=True)
        shutil.copy2(SCRIPT, package / "scripts/prepare_release.py")
        (package / "package.xml").write_text("<package>fixture</package>\n")
        for name, contents in CORE_FILES.items():
            path = core / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(contents)

        # These are deliberately fake credentials, never read from a real checkout.
        fixtures = {
            ".git/config": "FAKE_GIT_CREDENTIAL_SENTINEL\n",
            ".hg/store/data": "FAKE_HG_SENTINEL\n",
            ".svn/wc.db": "FAKE_SVN_SENTINEL\n",
        }
        for name, contents in fixtures.items():
            path = self.repository / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(contents)
        if not embedded_core:
            (package / ".git").write_text("gitdir: FAKE_WORKTREE_SENTINEL\n")
        for base in (package, package / "nested"):
            base.mkdir(exist_ok=True)
            for name in IGNORED_NAMES - {".git"}:
                path = base / name / "sentinel"
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("FAKE_IGNORED_SENTINEL\n")
            (base / "cache.pyc").write_bytes(b"FAKE_BYTECODE\n")
        (package / "nested/.git").write_text("gitdir: FAKE_NESTED_SENTINEL\n")
        return package

    def run_preparation(self, package, output=None):
        return subprocess.run(
            [
                sys.executable,
                str(package / "scripts/prepare_release.py"),
                "--output",
                str(self.output if output is None else output),
            ],
            cwd=package,
            capture_output=True,
            text=True,
            check=False,
        )

    def assert_prepared(self, embedded_core):
        package = self.make_fixture(embedded_core)
        before = snapshot(self.repository)
        result = self.run_preparation(package)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(snapshot(self.repository), before)
        self.assertEqual(
            (self.output / "package.xml").read_bytes(),
            (package / "package.xml").read_bytes(),
        )
        for path in self.output.rglob("*"):
            self.assertNotIn(path.name, IGNORED_NAMES, str(path))
            self.assertNotEqual(path.suffix, ".pyc", str(path))
        hashes = {}
        for name, contents in CORE_FILES.items():
            self.assertEqual((self.output / "core" / name).read_bytes(), contents)
            hashes[name] = hashlib.sha256(contents).hexdigest()
        self.assertEqual(
            json.loads((self.output / "CORE_PROVENANCE.json").read_text()),
            {
                "package": "igo_nav2_controller",
                "version": "0.1.0",
                "upstream_repository": "https://github.com/Doribelove/IGO_planner",
                "original_core_commit": "ac87ef450bde039564b2d555cc27f53b7c5c7221",
                "core_sha256": hashes,
                "publication_performed": False,
            },
        )
        self.assertEqual(
            json.loads(result.stdout),
            {
                "source_directory": str(self.output),
                "core_sha256": hashes,
                "publication_performed": False,
            },
        )

    def test_embedded_core_root_layout(self):
        self.assert_prepared(embedded_core=True)

    def test_shared_core_nav2_layout(self):
        self.assert_prepared(embedded_core=False)

    def test_existing_output_is_not_overwritten(self):
        package = self.make_fixture(embedded_core=True)
        self.output.mkdir()
        (self.output / "keep.txt").write_text("existing work\n")
        source_before = snapshot(self.repository)
        output_before = snapshot(self.output)
        result = self.run_preparation(package)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("output already exists", result.stderr)
        self.assertEqual(snapshot(self.output), output_before)
        self.assertEqual(snapshot(self.repository), source_before)

    def test_output_inside_source_is_rejected(self):
        package = self.make_fixture(embedded_core=True)
        before = snapshot(self.repository)
        result = self.run_preparation(package, package / "bundle")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("output must be outside the source tree", result.stderr)
        self.assertEqual(snapshot(self.repository), before)


if __name__ == "__main__":
    unittest.main()
