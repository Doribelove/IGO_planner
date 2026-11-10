#!/usr/bin/env python3
"""Create self-contained local ROS2 source, without publishing or modifying Git."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="New empty destination directory, outside the repository",
    )
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    repository = package.parent
    output = args.output.resolve()
    embedded_core = (package / "core/src/core.cpp").exists()
    source_root = package if embedded_core else repository
    if output == source_root or source_root in output.parents:
        parser.error("output must be outside the source tree")
    if output.exists():
        parser.error(
            "output already exists; choose a new path to avoid overwriting work"
        )
    core = package / "core" if embedded_core else repository
    required = [
        "include/igo_local_planner/core.h",
        "src/core.cpp",
        "test/scenarios.cpp",
    ]
    for name in required:
        if not (core / name).is_file():
            parser.error("missing shared core file: " + name)
    shutil.copytree(
        package,
        output,
        ignore=shutil.ignore_patterns(
            ".git", ".hg", ".svn", "build", "install", "log", "__pycache__", "*.pyc"
        ),
    )
    hashes = {}
    for name in required:
        destination = output / "core" / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(core / name, destination)
        hashes[name] = hashlib.sha256(destination.read_bytes()).hexdigest()
    manifest = {
        "package": "igo_nav2_controller",
        "version": "0.1.0",
        "upstream_repository": "https://github.com/Doribelove/IGO_planner",
        "original_core_commit": "ac87ef450bde039564b2d555cc27f53b7c5c7221",
        "core_sha256": hashes,
        "publication_performed": False,
    }
    (output / "CORE_PROVENANCE.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(
        json.dumps(
            {
                "source_directory": str(output),
                "core_sha256": hashes,
                "publication_performed": False,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
