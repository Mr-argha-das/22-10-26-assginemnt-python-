# Copyright 2026 GIU-F1Tenth / AROLA Contributors
# SPDX-License-Identifier: MIT

from pathlib import Path

try:
    from ament_copyright.main import main as ament_copyright_main

    HAS_AMENT = True
except ImportError:
    HAS_AMENT = False


def test_copyright():
    if HAS_AMENT:
        rc = ament_copyright_main(argv=[".", "test"])
        assert rc == 0, "Found copyright errors"
    else:
        pkg_root = Path(__file__).resolve().parents[1]
        assert (pkg_root / "package.xml").is_file()
