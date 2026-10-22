# Copyright 2026 GIU-F1Tenth / AROLA Contributors
# SPDX-License-Identifier: MIT

from pathlib import Path
import subprocess
import sys

try:
    from ament_flake8.main import main_with_errors

    HAS_AMENT = True
except ImportError:
    HAS_AMENT = False


def test_flake8():
    if HAS_AMENT:
        rc, errors = main_with_errors(argv=[])
        assert rc == 0, f"Found {len(errors)} flake8 errors: {errors}"
    else:
        pkg_root = Path(__file__).resolve().parents[1]
        proc = subprocess.run(
            [sys.executable, "-m", "flake8", str(pkg_root)],
            capture_output=True,
            text=True,
        )
        assert proc.returncode == 0, proc.stdout + proc.stderr
