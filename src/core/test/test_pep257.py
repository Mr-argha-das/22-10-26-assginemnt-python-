# Copyright 2026 GIU-F1Tenth / AROLA Contributors
# SPDX-License-Identifier: MIT

import ast
from pathlib import Path

try:
    from ament_pep257.main import main as ament_pep257_main

    HAS_AMENT = True
except ImportError:
    HAS_AMENT = False


def test_pep257():
    if HAS_AMENT:
        rc = ament_pep257_main(argv=[".", "test"])
        assert rc == 0, "Found code style warnings / errors"
    else:
        pkg_root = Path(__file__).resolve().parents[1]
        py_files = list(pkg_root.rglob("*.py"))
        assert py_files, "Expected Python source files in package"
        for py_path in py_files:
            ast.parse(py_path.read_text(encoding="utf-8"))
