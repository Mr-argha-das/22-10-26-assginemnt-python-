#!/usr/bin/env python3
# Copyright 2026 GIU-F1Tenth / AROLA Contributors
# SPDX-License-Identifier: MIT
"""Unit tests for GymBridge map stem resolution and 1-bit PNG conversion."""

from pathlib import Path
import sys
import tempfile

import numpy as np
from PIL import Image

PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parent
sys.modules.pop('f1tenth_gym_ros', None)
for p in (PKG_ROOT, REPO_ROOT / 'tools'):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from ros2_sim_harness.ros2_compat import (  # noqa: E402
    ensure_ros2_environment,
)

ensure_ros2_environment(str(REPO_ROOT))

from f1tenth_gym_ros.gym_bridge import resolve_gym_map_path  # noqa: E402


def test_resolve_berlin_map_and_convert_1bit_png():
    berlin_stem = resolve_gym_map_path('maps/berlin', '.png')
    assert Path(berlin_stem + '.yaml').is_file()
    assert Image.open(berlin_stem + '.png').mode == 'L'

    # Replicate upstream GIU-F1Tenth/f1tenth_maps 1-bit PNG bug in temp dir
    with tempfile.TemporaryDirectory() as tmp:
        stem = Path(tmp) / 'onebit_map'
        bits = np.zeros((32, 32), dtype=np.uint8)
        bits[8:24, 8:24] = 255
        Image.fromarray(bits, mode='L').convert('1').save(
            str(stem) + '.png'
        )
        (Path(str(stem) + '.yaml')).write_text(
            'image: onebit_map.png\nresolution: 0.05\norigin: [0, 0, 0]\n',
            encoding='utf-8',
        )
        assert Image.open(str(stem) + '.png').mode == '1'

        fixed_stem = resolve_gym_map_path(str(stem), '.png')
        converted = Image.open(fixed_stem + '.png')
        assert converted.mode == 'L'
        arr = np.asarray(converted)
        assert int(np.max(arr)) == 255 and int(np.min(arr)) == 0
