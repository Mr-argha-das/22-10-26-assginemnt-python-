#!/usr/bin/env python3
"""
Occupancy grid map loader, coordinate frame transforms, and Euclidean signed
distance field utilities for F1TENTH Berlin track planning and validation.
"""

from dataclasses import dataclass
import os
from typing import Tuple

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
import yaml


@dataclass
class MapMetadata:
    """ROS map_server YAML metadata."""

    yaml_path: str
    image_path: str
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: int
    occupied_thresh: float
    free_thresh: float
    width: int
    height: int


class OccupancyGridMap:
    """
    Validated 2D occupancy grid with exact Euclidean distance field (in meters)
    and bidirectional ROS world <-> image grid coordinate transforms.
    """

    def __init__(self, yaml_path: str):
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f'Map YAML file not found: {yaml_path}')

        with open(yaml_path, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)

        img_rel = raw['image']
        img_path = (
            img_rel
            if os.path.isabs(img_rel)
            else os.path.join(
                os.path.dirname(os.path.abspath(yaml_path)), img_rel
            )
        )
        if not os.path.exists(img_path):
            raise FileNotFoundError(f'Map image file not found: {img_path}')

        pil_img = Image.open(img_path)
        if pil_img.mode == '1':
            gray = np.where(np.asarray(pil_img, dtype=bool), 255, 0).astype(
                np.uint8
            )
        else:
            gray = np.asarray(pil_img.convert('L'), dtype=np.uint8)

        height, width = gray.shape
        resolution = float(raw['resolution'])
        origin = raw['origin']
        negate = int(raw.get('negate', 0))
        occ_thresh = float(raw.get('occupied_thresh', 0.65))
        free_thresh = float(raw.get('free_thresh', 0.196))

        if negate == 1:
            occ_prob = gray.astype(np.float64) / 255.0
        else:
            occ_prob = (255.0 - gray.astype(np.float64)) / 255.0

        self.gray = gray
        self.occ_prob = occ_prob
        self.free_mask: np.ndarray = occ_prob < free_thresh
        self.occupied_mask: np.ndarray = occ_prob > occ_thresh

        self.meta = MapMetadata(
            yaml_path=os.path.abspath(yaml_path),
            image_path=os.path.abspath(img_path),
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            negate=negate,
            occupied_thresh=occ_thresh,
            free_thresh=free_thresh,
            width=width,
            height=height,
        )

        # Exact Euclidean distance to nearest non-free cell in meters
        self.clearance_m: np.ndarray = (
            distance_transform_edt(self.free_mask) * resolution
        )
        # Spatial gradient of clearance in image coordinates (d/drow, d/dcol)
        grad_r, grad_c = np.gradient(self.clearance_m)
        # World coordinates: dx = +res * dcol, dy = -res * drow
        self.grad_x: np.ndarray = grad_c / resolution
        self.grad_y: np.ndarray = -grad_r / resolution

        # Identify connected components of occupied cells
        occ_labels, num_occ = label(~self.free_mask)
        self.occ_labels = occ_labels
        self.num_occupied_components = int(num_occ)

        free_labels, num_free = label(self.free_mask)
        self.free_labels = free_labels
        self.num_free_components = int(num_free)

    def world_to_grid_float(
        self, x: np.ndarray, y: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Convert world map (x, y) meters to floating-point (row, col)."""
        col = (np.asarray(x, dtype=np.float64) - self.meta.origin_x) / (
            self.meta.resolution
        )
        row = (
            self.meta.height
            - 1.0
            - (np.asarray(y, dtype=np.float64) - self.meta.origin_y)
            / self.meta.resolution
        )
        return row, col

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert scalar world (x, y) meters to nearest integer (row, col)."""
        row_f, col_f = self.world_to_grid_float(x, y)
        return int(np.round(row_f)), int(np.round(col_f))

    def grid_to_world(
        self, row: np.ndarray, col: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Convert image (row, col) indices to world map (x, y) meters."""
        x = (
            self.meta.origin_x
            + np.asarray(col, dtype=np.float64) * self.meta.resolution
        )
        y = (
            self.meta.origin_y
            + (self.meta.height - 1.0 - np.asarray(row, dtype=np.float64))
            * self.meta.resolution
        )
        return x, y

    def in_bounds(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Return boolean mask whether world coordinates lie inside map."""
        row, col = self.world_to_grid_float(x, y)
        return (
            (row >= 0.0)
            & (row <= self.meta.height - 1.0)
            & (col >= 0.0)
            & (col <= self.meta.width - 1.0)
        )

    def clearance_at(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Bilinearly interpolate wall clearance (meters) at world coordinates.
        Points outside the map or inside occupied cells return 0.0.
        """
        xa = np.asarray(x, dtype=np.float64)
        ya = np.asarray(y, dtype=np.float64)
        scalar = xa.ndim == 0 and ya.ndim == 0
        xa1 = np.atleast_1d(xa)
        ya1 = np.atleast_1d(ya)

        row, col = self.world_to_grid_float(xa1, ya1)
        valid = (
            (row >= 0.0)
            & (row <= self.meta.height - 1.001)
            & (col >= 0.0)
            & (col <= self.meta.width - 1.001)
        )
        out = np.zeros_like(xa1, dtype=np.float64)
        if np.any(valid):
            rv = row[valid]
            cv = col[valid]
            r0 = np.floor(rv).astype(int)
            c0 = np.floor(cv).astype(int)
            dr = rv - r0
            dc = cv - c0

            c00 = self.clearance_m[r0, c0]
            c01 = self.clearance_m[r0, c0 + 1]
            c10 = self.clearance_m[r0 + 1, c0]
            c11 = self.clearance_m[r0 + 1, c0 + 1]
            out[valid] = (
                (1.0 - dr) * (1.0 - dc) * c00
                + (1.0 - dr) * dc * c01
                + dr * (1.0 - dc) * c10
                + dr * dc * c11
            )

        return float(out[0]) if scalar else out

    def clearance_gradient_at(
        self, x: np.ndarray, y: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return world-frame clearance gradient (dc/dx, dc/dy) at (x, y)."""
        row, col = self.world_to_grid_float(x, y)
        ri = np.clip(np.round(row).astype(int), 0, self.meta.height - 1)
        ci = np.clip(np.round(col).astype(int), 0, self.meta.width - 1)
        return self.grad_x[ri, ci], self.grad_y[ri, ci]
