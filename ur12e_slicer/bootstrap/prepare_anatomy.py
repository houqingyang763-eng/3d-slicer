#!/usr/bin/env python3
"""Prepare the BrainWeb subject 04 static-head model, using its official label volume."""
import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import numpy as np
from scipy.ndimage import binary_fill_holes, distance_transform_edt
import vtk
from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy


PROJECT = Path(__file__).resolve().parents[1]
URL = "https://brainweb.bic.mni.mcgill.ca/cgi/brainweb1"
DATA_INFO = "https://brainweb.bic.mni.mcgill.ca/cgi/brainweb1?alias=subject04_crisp&download=1"
LABELS = {"brain": (2, 3), "skull": (7, 11)}
SHAPE = (362, 434, 362)  # Official raw file order is z, y, x.
SOURCE_ORIGIN_RAS_MM = np.array([-90.25, -126.25, -72.25])
SOURCE_SPACING_MM = 0.5
BLOCK = 4
PLANNING_PAD_MM = 16.0
ASSET_DIR = PROJECT / ".local/head_anatomy"
ARCHIVE = PROJECT / "downloads/brainweb_subject04_crisp.raw.gz"

sys.path.insert(0, str(PROJECT / "ros2_ws/src/ur12e_slicer_bridge"))
from ur12e_slicer_bridge.anatomy_geometry import sha256


def load_labels(path):
    with gzip.open(path, "rb") as stream:
        raw = stream.read(math.prod(SHAPE) + 1)
    if len(raw) != math.prod(SHAPE):
        raise ValueError("BrainWeb raw volume dimensions did not match the source description")
    digest = hashlib.sha256(raw).hexdigest()
    expected = json.loads((PROJECT / "assets/brainweb_source.json").read_text(encoding="utf-8"))["raw_sha256"]
    if digest != expected:
        raise ValueError(f"BrainWeb raw SHA-256 mismatch ({digest})")
    labels = np.frombuffer(raw, dtype=np.uint8).reshape(SHAPE).transpose(2, 1, 0)
    return labels, digest


def download(destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    post = urlencode({"do_download_alias": "subject04_crisp", "format_value": "raw_byte",
                      "zip_value": "gnuzip", "download_for_real": "1"}).encode("ascii")
    with tempfile.NamedTemporaryFile(dir=destination.parent, suffix=".raw.gz.part", delete=False) as stream:
        temporary = Path(stream.name)
        print(f"Downloading BrainWeb to {temporary}", flush=True)
        with urlopen(Request(URL, data=post), timeout=45) as response:
            shutil.copyfileobj(response, stream)
    load_labels(temporary)
    os.replace(temporary, destination)


def blocked_mask(labels, ids):
    mask = np.isin(labels, ids)
    mask = np.pad(mask, [(0, -length % BLOCK) for length in mask.shape])
    x, y, z = mask.shape
    return mask.reshape(x // BLOCK, BLOCK, y // BLOCK, BLOCK, z // BLOCK, BLOCK).any(axis=(1, 3, 5))


def contour(mask, origin, spacing, max_faces=None):
    image = vtk.vtkImageData()
    image.SetDimensions(*mask.shape)
    image.SetOrigin(*origin)
    image.SetSpacing(spacing, spacing, spacing)
    image.GetPointData().SetScalars(numpy_to_vtk(mask.astype(np.uint8).ravel(order="F"), deep=True))
    surface = vtk.vtkFlyingEdges3D()
    surface.SetInputData(image)
    surface.SetValue(0, 0.5)
    surface.ComputeNormalsOff()
    surface.Update()
    mesh = surface.GetOutput()
    if max_faces and mesh.GetNumberOfPolys() > max_faces:
        decimate = vtk.vtkQuadricDecimation()
        decimate.SetInputData(mesh)
        decimate.SetTargetReduction(1.0 - max_faces / mesh.GetNumberOfPolys())
        decimate.VolumePreservationOn()
        decimate.Update()
        mesh = decimate.GetOutput()
    points = vtk_to_numpy(mesh.GetPoints().GetData()).astype(np.float32, copy=True)
    faces = vtk_to_numpy(mesh.GetPolys().GetConnectivityArray()).reshape(-1, 3).astype(np.int32, copy=True)
    if len(points) < 3 or not len(faces):
        raise RuntimeError("Anatomy contour was empty")
    return points, faces


def prepare(labels, digest):
    observed = set(np.unique(labels))
    if not observed <= set(range(12)) or not all(set(ids) <= observed for ids in LABELS.values()):
        raise ValueError(f"Unexpected BrainWeb anatomical labels: {sorted(observed)}")
    masks = {name: blocked_mask(labels, ids) for name, ids in LABELS.items()}
    # Treat internal voids enclosed by brain tissue as protected too.
    masks["brain"] = binary_fill_holes(masks["brain"])
    occupied = np.argwhere(masks["brain"] | masks["skull"])
    first, last = occupied.min(axis=0), occupied.max(axis=0)
    crop = tuple(slice(lo, hi + 1) for lo, hi in zip(first, last))
    spacing = SOURCE_SPACING_MM * BLOCK
    pad_cells = math.ceil(PLANNING_PAD_MM / spacing) + 3
    origin = -(last - first) * spacing / 2 - pad_cells * spacing
    arrays = {"grid_origin_mm": origin, "grid_spacing_mm": spacing}
    metadata = {}
    for name, mask in masks.items():
        mask = np.pad(mask[crop], pad_cells)
        distance = distance_transform_edt(~mask, sampling=spacing)
        arrays[name + "_distance_mm"] = distance.astype(np.float32)
        points, faces = contour(mask, origin, spacing)
        arrays[name + "_points_mm"], arrays[name + "_faces"] = points, faces
        points_pad, faces_pad = contour(distance <= PLANNING_PAD_MM, origin, spacing, max_faces=5500)
        arrays[name + "_planning_points_mm"], arrays[name + "_planning_faces"] = points_pad, faces_pad
        metadata[name] = {"source_labels": list(LABELS[name]), "occupied_cells": int(mask.sum()),
                          "nominal_faces": len(faces), "planning_faces": len(faces_pad),
                          "nominal_bounds_mm": [points.min(axis=0).tolist(), points.max(axis=0).tolist()]}
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=ASSET_DIR, suffix=".npz", delete=False) as stream:
        np.savez_compressed(stream, **arrays)
        temp_geometry = Path(stream.name)
    manifest = {
        "schema_version": 1, "subject": "BrainWeb subject 04",
        "source": DATA_INFO, "source_raw_sha256": digest, "geometry_sha256": sha256(temp_geometry),
        "grid_spacing_mm": spacing, "planning_pad_mm": PLANNING_PAD_MM, "structures": metadata,
        "source_origin_ras_mm": SOURCE_ORIGIN_RAS_MM.tolist(),
        "source_ras_at_head_origin_mm": (SOURCE_ORIGIN_RAS_MM + (first + last) * spacing / 2
                                          + (BLOCK - 1) * SOURCE_SPACING_MM / 2).tolist(),
    }
    os.replace(temp_geometry, ASSET_DIR / "geometry.npz")
    temporary = ASSET_DIR / "manifest.json.tmp"
    temporary.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, ASSET_DIR / "manifest.json")
    print(json.dumps(manifest, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offline", action="store_true", help="Prepare from the already downloaded label volume")
    args = parser.parse_args()
    if not ARCHIVE.is_file() and not args.offline:
        download(ARCHIVE)
    labels, digest = load_labels(ARCHIVE)
    prepare(labels, digest)


if __name__ == "__main__":
    main()
