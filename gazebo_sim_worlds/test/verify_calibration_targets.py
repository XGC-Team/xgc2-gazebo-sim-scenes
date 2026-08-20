#!/usr/bin/env python3
"""Verify the retained checkerboard and field-matched AprilGrid worlds."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LEGACY_WORLD = (
    PACKAGE_ROOT
    / "worlds/camera_calibration_intrinsic/camera_calibration_intrinsic.world"
)
APRILGRID_WORLD = (
    PACKAGE_ROOT
    / "worlds/camera_calibration_intrinsic_aprilgrid_6x6"
    / "camera_calibration_intrinsic_aprilgrid_6x6.world"
)
MODEL_ROOT = PACKAGE_ROOT / "models/aprilgrid_6x6_tag36h11_88mm"


def _included_models(path: Path) -> dict[str, ET.Element]:
    world = ET.parse(str(path)).getroot().find("world")
    assert world is not None
    return {
        include.findtext("name", default=""): include
        for include in world.findall("include")
        if include.find("name") is not None
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _png_size(path: Path) -> tuple[int, int]:
    with path.open("rb") as stream:
        assert stream.read(8) == b"\x89PNG\r\n\x1a\n"
        length = struct.unpack(">I", stream.read(4))[0]
        assert stream.read(4) == b"IHDR" and length == 13
        return struct.unpack(">II", stream.read(8))


def test_calibration_worlds_keep_independent_targets() -> None:
    legacy = _included_models(LEGACY_WORLD)
    aprilgrid = _included_models(APRILGRID_WORLD)
    assert legacy["intrinsic_checkerboard"].findtext("uri") == "model://checkerboard_8x6"
    assert "intrinsic_aprilgrid" not in legacy
    assert aprilgrid["intrinsic_aprilgrid"].findtext("uri") == (
        "model://aprilgrid_6x6_tag36h11_88mm"
    )
    assert "intrinsic_checkerboard" not in aprilgrid
    assert aprilgrid["intrinsic_aprilgrid"].findtext("pose") == "2 0 2.2 0 0 0"


def test_field_aprilgrid_geometry_and_official_export_are_exact() -> None:
    target = yaml.safe_load((MODEL_ROOT / "target.yaml").read_text(encoding="utf-8"))
    assert target == {
        "target_type": "aprilgrid",
        "tagCols": 6,
        "tagRows": 6,
        "tagSize": 0.088,
        "tagSpacing": 0.3,
        "tagFamily": "tag36h11",
        "tagStartId": 0,
        "tagEndId": 35,
    }
    assert 6 * target["tagSize"] + 5 * target["tagSize"] * target["tagSpacing"] == 0.66

    manifest = json.loads((MODEL_ROOT / "source_manifest.json").read_text(encoding="utf-8"))
    pdf = MODEL_ROOT / manifest["officialPdf"]["path"]
    texture = MODEL_ROOT / manifest["gazeboTexture"]["path"]
    assert _sha256(pdf) == manifest["officialPdf"]["sha256"]
    assert _sha256(texture) == manifest["gazeboTexture"]["sha256"]
    assert _png_size(texture) == (3564, 3564)
    assert pdf.read_bytes().startswith(b"%PDF-")

    model = ET.parse(str(MODEL_ROOT / "model.sdf")).getroot().find("model")
    assert model is not None
    target_visual = model.find("./link/visual[@name='official_aprilgrid_target']")
    assert target_visual is not None
    assert target_visual.findtext("./geometry/box/size") == "0.002 0.7128 0.7128"
    assert target_visual.findtext("pose") == "-0.0111 0 0 0 0 0"
