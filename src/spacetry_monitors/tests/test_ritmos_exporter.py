"""Tests for ritmos_exporter.py"""

import pytest
import tempfile
from pathlib import Path

import yaml

from fretish_agent.fretish_writer import Requirement
from fretish_agent.signal_registry import SignalRegistry
from fretish_agent.variable_mapper import build_variable_map
from fretish_agent.ritmos_exporter import export_ritmos_artifacts

INVENTORY_PATH = Path(__file__).resolve().parent.parent / "config" / "signal_inventory.yaml"


@pytest.fixture
def registry():
    return SignalRegistry(INVENTORY_PATH)


def _make_reqs():
    return [
        Requirement(
            req_id="REQ_OBSTACLE_STOP",
            scope="globally",
            condition="if obstacle_too_close",
            component="the rover",
            timing="immediately",
            response="satisfy cmd_move_stop",
            description="Stop on obstacle",
            kind="guarantee",
            signals_used=["obstacle_too_close", "cmd_move_stop"],
        ),
        Requirement(
            req_id="ASM_ENV_CLEARANCE",
            scope="globally",
            condition="",
            component="the environment",
            timing="always",
            response="satisfy obstacle_min_range > 0.0",
            description="Environment clearance",
            kind="assumption",
            signals_used=["obstacle_min_range"],
        ),
    ]


def test_export_creates_files(registry):
    reqs = _make_reqs()
    vmap = build_variable_map(reqs, registry)
    with tempfile.TemporaryDirectory() as td:
        result = export_ritmos_artifacts(reqs, vmap, Path(td))
        sig_path = Path(result["signals_path"])
        spec_path = Path(result["spec_path"])
        assert sig_path.exists()
        assert spec_path.exists()


def test_signals_yaml_valid(registry):
    reqs = _make_reqs()
    vmap = build_variable_map(reqs, registry)
    with tempfile.TemporaryDirectory() as td:
        result = export_ritmos_artifacts(reqs, vmap, Path(td))
        data = yaml.safe_load(Path(result["signals_path"]).read_text())
        assert "obstacle_too_close" in data
        assert "cmd_move_stop" in data
        assert data["cmd_move_stop"]["kind"] == "output"


def test_fcs_contains_requirements(registry):
    reqs = _make_reqs()
    vmap = build_variable_map(reqs, registry)
    with tempfile.TemporaryDirectory() as td:
        result = export_ritmos_artifacts(reqs, vmap, Path(td))
        content = Path(result["spec_path"]).read_text()
        assert "REQ_OBSTACLE_STOP" in content
        assert "ASM_ENV_CLEARANCE" in content
        assert "(guarantee)" in content
        assert "(assumption)" in content


def test_no_warnings_for_verified_only(registry):
    reqs = _make_reqs()
    vmap = build_variable_map(reqs, registry)
    with tempfile.TemporaryDirectory() as td:
        result = export_ritmos_artifacts(reqs, vmap, Path(td))
        assert len(result["warnings"]) == 0
