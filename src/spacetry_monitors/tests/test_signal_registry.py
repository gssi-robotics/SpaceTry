"""Tests for signal_registry.py"""

import pytest
from pathlib import Path

from fretish_agent.signal_registry import SignalRegistry

INVENTORY_PATH = Path(__file__).resolve().parent.parent / "config" / "signal_inventory.yaml"


@pytest.fixture
def registry():
    return SignalRegistry(INVENTORY_PATH)


def test_loads_signals(registry):
    assert len(registry.all_signals()) > 0


def test_verified_signals_exist(registry):
    verified = registry.verified_signals()
    assert len(verified) > 0
    names = [s.name for s in verified]
    assert "obstacle_min_range" in names
    assert "rover_position_x" in names
    assert "cmd_move_stop" in names


def test_unverified_signals_flagged(registry):
    unverified = registry.unverified_signals()
    names = [s.name for s in unverified]
    assert "battery_percentage" in names
    assert "sample_detected" in names


def test_signal_kinds(registry):
    inputs = registry.inputs()
    outputs = registry.outputs()
    internals = registry.internals()
    assert any(s.name == "obstacle_min_range" for s in inputs)
    assert any(s.name == "cmd_move_forward" for s in outputs)
    assert any(s.name == "obstacle_too_close" for s in internals)


def test_get_existing(registry):
    sig = registry.get("rover_position_x")
    assert sig is not None
    assert sig.kind == "input"
    assert sig.verified is True
    assert sig.ros_topic == "/model/curiosity_mars_rover/odometry"


def test_get_missing(registry):
    assert registry.get("nonexistent_signal_xyz") is None
    assert registry.exists("nonexistent_signal_xyz") is False


def test_is_verified(registry):
    assert registry.is_verified("obstacle_min_range") is True
    assert registry.is_verified("battery_percentage") is False
    assert registry.is_verified("nonexistent") is False
