"""Tests for linter.py"""

import pytest
from pathlib import Path

from fretish_agent.fretish_writer import Requirement
from fretish_agent.linter import lint_requirements, LintIssue
from fretish_agent.signal_registry import SignalRegistry

INVENTORY_PATH = Path(__file__).resolve().parent.parent / "config" / "signal_inventory.yaml"


@pytest.fixture
def registry():
    return SignalRegistry(INVENTORY_PATH)


def _make_req(**kwargs):
    defaults = dict(
        req_id="REQ_TEST",
        scope="globally",
        condition="if obstacle_too_close",
        component="the rover",
        timing="immediately",
        response="satisfy cmd_move_stop",
        description="test",
        kind="guarantee",
        signals_used=["obstacle_too_close", "cmd_move_stop"],
    )
    defaults.update(kwargs)
    return Requirement(**defaults)


def test_valid_requirement_no_issues(registry):
    reqs = [_make_req()]
    issues = lint_requirements(reqs, registry)
    errors = [i for i in issues if i.severity == "error"]
    assert len(errors) == 0


def test_duplicate_id_flagged(registry):
    reqs = [_make_req(), _make_req()]
    issues = lint_requirements(reqs, registry)
    assert any("Duplicate" in i.message for i in issues)


def test_empty_response_flagged(registry):
    reqs = [_make_req(response="")]
    issues = lint_requirements(reqs, registry)
    assert any("Empty response" in i.message for i in issues)


def test_invalid_kind_flagged(registry):
    reqs = [_make_req(kind="invalid")]
    issues = lint_requirements(reqs, registry)
    assert any("Invalid kind" in i.message for i in issues)


def test_unknown_signal_flagged(registry):
    reqs = [_make_req(response="satisfy totally_fake_signal_xyz")]
    issues = lint_requirements(reqs, registry)
    assert any("totally_fake_signal_xyz" in i.message for i in issues)


def test_unverified_signal_warned(registry):
    reqs = [_make_req(
        condition="if battery_is_low",
        response="satisfy at_waypoint",
        signals_used=["battery_is_low", "at_waypoint"],
    )]
    issues = lint_requirements(reqs, registry)
    warnings = [i for i in issues if i.severity == "warning"]
    assert any("battery_is_low" in i.message and "NOT VERIFIED" in i.message for i in warnings)


def test_guarantee_only_inputs_warned(registry):
    """A guarantee that only references input signals should get a warning."""
    reqs = [_make_req(
        condition="",
        response="satisfy obstacle_min_range > 0",
        signals_used=["obstacle_min_range"],
    )]
    issues = lint_requirements(reqs, registry)
    warnings = [i for i in issues if i.severity == "warning"]
    assert any("assumption" in i.message.lower() for i in warnings)
