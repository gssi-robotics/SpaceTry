"""Tests for fretish_writer.py"""

import pytest

from fretish_agent.fretish_writer import (
    Requirement,
    extract_signal_names,
    requirements_to_fcs,
    requirements_to_yaml,
    parse_fcs,
)


def _make_req(**kwargs):
    defaults = dict(
        req_id="REQ_TEST",
        scope="globally",
        condition="if obstacle_too_close",
        component="the rover",
        timing="immediately",
        response="satisfy cmd_move_stop",
        description="test requirement",
        kind="guarantee",
        signals_used=["obstacle_too_close", "cmd_move_stop"],
    )
    defaults.update(kwargs)
    return Requirement(**defaults)


def test_fretish_text_basic():
    r = _make_req()
    text = r.fretish_text()
    assert "if obstacle_too_close" in text
    assert "the rover shall" in text
    assert "immediately" in text
    assert "satisfy cmd_move_stop" in text


def test_fretish_text_globally_omitted():
    r = _make_req(scope="globally")
    text = r.fretish_text()
    # "globally" should not appear since it's the default scope
    assert not text.startswith("globally")


def test_fretish_text_custom_scope():
    r = _make_req(scope="after at_waypoint")
    text = r.fretish_text()
    assert text.startswith("after at_waypoint")


def test_to_dict():
    r = _make_req()
    d = r.to_dict()
    assert d["req_id"] == "REQ_TEST"
    assert d["kind"] == "guarantee"
    assert "fretish" in d


def test_requirements_to_fcs():
    reqs = [_make_req(), _make_req(req_id="REQ_TWO", response="satisfy cmd_turn_left")]
    fcs = requirements_to_fcs(reqs)
    assert "REQ_TEST" in fcs
    assert "REQ_TWO" in fcs
    assert "(guarantee)" in fcs


def test_requirements_to_yaml():
    reqs = [_make_req()]
    y = requirements_to_yaml(reqs)
    assert "requirements:" in y
    assert "REQ_TEST" in y


def test_parse_fcs_roundtrip():
    reqs = [_make_req(), _make_req(req_id="REQ_TWO", kind="assumption")]
    fcs = requirements_to_fcs(reqs)
    parsed = parse_fcs(fcs)
    assert len(parsed) == 2
    assert parsed[0].req_id == "REQ_TEST"
    assert parsed[1].kind == "assumption"


def test_extract_signal_names():
    r = _make_req()
    names = extract_signal_names(r)
    assert "obstacle_too_close" in names
    assert "cmd_move_stop" in names
    # keywords should be excluded
    assert "shall" not in names
    assert "the" not in names
    assert "rover" not in names
