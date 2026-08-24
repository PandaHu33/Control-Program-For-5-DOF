"""Admittance-mode regression tests (E2: Off/HandOnly/HandConfig/Holistic).

bridge.py binds sockets and reads config at import time, so these tests extract
only the pure helpers from its source instead of importing the module.  This
keeps the test process side-effect free while still guarding the exact code
that ships.
"""
from __future__ import annotations

import ast
import math
import pathlib

import pytest

BRIDGE_PATH = pathlib.Path(__file__).resolve().parents[1] / "control_ui" / "bridge.py"


def _extract(names):
    source = BRIDGE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    namespace = {"math": math}
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            getattr(target, "id", "") in names for target in node.targets
        ):
            exec(compile(ast.Module([node], []), "<bridge-extract>", "exec"), namespace)
        if isinstance(node, ast.FunctionDef) and node.name in names:
            exec(compile(ast.Module([node], []), "<bridge-extract>", "exec"), namespace)
    missing = names - set(namespace)
    assert not missing, f"symbols missing from bridge.py: {sorted(missing)}"
    return namespace


@pytest.fixture(scope="module")
def helpers():
    return _extract(
        {"ADMITTANCE_MODE_NAMES", "normalize_admittance_mode", "_finite_six", "_bool_six"}
    )


@pytest.mark.parametrize(
    "value,expected",
    [
        (0, 0),
        (1, 1),
        (2, 2),
        (3, 3),
        ("off", 0),
        ("HANDONLY", 1),
        ("hand_config", 3),
        ("Holistic", 2),
        ("legacy", 1),
        ("bogus", None),
        (True, None),
        (4, None),
        (-1, None),
        (2.7, None),
        (2.0, 2),
    ],
)
def test_normalize_admittance_mode(helpers, value, expected):
    assert helpers["normalize_admittance_mode"](value) == expected


def test_mode_names_cover_four_modes(helpers):
    names = helpers["ADMITTANCE_MODE_NAMES"]
    assert names == {0: "off", 1: "handonly", 2: "holistic", 3: "handconfig"}


@pytest.mark.parametrize(
    "value,default,expected",
    [
        ([0.0] * 6, None, [0.0] * 6),
        (["0.5"] * 6, None, [0.5] * 6),
        ([1.0, 2.0], None, None),
        (None, [None] * 6, [None] * 6),
        ([1.0, 2.0, math.nan, 4.0, 5.0, 6.0], None, None),
    ],
)
def test_finite_six(helpers, value, default, expected):
    assert helpers["_finite_six"](value, default) == expected


def test_bool_six(helpers):
    f = helpers["_bool_six"]
    assert f([True, False, True, False, True, False]) == [True, False, True, False, True, False]
    assert f([True, False]) is None
    assert f(None, [None] * 6) == [None] * 6


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
