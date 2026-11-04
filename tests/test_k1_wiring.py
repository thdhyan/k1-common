"""Assert scripts/k1_wiring.py against the real URDF / MJCF / SDK header / USD.

Run: uv run --no-project --with pytest --with usd-core pytest tests/test_k1_wiring.py
(the USD test is skipped if pxr is not importable).
"""
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
from k1_common import k1_wiring as w  # noqa: E402

K1 = ROOT / "src/k1_description/assets/robots/K1"
URDFS = ["K1_22dof.urdf", "K1_22dof-ZED.urdf"]
MJCF = K1 / "K1_22dof.xml"
SDK_HDR = ROOT / "sdk/booster_robotics_sdk/include/booster/robot/b1/b1_api_const.hpp"


def _urdf(name):
    return ET.parse(K1 / name).getroot()


def test_lists_consistent():
    assert len(w.JOINTS) == len(set(w.JOINTS)) == 22
    assert sorted(w.BFS_ORDER) == sorted(w.JOINTS)
    assert sorted(w.LEG_BFS_ORDER) == sorted(w.LEG_DEPLOY_ORDER)
    assert w.LEG_SDK_INDEX == list(range(10, 22))
    assert [w.JOINTS[i] for i in w.SDK_TO_BFS] == w.BFS_ORDER
    assert [w.BFS_ORDER[i] for i in w.BFS_TO_SDK] == w.JOINTS
    v = list(range(12))
    assert w.reorder(w.reorder(v, w.LEG_DEPLOY_ORDER, w.LEG_BFS_ORDER),
                     w.LEG_BFS_ORDER, w.LEG_DEPLOY_ORDER) == v
    assert w.LEG_BFS_ORDER[:2] == ["Left_Hip_Pitch", "Right_Hip_Pitch"]
    # HANDOFF.md:644 play log leg ids for [LHipP, RHipP, LHipR, RHipR, ...]
    assert sorted(w.BFS_ORDER.index(n) for n in w.LEG_DEPLOY_ORDER) == [3, 4, 8, 9, 12, 13, 16, 17, 18, 19, 20, 21]


@pytest.mark.parametrize("fn", URDFS)
def test_urdf_order_limits_axes(fn):
    joints = [j for j in _urdf(fn).findall("joint") if j.get("type") == "revolute"]
    assert [j.get("name") for j in joints] == w.JOINTS
    for j in joints:
        n = j.get("name")
        lim = j.find("limit")
        assert (float(lim.get("lower")), float(lim.get("upper"))) == pytest.approx(w.LIMITS[n]), n
        assert float(lim.get("effort")) == w.URDF_EFFORT[n], n
        assert tuple(float(x) for x in j.find("axis").get("xyz").split()) == w.AXES[n], n


def test_mjcf_joints_actuators_armature():
    root = ET.parse(MJCF).getroot()
    joints = [j for j in root.iter("joint") if j.get("name") and j.get("name") != "world_joint"]
    assert [j.get("name") for j in joints] == w.JOINTS
    acts = list(root.find("actuator"))
    assert [a.tag for a in acts] == ["motor"] * 22
    assert [a.get("joint") for a in acts] == w.JOINTS
    for j in joints:
        n = j.get("name")
        lo, hi = (float(x) for x in j.get("range").split())
        assert (lo, hi) == pytest.approx(w.LIMITS[n]), n
        assert tuple(float(x) for x in j.get("axis").split()) == w.AXES[n], n
        if j.get("armature"):
            exp = w.SIM_ACTUATOR_PARAMS[w._sim_key(n)][0]
            if "Ankle" not in n and n not in w.HEAD:  # ankle: raw 0.0565 vs ratio split; head MJCF 0.002 vs sim 0.001
                assert float(j.get("armature")) == pytest.approx(exp, rel=1e-3), n


def test_sdk_header_enum():
    txt = SDK_HDR.read_text()
    body = txt[txt.index("JointIndexK1"):]
    body = body[body.index("{") + 1: body.index("}")]
    names = [m.group(1) for m in re.finditer(r"\b(k\w+)\s*(?:=\s*\d+)?\s*,", body)]
    names = [n for n in names if n != "kJointCntK1"]
    assert names == w.SDK_ENUM_NAMES
    assert re.search(r"kJointCntK1\s*=\s*22", txt)


def test_gain_formula_and_known_values():
    kp, kd = w.derive_pd(0.0478125, 4.0, 1.5)
    assert kp == pytest.approx(30.2, abs=0.05) and kd == pytest.approx(3.60, abs=0.01)
    kp, kd = w.SIM_TRAIN_GAINS["Left_Knee_Pitch"]
    assert kp == pytest.approx(60.4, abs=0.05) and kd == pytest.approx(4.81, abs=0.01)
    assert w.SIM_TRAIN_GAINS["AAHead_yaw"][0] == pytest.approx(3.95, abs=0.01)
    assert w.ACTUATOR_DELAY_SECONDS == pytest.approx((0.010, 0.040))
    assert math.isclose(w.CONTROL_HZ * w.PHYSICS_DT * 4, 1.0)


def test_leg_target_clip_scale():
    t = w.leg_target([2.0] * 12, scale=0.25)
    assert t[0] == pytest.approx(0.25)


def test_usd_bfs_leg_ids():
    pxr = pytest.importorskip("pxr")
    usd = next(iter(K1.parent.parent.rglob("K1_flat.usd")), None)
    if usd is None:
        pytest.skip("K1_flat.usd not found")
    from pxr import Usd, UsdPhysics
    stage = Usd.Stage.Open(str(usd))
    names = {p.GetName() for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
    assert names == set(w.JOINTS)
