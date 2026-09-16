"""
zeus.urdf is generated, so these tests check two things: that it is still what
clean_urdf.py makes from the export and zeus_model.yaml (nobody hand-edited it,
nobody changed the YAML and forgot to regenerate), and that what it makes is
the robot the estimator assumes. Plain numpy, no ROS.
"""

import importlib.util
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

spec = importlib.util.spec_from_file_location("clean_urdf", os.path.join(PKG, "scripts", "clean_urdf.py"))
clean_urdf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(clean_urdf)

ACTUATED = ["waist_pitch", "waist_roll"] + [
    f"{s}_{j}" for s in ("left", "right")
    for j in ("hip_roll", "hip_pitch", "hip_pitch_spring", "knee_pitch", "knee_pitch_spring", "ankle_pitch")]
FRAMES = ["imu_link", "left_toe", "left_heel", "right_toe", "right_heel"]


@pytest.fixture(scope="module")
def tree():
    return clean_urdf.Tree(ET.parse(os.path.join(PKG, "urdf", "zeus.urdf")).getroot(),
                           os.path.join(PKG, "meshes"))


def fk(tree, target, q):
    """Position and rotation of a link in base_link, joints at q (by name)."""
    chain = []
    link = target
    while link != tree.root:
        j = tree.by_child[link]
        chain.append(j)
        link = j.find("parent").get("link")
    T = np.eye(4)
    for j in reversed(chain):
        o = j.find("origin")
        T = T @ clean_urdf.make_T(clean_urdf.vec(o.get("xyz")), clean_urdf.vec(o.get("rpy")))
        if j.get("type") == "revolute":
            a = clean_urdf.vec(j.find("axis").get("xyz"))
            a = a / np.linalg.norm(a)
            th = q.get(j.get("name"), 0.0)
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
            T = T @ np.block([[R, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]])
    return T


def test_urdf_is_regenerated():
    cfg = clean_urdf.yaml.safe_load(open(clean_urdf.CONFIG))
    with open(os.path.join(PKG, cfg["output_urdf"])) as f:
        assert f.read() == clean_urdf.clean(cfg), \
            "urdf/zeus.urdf is stale: run python3 zeus_description/scripts/clean_urdf.py"


def test_joints_and_frames(tree):
    moving = sorted(n for n, j in tree.joints.items() if j.get("type") != "fixed")
    assert moving == sorted(ACTUATED)
    assert all(tree.joints[n].get("type") == "revolute" for n in ACTUATED)
    assert tree.root == "base_link"
    for f in FRAMES:
        assert f in tree.links


def test_axes_follow_the_convention(tree):
    """Pitch about +Y, roll about +X, on both legs, at the zero pose."""
    for name in ACTUATED:
        j = tree.joints[name]
        T = fk(tree, j.find("child").get("link"), {})
        a = T[:3, :3] @ clean_urdf.vec(j.find("axis").get("xyz"))
        want = [1, 0, 0] if name.endswith("roll") else [0, 1, 0]
        assert np.allclose(a, want, atol=1e-3), f"{name} turns about {np.round(a, 3)}"


@pytest.mark.parametrize("side", ["left", "right"])
def test_positive_angles_move_the_foot_the_same_way_on_both_legs(tree, side):
    toe0 = fk(tree, f"{side}_toe", {})[:3, 3]
    for joint, axis, sign in [("hip_pitch", 0, -1), ("hip_pitch_spring", 0, -1), ("hip_roll", 1, +1)]:
        d = fk(tree, f"{side}_toe", {f"{side}_{joint}": 0.1})[:3, 3] - toe0
        assert np.sign(d[axis]) == sign, f"{side}_{joint} +0.1 rad moved the toe {np.round(d, 4)}"


def test_frames_are_where_they_belong(tree):
    p = {f: fk(tree, f, {})[:3, 3] for f in FRAMES}
    for side, y in (("left", 1), ("right", -1)):
        assert p[f"{side}_toe"][0] > p[f"{side}_heel"][0], f"{side} toe is behind its heel"
        assert np.sign(p[f"{side}_toe"][1]) == y and np.sign(p[f"{side}_heel"][1]) == y
        assert p[f"{side}_toe"][2] < -0.4
    assert p["imu_link"][2] > 0.1
