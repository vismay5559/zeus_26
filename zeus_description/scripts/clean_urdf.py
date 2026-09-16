#!/usr/bin/env python3
"""
zeus_raw.urdf (straight from Fusion 360) + zeus_model.yaml  ->  zeus.urdf

The Fusion export is kept exactly as exported and never edited by hand: the
next export would silently wipe any edit. Everything the export gets wrong or
does not know lives in config/zeus_model.yaml instead, and this script applies
it, so a re-export picks the same fixes up automatically.

What it does, in order:

  1. a new massless root, base_link, at the hip centre, with the robot's own
     axes: X forward, Y left, Z up (the export's axes are whatever Fusion's
     world was)
  2. renames links and joints from Fusion's automatic names
  3. makes every actuated and spring joint `revolute` with limits, and flips
     joint axes so every pitch joint turns about +Y and every roll joint about
     +X - on both legs
  4. adds frames Fusion has no reason to export: imu_link and the toe and heel
     contact points, given in the robot's frame at the zero pose
  5. points meshes at package://zeus_description/meshes/

Usage (from anywhere; needs numpy and pyyaml):

    python3 zeus_description/scripts/clean_urdf.py            # write urdf/zeus.urdf
    python3 zeus_description/scripts/clean_urdf.py --check    # exit 1 if zeus.urdf is stale
    python3 zeus_description/scripts/clean_urdf.py --suggest  # IMU/toe/heel positions from the meshes
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
CONFIG = os.path.join(PKG, "config", "zeus_model.yaml")

AXIS = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Y": (0, 1, 0),
        "-Y": (0, -1, 0), "+Z": (0, 0, 1), "-Z": (0, 0, -1)}


# ---------------------------------------------------------------- geometry

def rpy_to_R(rpy) -> np.ndarray:
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx                                   # URDF: fixed-axis roll, pitch, yaw


def R_to_rpy(R: np.ndarray):
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    else:                                                  # gimbal lock: put it all in roll
        r = math.atan2(-R[1, 2], R[1, 1])
        y = 0.0
    return r, p, y


def make_T(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(rpy)
    T[:3, 3] = xyz
    return T


def vec(s, n=3):
    return np.array([float(x) for x in s.split()]) if s else np.zeros(n)


def fmt(v) -> str:
    return " ".join(f"{float(x):.9g}" if abs(x) > 1e-12 else "0" for x in v)


def line_midpoint(p1, a1, p2, a2):
    """Midpoint of the closest points between two axis lines."""
    w = p1 - p2
    b = a1 @ a2
    d = 1.0 - b * b
    if d < 1e-12:
        raise SystemExit("hip roll and pitch axes are parallel - cannot find the hip centre")
    s = (b * (a2 @ w) - (a1 @ w)) / d
    t = ((a2 @ w) - b * (a1 @ w)) / d
    return 0.5 * ((p1 + s * a1) + (p2 + t * a2))


class Tree:
    """A URDF held as XML, with zero-pose transforms in its root frame."""

    def __init__(self, root: ET.Element, mesh_dir: str):
        self.x = root
        self.mesh_dir = mesh_dir
        self.links = {l.get("name"): l for l in root.findall("link")}
        self.joints = {j.get("name"): j for j in root.findall("joint")}
        self.by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        roots = [n for n in self.links if n not in self.by_child]
        if len(roots) != 1:
            raise SystemExit(f"expected one root link, found {roots}")
        self.root = roots[0]
        self._T = {self.root: np.eye(4)}

    def T(self, link: str) -> np.ndarray:
        if link not in self._T:
            j = self.by_child[link]
            o = j.find("origin")
            self._T[link] = self.T(j.find("parent").get("link")) @ make_T(
                vec(o.get("xyz") if o is not None else ""), vec(o.get("rpy") if o is not None else ""))
        return self._T[link]

    def joint_frame(self, name: str):
        j = self.joints[name]
        Tj = self.T(j.find("child").get("link"))
        a = Tj[:3, :3] @ vec(j.find("axis").get("xyz"))
        return Tj[:3, 3], a / np.linalg.norm(a)

    def mesh_world(self, link: str) -> np.ndarray:
        vis = self.links[link].find("visual")
        m = vis.find("geometry/mesh")
        scale = vec(m.get("scale")) if m.get("scale") else np.ones(3)
        raw = open(os.path.join(self.mesh_dir, os.path.basename(m.get("filename"))), "rb").read()
        n = struct.unpack_from("<I", raw, 80)[0]
        if 84 + 50 * n != len(raw):
            raise SystemExit(f"{m.get('filename')}: only binary STL is supported")
        tri = np.frombuffer(raw, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]),
                            count=n, offset=84)
        v = tri["v"].reshape(-1, 3).astype(float) * scale
        vo = vis.find("origin")
        Tv = self.T(link) @ make_T(vec(vo.get("xyz")), vec(vo.get("rpy")))
        return (Tv[:3, :3] @ v.T).T + Tv[:3, 3]


# ---------------------------------------------------------------- the clean-up

def body_frame(tree: Tree, cfg: dict):
    """Rotation and origin of the robot's own frame, in the raw export's frame."""
    bf = cfg["body_frame"]
    R = np.column_stack([AXIS[bf["forward"]], AXIS[bf["left"]], AXIS[bf["up"]]]).astype(float)
    if abs(np.linalg.det(R) - 1.0) > 1e-9:
        raise SystemExit(f"body_frame {bf} is not a right-handed frame")

    raw_name = {v["name"]: k for k, v in cfg["joints"].items()}
    centres = []
    for roll, pitch in bf["origin_between"]:
        p1, a1 = tree.joint_frame(raw_name[roll])
        p2, a2 = tree.joint_frame(raw_name[pitch])
        centres.append(line_midpoint(p1, a1, p2, a2))
    return R, np.mean(centres, axis=0)


def to_urdf_origin(el: ET.Element, T: np.ndarray):
    o = el.find("origin")
    if o is None:
        o = ET.SubElement(el, "origin")
    o.set("xyz", fmt(T[:3, 3]))
    o.set("rpy", fmt(R_to_rpy(T[:3, :3])))


def add_frame(robot: ET.Element, name: str, parent: str, T_parent_frame: np.ndarray, comment: str):
    robot.append(ET.Comment(f" {comment} "))
    ET.SubElement(robot, "link", name=name)
    j = ET.SubElement(robot, "joint", name=f"{parent}_to_{name}", type="fixed")
    ET.SubElement(j, "parent", link=parent)
    ET.SubElement(j, "child", link=name)
    to_urdf_origin(j, T_parent_frame)


def clean(cfg: dict) -> str:
    raw_path = os.path.join(PKG, cfg["raw_urdf"])
    raw = ET.parse(raw_path).getroot()
    tree = Tree(raw, os.path.join(PKG, "meshes"))
    for name in list(tree.links):           # every zero-pose transform, before anything is renamed
        tree.T(name)
    R_wb, c_w = body_frame(tree, cfg)
    T_wb = np.eye(4)
    T_wb[:3, :3] = R_wb
    T_wb[:3, 3] = c_w

    link_names = dict(cfg["links"])
    unknown = set(tree.links) - set(link_names)
    if unknown:
        raise SystemExit(f"links in the export with no name in zeus_model.yaml: {sorted(unknown)}")

    movable = {n for n, j in tree.joints.items() if j.get("type") != "fixed"}
    missing = movable - set(cfg["joints"])
    if missing:
        raise SystemExit(f"movable joints in the export with no entry in zeus_model.yaml: {sorted(missing)}")

    lim = cfg["joint_defaults"]

    # ---- joints: name, type, axis direction, limits (before any renaming,
    # so zero-pose geometry is still looked up by the raw names)
    for raw_name, j in tree.joints.items():
        parent = link_names[j.find("parent").get("link")]
        child = link_names[j.find("child").get("link")]
        if raw_name not in cfg["joints"]:
            j.set("name", f"{parent}_to_{child}")
        else:
            spec = cfg["joints"][raw_name]
            _, a_w = tree.joint_frame(raw_name)
            want_w = R_wb @ np.array(AXIS["+Y" if spec["kind"] == "pitch" else "+X"], float)
            dot = float(a_w @ want_w)
            if abs(dot) < 0.999:
                raise SystemExit(f"{raw_name} ({spec['name']}) is declared {spec['kind']} but its "
                                 f"axis is {np.round(a_w, 3)} in the export")
            ax = j.find("axis")
            if dot < 0:
                ax.set("xyz", fmt(-vec(ax.get("xyz"))))
            j.set("name", spec["name"])
            j.set("type", "revolute")
            for old in j.findall("limit"):
                j.remove(old)
            lo, hi = spec["limit"]
            ET.SubElement(j, "limit", lower=f"{lo:.6g}", upper=f"{hi:.6g}",
                          effort=f"{spec.get('effort', lim['effort']):.6g}",
                          velocity=f"{spec.get('velocity', lim['velocity']):.6g}")
        j.find("parent").set("link", parent)
        j.find("child").set("link", child)

    # ---- links and meshes
    for raw_name, l in tree.links.items():
        l.set("name", link_names[raw_name])
        for m in l.iter("mesh"):
            m.set("filename", cfg["mesh_uri_prefix"] + os.path.basename(m.get("filename")))

    # ---- the new root at the hip centre, in the robot's axes
    root_name = cfg["body_frame"]["root_link"]
    raw.insert(0, ET.Comment(" base_link: the robot's own frame, X forward / Y left / Z up, "
                             "origin at the hip centre (added by clean_urdf.py) "))
    root_link = ET.Element("link", name=root_name)
    raw.insert(1, root_link)
    j = ET.Element("joint", name=f"{root_name}_to_{link_names[tree.root]}", type="fixed")
    ET.SubElement(j, "parent", link=root_name)
    ET.SubElement(j, "child", link=link_names[tree.root])
    to_urdf_origin(j, np.linalg.inv(T_wb))                 # raw root frame is the export's world
    raw.insert(2, j)

    # ---- frames, given in the robot's frame at the zero pose
    inv_name = {v: k for k, v in link_names.items()}

    def frame(name, spec, comment):
        T_w = T_wb @ make_T(spec["xyz"], spec.get("rpy", [0, 0, 0]))
        T_parent = np.linalg.inv(tree.T(inv_name[spec["parent"]])) @ T_w
        add_frame(raw, name, spec["parent"], T_parent, comment)

    frame("imu_link", cfg["imu"], "imu_link: the IMU chip, axes as mounted (added by clean_urdf.py)")
    for name, spec in cfg["contacts"].items():
        frame(name, spec, f"{name}: contact switch point on the sole (added by clean_urdf.py)")

    raw.set("name", cfg["robot_name"])
    ET.indent(raw, space="  ")
    header = ("<?xml version='1.0' encoding='utf-8'?>\n"
              "<!--\n"
              "  GENERATED by zeus_description/scripts/clean_urdf.py from\n"
              f"  {cfg['raw_urdf']} + config/zeus_model.yaml. Do not edit: change the YAML\n"
              "  (or re-export from Fusion) and run the script again.\n"
              "-->\n")
    return header + ET.tostring(raw, encoding="unicode") + "\n"


def suggest(cfg: dict):
    """IMU on top of its parent part, toe/heel at the sole's front and rear edge."""
    raw = ET.parse(os.path.join(PKG, cfg["raw_urdf"])).getroot()
    tree = Tree(raw, os.path.join(PKG, "meshes"))
    R_wb, c_w = body_frame(tree, cfg)
    inv_name = {v: k for k, v in cfg["links"].items()}

    def body(v_w):
        return (R_wb.T @ (v_w - c_w).T).T

    imu_parent = cfg["imu"]["parent"]
    vb = body(tree.mesh_world(inv_name[imu_parent]))
    lo, hi = vb.min(0), vb.max(0)
    print(f"imu on top of {imu_parent}, centre of its top face:")
    print(f"  xyz: [{(lo[0]+hi[0])/2:.4f}, {(lo[1]+hi[1])/2:.4f}, {hi[2]:.4f}]")
    for name, spec in cfg["contacts"].items():
        vb = body(tree.mesh_world(inv_name[spec["parent"]]))
        sole = vb[vb[:, 2] < vb[:, 2].min() + 0.002]
        x = sole[:, 0].max() if name.endswith("toe") else sole[:, 0].min()
        print(f"{name} ({spec['parent']}, sole {'front' if name.endswith('toe') else 'rear'} edge):")
        print(f"  xyz: [{x:.4f}, {(sole[:, 1].min()+sole[:, 1].max())/2:.4f}, {vb[:, 2].min():.4f}]")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="exit 1 if the output is out of date")
    ap.add_argument("--suggest", action="store_true", help="print frame positions measured from the meshes")
    a = ap.parse_args(argv)
    cfg = yaml.safe_load(open(CONFIG))

    if a.suggest:
        suggest(cfg)
        return 0

    text = clean(cfg)
    out = os.path.join(PKG, cfg["output_urdf"])
    if a.check:
        current = open(out).read() if os.path.exists(out) else ""
        if current != text:
            print(f"{cfg['output_urdf']} is out of date - run clean_urdf.py", file=sys.stderr)
            return 1
        print(f"{cfg['output_urdf']} is up to date")
        return 0
    with open(out, "w", newline="\n") as f:
        f.write(text)
    print(f"wrote {cfg['output_urdf']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
