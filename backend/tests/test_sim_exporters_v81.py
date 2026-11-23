from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from app.cad.sim_exporters import export_full_dof_urdf, export_topology_mujoco
from app.schemas import ModularArmRequest


def _urdf_joints(path: Path) -> list[ET.Element]:
    root = ET.parse(path).getroot()
    assert root.tag == "robot"
    return root.findall("joint")


def _mjcf_root(path: Path) -> ET.Element:
    root = ET.parse(path).getroot()
    assert root.tag == "mujoco"
    return root


def test_articulated_exact_dof_counts_urdf_and_mujoco(tmp_path: Path):
    for dof in (4, 5, 6, 7):
        req = ModularArmRequest(dof=dof, kinematics_type="articulated")
        u_path = tmp_path / f"art_{dof}.urdf"
        m_path = tmp_path / f"art_{dof}.xml"
        export_full_dof_urdf(req, {"shoulder_torque_nm": 12.0}, u_path)
        export_topology_mujoco(req, {"shoulder_torque_nm": 12.0}, m_path)

        u_joints = _urdf_joints(u_path)
        assert len(u_joints) == dof
        assert all(j.attrib["type"] == "revolute" for j in u_joints)

        m_root = _mjcf_root(m_path)
        m_joints = m_root.findall(".//joint")
        assert len(m_joints) == dof
        assert all(j.attrib["type"] == "hinge" for j in m_joints)


def test_bimanual_14dof_urdf_and_mujoco(tmp_path: Path):
    req = ModularArmRequest(dof=14, kinematics_type="bimanual_dual_arm")
    u_path = tmp_path / "bimanual.urdf"
    m_path = tmp_path / "bimanual.xml"
    export_full_dof_urdf(req, {}, u_path)
    export_topology_mujoco(req, {}, m_path)

    u_joints = _urdf_joints(u_path)
    assert len(u_joints) == 14
    assert all(j.attrib["type"] == "revolute" for j in u_joints)

    m_joints = _mjcf_root(m_path).findall(".//joint")
    assert len(m_joints) == 14


def test_scara_cartesian_delta_stewart_and_continuum_exporters(tmp_path: Path):
    # 1. SCARA: 4 joints (3 revolute + 1 prismatic / slide)
    req_scara = ModularArmRequest(dof=4, kinematics_type="scara")
    export_full_dof_urdf(req_scara, {}, tmp_path / "scara.urdf")
    export_topology_mujoco(req_scara, {}, tmp_path / "scara.xml")
    u_scara = _urdf_joints(tmp_path / "scara.urdf")
    assert len(u_scara) == 4
    assert sum(1 for j in u_scara if j.attrib["type"] == "prismatic") == 1
    m_scara = _mjcf_root(tmp_path / "scara.xml").findall(".//joint")
    assert len(m_scara) == 4
    assert sum(1 for j in m_scara if j.attrib["type"] == "slide") == 1

    # 2. Cartesian Gantry: 3 prismatic/slide + 1 revolute/hinge
    req_cart = ModularArmRequest(dof=4, kinematics_type="cartesian_gantry")
    export_full_dof_urdf(req_cart, {}, tmp_path / "cart.urdf")
    export_topology_mujoco(req_cart, {}, tmp_path / "cart.xml")
    u_cart = _urdf_joints(tmp_path / "cart.urdf")
    assert sum(1 for j in u_cart if j.attrib["type"] == "prismatic") == 3
    m_cart = _mjcf_root(tmp_path / "cart.xml").findall(".//joint")
    assert sum(1 for j in m_cart if j.attrib["type"] == "slide") == 3

    # 3. Delta Parallel: <equality><connect/> loop closure in MuJoCo
    req_delta = ModularArmRequest(dof=4, kinematics_type="delta_parallel")
    export_full_dof_urdf(req_delta, {}, tmp_path / "delta.urdf")
    export_topology_mujoco(req_delta, {}, tmp_path / "delta.xml")
    assert len(_urdf_joints(tmp_path / "delta.urdf")) == 4
    m_delta = _mjcf_root(tmp_path / "delta.xml")
    assert m_delta.find("equality") is not None
    assert len(m_delta.findall("equality/connect")) == 3

    # 4. Stewart Hexapod: 6 slide/prismatic legs + <equality>
    req_stw = ModularArmRequest(dof=6, kinematics_type="stewart_hexapod")
    export_full_dof_urdf(req_stw, {}, tmp_path / "stewart.urdf")
    export_topology_mujoco(req_stw, {}, tmp_path / "stewart.xml")
    u_stw = _urdf_joints(tmp_path / "stewart.urdf")
    assert len(u_stw) == 6
    assert all(j.attrib["type"] == "prismatic" for j in u_stw)
    m_stw = _mjcf_root(tmp_path / "stewart.xml")
    assert sum(1 for j in m_stw.findall(".//joint") if j.attrib.get("type") == "slide") == 6
    assert len(m_stw.findall("equality/connect")) == 6

    # 5. Continuum PCC: 6 segmented bodies + <tendon>
    req_cont = ModularArmRequest(dof=6, kinematics_type="continuum_pcc")
    export_full_dof_urdf(req_cont, {}, tmp_path / "cont.urdf")
    export_topology_mujoco(req_cont, {}, tmp_path / "cont.xml")
    assert len(_urdf_joints(tmp_path / "cont.urdf")) == 6
    m_cont = _mjcf_root(tmp_path / "cont.xml")
    assert m_cont.find("tendon") is not None
    assert len(m_cont.findall("tendon/spatial")) == 3
