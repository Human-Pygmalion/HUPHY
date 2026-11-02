"""학습 레포의 URDF 를 RViz 용 `huphy.urdf` 로 바꿈. 원본이 바뀌었을 때만 다시 돌림.

    python3 ros/huphy_ros/urdf/make_urdf.py <원본.urdf> <원본.xml (MJCF)> <메시 폴더>

원본 (Human-Pygmalion/HuphyChan-dev, main):

    pygmalion_locomotion/assets/pygmalion_v2/
        LegOnly_prototype-tempmass-motormeasured-armfix_v30_proxyfix_loop.urdf
        LegOnly_prototype-tempmass-motormeasured-armfix_v30_proxyfix_loop.xml
    tools/robot_model/fusion_snapshots/v30_inspection/meshes_armfix/

`_loop` 이 붙은 것을 씀 -- 발목 크랭크(`crank_A/B`)와 막대가 들어 있어서 우리 모터
`ankle_a/b` 가 그대로 보임. 원본의 `meshes_v30_armfix` 는 다른 사람 컴퓨터를 가리키는
링크라 위 폴더에서 가져옴 (크랭크·막대 메시는 `meshes/` 와 같은 파일).


## 바꾸는 것

    관절 이름   R_hip_pitch_joint -> right_leg__hip_pitch     (`/huphy/joint_states` 와 글자까지 같게)
                R_crank_A_joint   -> right_leg__ankle_a
                R_ankle_pitch_joint -> right_leg__ankle_pitch
                R_rod_A_u1 같은 막대 관절은 그대로 (linkage_node 가 값을 채움)
    점 링크     막대 끝(R_rod_A_end)과 발의 볼조인트(R_ball_A)를 fixed 링크로 더함 (아래)
    보이는 모양 MJCF 가 그리는 것을 따름 (아래 "모양")
    메시 경로   meshes_v30_armfix/pelvis.stl -> package://huphy_ros/meshes/pelvis.stl
    충돌 모양   <collision> 을 지움. RViz 는 안 씀 (hull 메시를 안 들고 와도 됨)
    base_link   <inertial> 을 지움. KDL 이 뿌리 링크의 관성을 못 다뤄 경고를 냄. 보기용이라 질량은 안 씀


## 막대

URDF 는 막대 끝이 발에 붙는 고리를 못 적음 (원본 URDF 에 주석으로만 있음). MJCF 는
`<equality><connect>` 로 막대 끝 site 와 발의 볼 site 를 묶음. 그 두 site 를 빈 링크로
옮겨 두면 `linkage_node` 가 URDF 만 읽고 "막대 끝이 볼에 닿는" 막대 각도를 풀 수 있음.
site 위치는 MJCF 값을 씀 -- 원본 URDF 주석의 막대 끝 y 는 부호가 반대로 적혀 있음.



## 모양

원본 URDF 의 정강이·발 메시(`shin.stl`, `foot.stl`)에는 크랭크·막대가 **0 자세로 붙어 있음.**
그대로 쓰면 움직이는 막대 옆에 고정된 막대가 하나 더 보임. MJCF 는 그것을 뺀
`*_noloop.stl` 을 씀. 그래서 링크마다 MJCF 의 visual 메시로 바꾸고, MJCF 가 따로 그리는
모터 원기둥(검정)도 옮겨 넣음.

robot_state_publisher 는 **URDF 에 없는 이름을 경고 없이 버림.** 이름이 하나라도 다르면
그 관절만 조용히 멈춰 있음 -- 그래서 손으로 고치지 않고 이 스크립트로 만듦.
"""

from __future__ import annotations

import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT_URDF = HERE / "huphy.urdf"
OUT_MESHES = HERE.parent / "meshes"

SIDES = {"R": "right_leg", "L": "left_leg"}
JOINTS = {
    "hip_pitch": "hip_pitch",
    "hip_roll": "hip_roll",
    "hip_yaw": "hip_yaw",
    "knee": "knee",
    "crank_A": "ankle_a",
    "crank_B": "ankle_b",
    "ankle_pitch": "ankle_pitch",
    "ankle_roll": "ankle_roll",
}
_NAME = re.compile(r"^([RL])_(.+)_joint$")
ROS_SEP = "__"  # huphy_ros/convert.py 와 같아야 함

SITE = re.compile(r"^[RL]_(rod_[AB]_end|ball_[AB])$")
"""고리를 닫는 두 점. linkage_node 가 같은 이름으로 찾음."""


def ros_name(name: str) -> str:
    m = _NAME.match(name)
    if not m or m.group(2) not in JOINTS:
        return name
    return f"{SIDES[m.group(1)]}{ROS_SEP}{JOINTS[m.group(2)]}"


def loop_sites(mjcf: Path):
    """MJCF 에서 (site 이름, 붙은 body 이름, pos) 를 뽑음."""
    out = []
    for body in ET.parse(mjcf).getroot().iter("body"):
        for site in body.findall("site"):
            if SITE.match(site.get("name", "")):
                out.append((site.get("name"), body.get("name"), site.get("pos")))
    return out


def _quat_to_rpy(w: float, x: float, y: float, z: float) -> str:
    """MJCF quat (w x y z) -> URDF rpy (고정축 X-Y-Z)."""
    import math
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return f"{roll:.6g} {pitch:.6g} {yaw:.6g}"


def mjcf_visuals(mjcf: Path):
    """MJCF 에서 body 별로 그리는 것: (메시 파일 목록, 원기둥 목록)."""
    root = ET.parse(mjcf).getroot()
    files = {m.get("name"): m.get("file") for m in root.iter("mesh")}
    meshes, cylinders = {}, {}
    for body in root.iter("body"):
        for g in body.findall("geom"):
            if g.get("class") != "visual":
                continue
            if g.get("mesh"):
                meshes.setdefault(body.get("name"), []).append(files[g.get("mesh")])
            elif g.get("type") == "cylinder":
                cylinders.setdefault(body.get("name"), []).append(g)
    return meshes, cylinders


def main(src: Path, mjcf: Path, mesh_dir: Path) -> None:
    tree = ET.parse(src)
    root = tree.getroot()

    renamed = []
    for joint in root.iter("joint"):
        old = joint.get("name")
        new = ros_name(old)
        joint.set("name", new)
        if new != old:
            renamed.append(new)
    for mimic in root.iter("mimic"):
        mimic.set("joint", ros_name(mimic.get("joint")))

    links = {link.get("name") for link in root.iter("link")}
    for link in root.iter("link"):
        for col in link.findall("collision"):
            link.remove(col)
        if link.get("name") == "base_link":
            for e in link.findall("inertial"):
                link.remove(e)

    # 모양: MJCF 가 그리는 메시로 바꾸고 모터 원기둥을 더함
    vis_mesh, vis_cyl = mjcf_visuals(mjcf)
    swapped, added = [], 0
    for link in root.iter("link"):
        name = link.get("name")
        files = vis_mesh.get(name, [])
        mesh = link.find("visual/geometry/mesh")
        if mesh is not None and len(files) == 1 and Path(mesh.get("filename")).name != files[0]:
            swapped.append(f"{name}: {Path(mesh.get('filename')).name} -> {files[0]}")
            mesh.set("filename", files[0])
        for g in vis_cyl.get(name, []):
            r, half = (float(v) for v in g.get("size").split()[:2])
            q = [float(v) for v in g.get("quat", "1 0 0 0").split()]
            vis = ET.SubElement(link, "visual", name=g.get("name", ""))
            ET.SubElement(vis, "origin", xyz=g.get("pos", "0 0 0"), rpy=_quat_to_rpy(*q))
            geo = ET.SubElement(vis, "geometry")
            ET.SubElement(geo, "cylinder", radius=f"{r:g}", length=f"{2 * half:g}")
            mat = ET.SubElement(vis, "material", name="motor_black")
            ET.SubElement(mat, "color", rgba="0.1 0.1 0.1 1")
            added += 1

    sites = loop_sites(mjcf)
    if len(sites) != 8:
        sys.exit(f"MJCF 에서 고리 site 를 {len(sites)}개 찾음 (8개여야 함): {[s[0] for s in sites]}")
    for name, body, pos in sites:
        if body not in links:
            sys.exit(f"site {name} 가 붙은 {body} 가 URDF 에 없음 -- URDF 와 MJCF 가 같은 모델인지 볼 것")
        ET.SubElement(root, "link", name=name)
        j = ET.SubElement(root, "joint", name=f"{name}_site", type="fixed")
        ET.SubElement(j, "parent", link=body)
        ET.SubElement(j, "child", link=name)
        ET.SubElement(j, "origin", xyz=pos, rpy="0 0 0")

    OUT_MESHES.mkdir(exist_ok=True)
    used = set()
    for mesh in root.iter("mesh"):
        name = Path(mesh.get("filename")).name
        mesh.set("filename", f"package://huphy_ros/meshes/{name}")
        used.add(name)
    for name in sorted(used):
        shutil.copyfile(mesh_dir / name, OUT_MESHES / name)

    ET.indent(tree)
    tree.write(OUT_URDF, encoding="utf-8", xml_declaration=True)
    print(f"{OUT_URDF}  관절 이름 {len(renamed)}개 바꿈, 고리 점 {len(sites)}개, "
          f"모터 원기둥 {added}개, 메시 {len(used)}개 -> {OUT_MESHES}")
    for n in swapped:
        print("   메시 바꿈", n)
    for n in sorted(renamed):
        print("  ", n)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
