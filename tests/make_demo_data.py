"""눈으로 확인하는 데모 — 바닥을 일부러 기울여 놓고 자동 정렬이 되세우는지 본다.

점군은 건드리지 않는다. 저장소의 실제 스냅샷을 그대로 쓰고, URDF origin 만
일부러 틀어 놓은 파일을 하나 만든다.

  python3 tests/make_demo_data.py

라이다 점군을 카메라 점군에서 합성하면 안 된다. 그렇게 만든 기준은 카메라 자신이므로
ICP 가 카메라를 카메라에 맞추는 순환 참조가 되고, 정확도는 아무것도 증명되지 않는다.
(이 스크립트의 이전 판이 그 실수를 했다.)

대신 1단계가 쓰는 기준은 애초에 순환하지 않는다: 평지에 선 로봇의 바닥은
base_link 에서 z=0, 기울기 0 이어야 한다는 절대 조건이다. 그래서 roll·pitch·z 를
틀어 놓으면 화면에서 바닥이 기울고 가라앉는 것이 눈에 보이고, 자동 정렬이
그것을 되세우는 것도 눈에 보인다.

x, y, rot_z 는 라이다의 수직 면이 기준이므로, 그 검증은 로봇에서 6/6 스냅샷을
받은 뒤에 해야 한다. 커밋된 data/ 는 일부 센서만 갱신된 상태다.

생성 파일: urdf/demo_misaligned.urdf
"""

import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

import auto_calib as ac      # noqa: E402
import make_request          # noqa: E402

# 일부러 틀어 놓는 양. 1단계가 잡는 축(rot_x, rot_y, z)에 집중해 화면에서 보이게 한다.
# 거부 기준(기울기 5도, z 100mm) 안쪽이라 정상 보정 대상이다.
PERTURB = {
    "front": {"rpy_deg": [1.8, 0.0, 0.0], "dz": 0.030},
    "fdown": {"rpy_deg": [0.0, 1.5, 0.0], "dz": -0.025},
    "right": {"rpy_deg": [1.2, 1.2, 0.0], "dz": 0.020},
    "rear":  {"rpy_deg": [-1.5, 0.0, 0.0], "dz": -0.035},
    "left":  {"rpy_deg": [0.0, -2.0, 0.0], "dz": 0.028},
}

DEMO_URDF = os.path.join(ROOT, "urdf", "demo_misaligned.urdf")
BASE_URDF = os.path.join(ROOT, "urdf", "ranger_new.urdf")


def perturbed_origin(cam):
    """정답 origin 에 PERTURB 를 얹은 (xyz, rpy)."""
    p = PERTURB[cam["key"]]
    R = ac.rpy_to_matrix(*cam["rpy"])
    dR = ac.Rotation.from_rotvec(np.radians(p["rpy_deg"])).as_matrix()
    xyz = np.asarray(cam["xyz"]) + np.array([0.0, 0.0, p["dz"]])
    return xyz, ac.matrix_to_rpy(dR @ R)


def main():
    req = make_request.build(os.path.join(ROOT, "tests", "request.json"))
    with open(BASE_URDF, encoding="utf-8") as f:
        xml = f.read()

    print("틀어 놓는 양 (자동 정렬 1단계가 이만큼 되돌려야 한다):")
    for cam in req["cams"]:
        xyz, rpy = perturbed_origin(cam)
        link = make_request.CHILD_LINK[cam["key"]]
        pattern = ('(<child link="%s"/>\\s*<origin xyz=")[^"]*(" rpy=")[^"]*("\\s*/>)' % link)
        xyz_str = "%.4f %.4f %.4f" % tuple(xyz)
        rpy_str = "%.6f %.6f %.6f" % tuple(rpy)
        xml, n = re.subn(pattern,
                         lambda m: m.group(1) + xyz_str + m.group(2) + rpy_str + m.group(3),
                         xml, count=1)
        if n != 1:
            raise SystemExit("%s origin 치환 실패 — %s 구조를 확인" % (link, BASE_URDF))
        p = PERTURB[cam["key"]]
        print("  %-6s 기울기 %4.2f°  z %+6.1fmm" % (
            cam["key"], np.linalg.norm(p["rpy_deg"]), p["dz"] * 1000))

    with open(DEMO_URDF, "w", encoding="utf-8") as f:
        f.write(xml)
    print("\n생성: %s" % os.path.relpath(DEMO_URDF, ROOT))
    print("""
확인 절차
  1) 서버 실행 (같은 PowerShell 창에서)
       $env:CONA_URDF_PATH = "urdf\\demo_misaligned.urdf"
       $env:RANGER_CALIB_PORT = "8099"
       & "$HOME\\venvs\\ranger_calib\\Scripts\\python.exe" calib_server.py
  2) http://127.0.0.1:8099 접속 후 'Top' 이 아니라 'Front' 또는 'Side' 시점으로 본다
     (바닥 기울기는 측면에서 봐야 보인다)
  3) '최신 URDF 불러오기' -> 바닥이 그라운드에서 기울고 가라앉는다
  4) '자동 정렬' -> 바닥이 그라운드에 평평하게 얹힌다. 보고에 기울기 전후 값이 찍힌다
  5) 원상복구
       Remove-Item urdf\\demo_misaligned.urdf
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
