"""눈으로 확인하는 데모 데이터 생성 — 틀어진 상태에서 자동 정렬이 맞추는지 본다.

문제: 저장소에 커밋된 data/ 는 센서별 촬영 시각이 어긋난 스냅샷이라 정답이 없다.
그래서 "틀어진 상태 -> 버튼 -> 맞춰짐" 을 눈으로 확인할 수 없다.

이 스크립트가 정답이 있는 데이터를 만든다.

  1) 카메라 5대의 점군을 index.html 내장 origin(= 정답)으로 base_link 에 펼친다
  2) 그걸 합쳐 라이다 점군으로 되돌려 저장한다 -> 정답 origin 에서 완벽히 겹치는 상태
  3) 정답에서 일부러 틀어 놓은 URDF 를 따로 쓴다 -> 화면에서 눈에 보이게 어긋난 상태

자동 정렬이 제대로 동작하면 2)와 3)의 차이를 되찾아 원래 자리로 돌려놓는다.

  python3 tests/make_demo_data.py

덮어쓰는 파일
  data/rslidar_points.json       (원본은 git 에 있으므로 git checkout 으로 복구)
  urdf/demo_misaligned.urdf      (새로 생성)
"""

import json
import os
import shutil
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

import auto_calib as ac      # noqa: E402
import make_request          # noqa: E402

# 일부러 틀어 놓는 양. 화면에서 눈에 보일 만큼 크고, 거부 기준(50mm / 5deg) 안에 들어야 한다.
# 카메라마다 다른 방향으로 틀어 어느 카메라가 어떻게 움직이는지 구분되게 한다.
PERTURB = {
    "front": {"xyz": [0.030, 0.000, 0.000], "rpy_deg": [0.0, 0.0, 1.2]},
    "fdown": {"xyz": [0.000, 0.000, 0.025], "rpy_deg": [1.0, 0.0, 0.0]},
    "right": {"xyz": [0.000, 0.028, 0.000], "rpy_deg": [0.0, 1.2, 0.0]},
    "rear":  {"xyz": [-0.025, 0.000, 0.015], "rpy_deg": [0.0, 0.0, -1.0]},
    "left":  {"xyz": [0.000, -0.030, 0.010], "rpy_deg": [0.8, 0.8, 0.0]},
}

LIDAR_VOXEL = 0.03       # 라이다 점 간격 (m). 실제 링 스캔보다 고르지만 정합 조건은 같다
MAX_POINTS = 20000       # capture_once.py 와 같은 상한
DEMO_URDF = os.path.join(ROOT, "urdf", "demo_misaligned.urdf")
BASE_URDF = os.path.join(ROOT, "urdf", "ranger_new.urdf")
LIDAR_JSON = os.path.join(ROOT, "data", "rslidar_points.json")


def voxel_downsample(pts, voxel):
    keys = np.floor(pts / voxel).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inv, pts)
    return sums / counts[:, None]


def write_cloud(path, pts):
    flat = [round(float(v), 4) for v in np.asarray(pts).ravel()]
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(flat, f, separators=(",", ":"))
    os.replace(tmp, path)


def perturbed_origin(cam):
    """정답 origin 에 PERTURB 를 얹은 (xyz, rpy) 를 돌려준다."""
    p = PERTURB[cam["key"]]
    R_true = ac.rpy_to_matrix(*cam["rpy"])
    dR = ac.Rotation.from_rotvec(np.radians(p["rpy_deg"])).as_matrix()
    xyz = np.asarray(cam["xyz"]) + np.asarray(p["xyz"])
    return xyz, ac.matrix_to_rpy(dR @ R_true)


def main():
    req = make_request.build(os.path.join(ROOT, "tests", "request.json"))

    # 1) 정답 origin 으로 카메라 점군을 base_link 에 펼쳐 합친다
    merged = []
    for cam in req["cams"]:
        T_cam = ac.make_transform(ac.rpy_to_matrix(*cam["rpy"]), np.asarray(cam["xyz"]))
        T_mount = ac.make_transform(
            ac.Rotation.from_quat(cam["mount"]["quat"]).as_matrix(),
            np.asarray(cam["mount"]["xyz"]))
        pts = ac.load_cloud(os.path.join(ROOT, cam["file"]))
        merged.append(ac.apply_transform(T_cam @ T_mount, pts))
        print("  %-6s %6d점" % (cam["key"], len(pts)))
    merged = np.vstack(merged)

    # 2) 라이다 프레임으로 되돌려 저장 (UI·auto_calib 이 라이다 pose 로 다시 펼친다)
    thinned = voxel_downsample(merged, LIDAR_VOXEL)
    if len(thinned) > MAX_POINTS:
        idx = np.random.default_rng(0).choice(len(thinned), MAX_POINTS, replace=False)
        thinned = thinned[idx]
    lidar = req["lidar"]
    R_l = ac.Rotation.from_quat(lidar["quat"]).as_matrix()
    t_l = np.asarray(lidar["xyz"])
    in_lidar_frame = (thinned - t_l) @ R_l

    backup = LIDAR_JSON + ".orig"
    if not os.path.exists(backup):
        shutil.copyfile(LIDAR_JSON, backup)
    write_cloud(LIDAR_JSON, in_lidar_frame)
    print("\n라이다 점군 생성: %s (%d점, 원본 백업 %s)"
          % (os.path.relpath(LIDAR_JSON, ROOT), len(in_lidar_frame),
             os.path.relpath(backup, ROOT)))

    # 3) 일부러 틀어 놓은 URDF 를 쓴다
    with open(BASE_URDF, encoding="utf-8") as f:
        xml = f.read()
    print("\n틀어 놓은 양 (자동 정렬이 이만큼 되돌려야 한다):")
    for cam in req["cams"]:
        xyz, rpy = perturbed_origin(cam)
        link = make_request.CHILD_LINK[cam["key"]]
        pattern = '(<child link="%s"/>\\s*<origin xyz=")[^"]*(" rpy=")[^"]*("\\s*/>)' % link
        import re
        xyz_str = "%.4f %.4f %.4f" % tuple(xyz)
        rpy_str = "%.6f %.6f %.6f" % tuple(rpy)
        xml, n = re.subn(pattern, lambda m: m.group(1) + xyz_str + m.group(2) + rpy_str + m.group(3),
                         xml, count=1)
        if n != 1:
            raise SystemExit("%s origin 치환 실패 — %s 구조를 확인" % (link, BASE_URDF))
        p = PERTURB[cam["key"]]
        print("  %-6s 이동 %5.1fmm  회전 %4.2fdeg" % (
            cam["key"], np.linalg.norm(p["xyz"]) * 1000, np.linalg.norm(p["rpy_deg"])))
    with open(DEMO_URDF, "w", encoding="utf-8") as f:
        f.write(xml)
    print("\n틀어진 URDF 생성: %s" % os.path.relpath(DEMO_URDF, ROOT))

    print("""
확인 절차
  1) 서버 실행 (같은 PowerShell 창에서)
       $env:CONA_URDF_PATH = "urdf\\demo_misaligned.urdf"
       $env:RANGER_CALIB_PORT = "8099"
       & "$HOME\\venvs\\ranger_calib\\Scripts\\python.exe" calib_server.py
  2) http://127.0.0.1:8099 접속
  3) '최신 URDF 불러오기' -> 점군이 회색 라이다에서 눈에 보이게 어긋난다
  4) '자동 정렬' -> 어긋남이 사라지고, 보고에 위 표와 비슷한 보정량이 찍힌다
  5) 원상복구
       git checkout -- data/rslidar_points.json
       Remove-Item data\\rslidar_points.json.orig, urdf\\demo_misaligned.urdf
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
