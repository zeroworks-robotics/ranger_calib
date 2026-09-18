"""미끄러짐 검출기 테스트 — 자동 정렬의 핵심 안전장치를 검증한다.

바닥 평면 하나만 겹치는 장면은 z, rot_x, rot_y 만 구속하고 x, y, rot_z 는
구속하지 못한다. ICP 는 그래도 수렴하고 fitness·rmse 까지 좋게 나오므로,
관측 가능성 판정이 없으면 엉뚱한 값을 그대로 적용한다.

  python3 tests/test_slip.py

합격 기준
  1) 바닥만 있는 장면: x, y, rot_z 가 제외되고 그 축 보정이 0 이어야 한다
  2) 직교하는 벽 2개를 더한 장면: 6 DOF 전부 구속돼 틀어 넣은 양을 되찾아야 한다
"""

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import auto_calib as ac      # noqa: E402

NOISE = 0.002                # 2mm 측정 잡음
PERTURB_X = 0.030            # 틀어 넣는 양 (m)
EX, EY, EZ = np.eye(3)
rng = np.random.default_rng(0)


def plane(axis_a, axis_b, origin, span=1.2, step=0.02):
    grid = np.arange(-span, span, step)
    u, v = np.meshgrid(grid, grid)
    pts = origin + np.outer(u.ravel(), axis_a) + np.outer(v.ravel(), axis_b)
    return pts + rng.normal(0, NOISE, pts.shape)


def floor_scene():
    return plane(EX, EY, np.array([0.5, 0.0, -0.5]))


def corner_scene():
    return np.vstack([
        floor_scene(),
        plane(EY, EZ, np.array([1.7, 0.0, 0.3]), span=0.8),    # x 를 구속하는 벽
        plane(EX, EZ, np.array([0.5, 1.2, 0.3]), span=0.8),    # y, rot_z 를 구속하는 벽
    ])


def run(scene_fn, label):
    """카메라와 라이다가 같은 면을 각각 독립 잡음으로 본 상태를 만든다.

    같은 점군을 그대로 두 번 쓰면 잡음 패턴 자체가 특징점이 되어
    구속되지 않은 방향까지 맞춰져 버린다 (테스트가 통과해 버린다).
    """
    cam_cloud = scene_fn()
    lidar_cloud = scene_fn()

    # 카메라 pose: 정답은 단위변환, 초기값은 x 로 PERTURB_X 만큼 틀어 놓음
    # file 은 로더를 바꿔 끼우므로 실제로 읽히지 않지만, 경로 조립에 쓰이므로 문자열이어야 한다
    cam = {"key": "test", "file": "synthetic.json",
           "xyz": [PERTURB_X, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0],
           "mount": {"xyz": [0.0, 0.0, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]}}

    # align_camera 는 파일에서 점군을 읽으므로 로더만 잠시 바꿔 끼운다
    original_loader = ac.load_cloud
    ac.load_cloud = lambda _path: cam_cloud
    try:
        r = ac.align_camera(cam, lidar_cloud)
    finally:
        ac.load_cloud = original_loader

    dropped = r.get("dropped_dofs", [])
    applied = r.get("delta_trans_mm", [0.0, 0.0, 0.0])
    print("%-14s ok=%-5s 제외축=%-16s 적용된 이동(mm)=[%s]"
          % (label, r["ok"], ",".join(dropped) or "-",
             ", ".join("%.2f" % v for v in applied)))
    print("%14s sigma_trans_mm=%s sigma_rot_deg=%s reason=%s"
          % ("", [round(v * 1000, 2) for v in r.get("sigma", {}).get("trans_m", [])],
             [round(v, 3) for v in r.get("sigma", {}).get("rot_deg", [])],
             r.get("reason")))
    return r


def main():
    failures = []

    r = run(floor_scene, "바닥만")
    need = {"x", "y", "rot_z"}
    missing = need - set(r.get("dropped_dofs", []))
    if missing:
        print("  FAIL: %s 가 제외되지 않음 — 미끄러진 값이 적용된다" % ", ".join(sorted(missing)))
        failures.append("floor/dropped")
    elif abs(r.get("delta_trans_mm", [9e9])[0]) > 0.5:
        print("  FAIL: x 보정이 0 이 아님 (%.2fmm)" % r["delta_trans_mm"][0])
        failures.append("floor/applied")
    else:
        print("  PASS: x, y, rot_z 제외 + 보정 0")

    r = run(corner_scene, "바닥+직교벽2")
    recovered = -r.get("delta_trans_mm", [0.0])[0]      # 보정은 초기값을 정답으로 되돌리는 방향
    if r.get("dropped_dofs"):
        print("  FAIL: 6 DOF 전부 구속돼야 하는데 제외축 있음 (%s)" % ",".join(r["dropped_dofs"]))
        failures.append("corner/dropped")
    elif abs(recovered - PERTURB_X * 1000) > 1.0:
        print("  FAIL: %.2fmm 복원 (기대 %.0fmm)" % (recovered, PERTURB_X * 1000))
        failures.append("corner/recover")
    else:
        print("  PASS: %.2fmm 복원 (기대 %.0fmm)" % (recovered, PERTURB_X * 1000))

    print("\n결과: %s" % ("전부 통과" if not failures else "실패 " + ", ".join(failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
