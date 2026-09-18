"""2단계(라이다 ICP) 미끄러짐 검출기 테스트.

2단계는 x, y, rot_z 만 푼다. 이 세 축을 구속하는 것은 수직 면이고, 그것도
서로 다른 방향의 면이 두 개 이상 있어야 한다. 벽이 하나뿐이면 그 벽에 평행한
방향으로 미끄러진다 — ICP 는 수렴하고 fitness·rmse 까지 좋게 나오므로,
관측 가능성 판정이 없으면 엉뚱한 값이 그대로 적용된다.

  python3 tests/test_slip.py

합격 기준
  1) 벽 하나만 있는 장면: 벽에 평행한 y 가 제외되고 그 축 보정이 0.
     x(벽 법선 방향)와 rot_z(벽 방향)는 벽 하나로도 구속되므로 살아 있어야 한다.
  2) 직교하는 벽 두 개: x, y, rot_z 전부 구속돼 틀어 넣은 30mm 를 되찾는다

바닥 면도 같이 넣는다. 2단계가 바닥을 제대로 걸러내는지(수직 면만 쓰는지)
같이 확인하기 위해서다.
"""

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import auto_calib as ac      # noqa: E402

NOISE = 0.002                # 2mm 측정 잡음
PERTURB_X = 0.030            # x 로 틀어 넣는 양 (m)
EX, EY, EZ = np.eye(3)
rng = np.random.default_rng(0)


def plane(axis_a, axis_b, origin, span=1.2, step=0.02):
    grid = np.arange(-span, span, step)
    u, v = np.meshgrid(grid, grid)
    pts = origin + np.outer(u.ravel(), axis_a) + np.outer(v.ravel(), axis_b)
    return pts + rng.normal(0, NOISE, pts.shape)


def floor():
    return plane(EX, EY, np.array([0.9, 0.0, 0.0]))


def one_wall():
    """x 를 구속하는 벽 하나 + 바닥. y 와 rot_z 는 구속되지 않는다."""
    return np.vstack([floor(), plane(EY, EZ, np.array([1.9, 0.0, 0.4]), span=0.9)])


def two_walls():
    """직교하는 벽 두 개 + 바닥. x, y, rot_z 전부 구속된다."""
    return np.vstack([
        floor(),
        plane(EY, EZ, np.array([1.9, 0.0, 0.4]), span=0.9),
        plane(EX, EZ, np.array([0.9, 1.4, 0.4]), span=0.9),
    ])


def run(scene_fn, label):
    """카메라와 라이다가 같은 장면을 각각 독립 잡음으로 본 상태를 만든다.

    같은 점군을 두 번 쓰면 잡음 패턴이 특징점이 되어 구속되지 않은 축까지 맞춰진다.
    """
    cam_cloud = scene_fn()
    lidar_cloud = scene_fn()

    # 카메라 pose: 정답은 단위변환, 초기값은 x 로 PERTURB_X 만큼 틀어 놓음.
    # 바닥이 이미 z=0 수평이라 1단계는 아무것도 바꾸지 않고, 2단계만 시험된다.
    cam = {"key": "test", "file": "synthetic.json",
           "xyz": [PERTURB_X, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0],
           "mount": {"xyz": [0.0, 0.0, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]}}

    original_loader = ac.load_cloud
    ac.load_cloud = lambda _path: cam_cloud
    try:
        r = ac.align_camera(cam, lidar_cloud)
    finally:
        ac.load_cloud = original_loader

    l = r["lidar"]
    dropped = l.get("dropped_dofs", [])
    print("%-16s 2단계=%-5s 제외축=%-14s x %7s  y %7s  rot_z %8s"
          % (label, l.get("ok"), ",".join(dropped) or "-",
             "%.2fmm" % l["fix_x_mm"] if "fix_x_mm" in l else "-",
             "%.2fmm" % l["fix_y_mm"] if "fix_y_mm" in l else "-",
             "%.3fdeg" % l["fix_yaw_deg"] if "fix_yaw_deg" in l else "-"))
    print("%16s sigma: rot_z %s deg, x %s mm, y %s mm   reason=%s"
          % ("", round(l.get("sigma", {}).get("rot_z_deg", -1), 3),
             round(l.get("sigma", {}).get("x_m", -1) * 1000, 2),
             round(l.get("sigma", {}).get("y_m", -1) * 1000, 2), l.get("reason")))
    return r


def main():
    failures = []

    r = run(one_wall, "벽 하나+바닥")
    l = r["lidar"]
    dropped = set(l.get("dropped_dofs", []))
    recovered = -l.get("fix_x_mm", 0.0)
    if "y" not in dropped:
        print("  FAIL: y 가 제외되지 않음 — 벽에 평행한 방향으로 미끄러진 값이 적용된다")
        failures.append("one_wall/dropped")
    elif abs(l.get("fix_y_mm", 99)) > 0.5:
        print("  FAIL: y 보정이 0 이 아님 (%.2fmm)" % l["fix_y_mm"])
        failures.append("one_wall/applied")
    elif dropped - {"y"}:
        print("  FAIL: 벽 하나로도 구속되는 축이 제외됨 (%s)" % ",".join(sorted(dropped - {"y"})))
        failures.append("one_wall/overdrop")
    elif abs(recovered - PERTURB_X * 1000) > 2.0:
        print("  FAIL: x 를 %.2fmm 만 복원 (기대 %.0fmm)" % (recovered, PERTURB_X * 1000))
        failures.append("one_wall/recover")
    else:
        print("  PASS: y 만 제외(보정 0), x %.2fmm 복원" % recovered)

    r = run(two_walls, "직교벽 둘+바닥")
    l = r["lidar"]
    recovered = -l.get("fix_x_mm", 0.0)      # 보정은 초기값을 정답으로 되돌리는 방향
    if not l.get("ok"):
        print("  FAIL: 2단계가 거부됨 — %s" % l.get("reason"))
        failures.append("two_walls/rejected")
    elif l.get("dropped_dofs"):
        print("  FAIL: 세 축 전부 구속돼야 하는데 제외축 있음 (%s)" % ",".join(l["dropped_dofs"]))
        failures.append("two_walls/dropped")
    elif abs(recovered - PERTURB_X * 1000) > 2.0:
        print("  FAIL: %.2fmm 복원 (기대 %.0fmm)" % (recovered, PERTURB_X * 1000))
        failures.append("two_walls/recover")
    else:
        print("  PASS: %.2fmm 복원 (기대 %.0fmm)" % (recovered, PERTURB_X * 1000))

    print("\n결과: %s" % ("전부 통과" if not failures else "실패 " + ", ".join(failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
