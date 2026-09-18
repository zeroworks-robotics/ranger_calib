"""바닥 기준 검증 — 실데이터로, 순환 참조 없이.

1단계(바닥 평면)는 절대 기준이라 합성 데이터가 필요 없다. 로봇이 평지에 있으면
바닥은 base_link 에서 z=0, 기울기 0 이어야 한다. 저장소에 커밋된 실제 점군으로
보정 전후를 재서 그 조건에 가까워지는지 본다.

  python3 tests/test_floor.py

합격 기준: 보정 후 기울기 0.2도 이하, z 오프셋 5mm 이하 (카메라 5대 전부).

이 테스트는 카메라 점군과 '평지' 가정만 쓴다. 라이다를 참조하지 않으므로
합성 라이다를 쓰는 test_synth.py 의 순환 참조 문제가 없다.
"""

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

import auto_calib as ac      # noqa: E402
import make_request          # noqa: E402

PASS_SLOPE_DEG = 0.2
PASS_Z_MM = 5.0


def main():
    req = make_request.build(os.path.join(ROOT, "tests", "request.json"))
    lidar = req["lidar"]
    T_lidar = ac.make_transform(
        ac.Rotation.from_quat(lidar["quat"]).as_matrix(), np.asarray(lidar["xyz"]))
    lidar_base = ac.apply_transform(
        T_lidar, ac.load_cloud(os.path.join(ROOT, lidar["file"])))

    print("%-6s %-26s %-26s %8s %7s %s" % (
        "cam", "보정 전 (slopeX/Y, z)", "보정 후 (slopeX/Y, z)", "판정", "점수", "비고"))
    failures = []
    for cam in req["cams"]:
        r = ac.align_camera(cam, lidar_base)
        f = r["floor"]
        if not f.get("ok"):
            print("%-6s %-26s %-26s %8s %7s %s" % (
                cam["key"], "-", "-", "FAIL", "-", f.get("reason", "1단계 실패")))
            failures.append(cam["key"])
            continue

        b, a = f["before"], f.get("after")
        before = "%6.3f° %6.3f° %6.1fmm" % (b["slope_x_deg"], b["slope_y_deg"], b["z_off_mm"])
        if a is None:
            print("%-6s %-26s %-26s %8s %7s %s" % (
                cam["key"], before, "-", "FAIL", "-", "보정 후 바닥 재검출 실패"))
            failures.append(cam["key"])
            continue

        after = "%6.3f° %6.3f° %6.1fmm" % (a["slope_x_deg"], a["slope_y_deg"], a["z_off_mm"])
        worst_slope = max(abs(a["slope_x_deg"]), abs(a["slope_y_deg"]))
        ok = worst_slope <= PASS_SLOPE_DEG and abs(a["z_off_mm"]) <= PASS_Z_MM
        if not ok:
            failures.append(cam["key"])
        print("%-6s %-26s %-26s %8s %7d %s" % (
            cam["key"], before, after, "PASS" if ok else "FAIL", f["points"],
            "평면두께 %.1fmm, 보정 %.2f° / %.1fmm" % (
                f["plane_rms_mm"], f["tilt_fix_deg"], f["dz_mm"])))

    print("\n결과: %d/%d 통과 (기준 기울기 %.1f° 이하, z %.0fmm 이하)"
          % (len(req["cams"]) - len(failures), len(req["cams"]), PASS_SLOPE_DEG, PASS_Z_MM))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
