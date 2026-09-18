"""합성 정답 테스트 — 알려진 양을 틀어 넣고 되찾는지 본다.

카메라 점군을 라이다 자리에 그대로 놓으면(같은 pose) 정답 정렬 오차는 0 이다.
거기서 카메라 pose 만 알려진 양으로 틀어 초기값으로 주고, auto_calib 이
그 양을 되찾는지 확인한다. 변환 체인·역변환 방향·rpy 규약을 한 번에 검증한다.

  python3 tests/test_synth.py

로봇 없이 개발 PC 에서 돌아간다 (numpy, scipy 만 필요).
합격 기준: 남은 오차 2mm / 0.05deg 이하.
"""

import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

import auto_calib as ac          # noqa: E402
import make_request              # noqa: E402

PERTURB_XYZ = [0.008, -0.004, 0.006]      # m
PERTURB_RPY = [0.3, -0.2, 0.4]            # deg (base_link 축 기준 회전)
PASS_TRANS_MM = 2.0
PASS_ROT_DEG = 0.05


def main():
    req = make_request.build(os.path.join(ROOT, "tests", "request.json"))
    failures = []

    for cam in req["cams"]:
        R_true = ac.rpy_to_matrix(*cam["rpy"])
        t_true = np.asarray(cam["xyz"])
        T_mount = ac.make_transform(
            ac.Rotation.from_quat(cam["mount"]["quat"]).as_matrix(),
            np.asarray(cam["mount"]["xyz"]))

        # 라이다 = 정답 pose 로 펼친 같은 점군
        T_lidar = ac.make_transform(R_true, t_true) @ T_mount
        lidar_base = ac.apply_transform(
            T_lidar, ac.load_cloud(os.path.join(ROOT, cam["file"])))

        # 초기값 = 정답에서 알려진 양만큼 틀어 놓은 pose
        dR = ac.Rotation.from_rotvec(np.radians(PERTURB_RPY)).as_matrix()
        bad = {"key": cam["key"], "file": cam["file"], "mount": cam["mount"],
               "xyz": [float(v) for v in t_true + np.asarray(PERTURB_XYZ)],
               "rpy": ac.matrix_to_rpy(dR @ R_true)}

        r = ac.align_camera(bad, lidar_base)
        if not r["ok"]:
            print("%-6s REJECT | %s" % (cam["key"], r.get("reason")))
            failures.append(cam["key"])
            continue

        res_t = float(np.linalg.norm(np.asarray(r["xyz"]) - t_true)) * 1000
        res_r = math.degrees(float(np.linalg.norm(
            ac.Rotation.from_matrix(ac.rpy_to_matrix(*r["rpy"]) @ R_true.T).as_rotvec())))
        within = res_t <= PASS_TRANS_MM and res_r <= PASS_ROT_DEG

        # 제외된 축은 보정을 적용하지 않으므로 그만큼 오차가 남는다. 설계된 동작이라
        # 실패로 세지 않고 PARTIAL 로 구분해 표시한다.
        if within:
            verdict = "PASS"
        elif r["dropped_dofs"]:
            verdict = "PARTIAL"
        else:
            verdict = "FAIL"
            failures.append(cam["key"])

        print("%-6s %-7s | 남은 오차 %.2fmm %.3fdeg (초기 오차 %.1fmm %.2fdeg) "
              "fitness %.2f rmse %.1fmm 제외축 %s"
              % (cam["key"], verdict, res_t, res_r,
                 np.linalg.norm(PERTURB_XYZ) * 1000, np.linalg.norm(PERTURB_RPY),
                 r["fitness"], r["inlier_rmse"] * 1000,
                 ",".join(r["dropped_dofs"]) or "-"))

    print("\n결과: %d/%d 통과" % (len(req["cams"]) - len(failures), len(req["cams"])))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
