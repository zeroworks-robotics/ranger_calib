"""RGBD 카메라 외부 파라미터 자동 정렬 (point-to-plane ICP).

웹 UI 의 '자동 정렬' 버튼이 calib_server.py 를 통해 이 스크립트를 호출한다.
단독 실행도 된다:  python3 auto_calib.py request.json

입력(JSON): 화면의 현재 상태를 그대로 받는다. 상수를 여기에 복사해 두면
index.html 과 값이 어긋나므로, 마운트 체인과 초기 origin 은 전부 호출자가 준다.

  {
    "lidar": {"file": "data/rslidar_points.json", "xyz": [..3], "quat": [x,y,z,w]},
    "cams": [
      {"key": "front",
       "file": "data/camera_frontRGBD_depth_points.json",
       "xyz": [..3], "rpy": [..3],                  # base_link -> camera_*RGBD_link (현재값)
       "mount": {"xyz": [..3], "quat": [x,y,z,w]}}  # camera_*RGBD_link -> optical frame
    ]
  }

출력(JSON): 카메라별 제안 origin + 판정 근거. 파일 저장은 하지 않는다.

정합 방향에 대한 메모
--------------------
라이다를 '타깃'으로 두는 편이 직관적이지만, 라이다는 링 스캔이라 점이 희박해
법선 추정이 나쁘다. 그래서 반대로 잡는다.

  타깃 = 카메라 점군 (밀집, 법선 양호) -> 여기서 평면 법선을 뽑는다
  소스 = 라이다 점군 (기준 진리값)
  라이다를 카메라 면에 맞추는 변환 T_d 를 구한 뒤, 카메라에 줄 보정은 그 역변환이다.

  T_camera_new = inv(T_d) @ T_camera_current
"""

import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = os.path.dirname(os.path.abspath(__file__))

# ---- 정합 파라미터 ----
# 초기값이 실측 URDF 라 이미 정답 근처다. 따라서 포착 범위를 넓게 잡을 이유가 없고,
# 넓게 잡으면 오히려 벽 위의 다른 지점으로 대응이 붙어 엉뚱한 최소점으로 걸어간다
# (실측 데이터에서 785mm, 892mm 짜리 오수렴을 실제로 만들었다).
SCALES = [0.05, 0.02]            # 멀티스케일 voxel 크기 (m)
CORR_MULT = 2.0                  # 대응점 최대 거리 = voxel * 이 값
MAX_CORR_CAP = 0.12              # 대응점 최대 거리 상한 (m). 이 이상은 다른 평면으로 넘어간다
STEP_TRANS_CAP = 0.02            # 반복 1회 평행이동 상한 (m)
STEP_ROT_CAP = 0.0175            # 반복 1회 회전 상한 (rad, 약 1도)
DIVERGE_TRANS = 0.10             # 누적 보정이 이만큼 넘으면 발산으로 보고 중단 (m)
MAX_ITER = 40                    # 스케일별 최대 반복
CONV_ROT = 1e-6                  # 수렴 판정 (rad)
CONV_TRANS = 1e-6                # 수렴 판정 (m)
BBOX_MARGIN = 0.30               # 시야 겹침 판정용 바운딩박스 여유 (m)
NORMAL_K = 20                    # 법선 추정 이웃 개수
MAX_CURVATURE = 0.08             # 곡률이 이보다 크면 평면이 아니므로 법선을 버린다
TRIM_SIGMA = 3.0                 # 잔차 MAD 기준 이상치 제거 배수
MIN_PAIRS = 80                   # 대응점이 이보다 적으면 판정 불가. 시야 안 라이다 점이
                                 # 1,000~2,000개 수준이라 이보다 높이면 정상 장면도 거부된다

# ---- 수락/거부 기준 ----
MIN_FITNESS = 0.30               # 겹침 비율 하한
MAX_RMSE = 0.02                  # 정합 품질 상한 (m)
MAX_TRANS_FIX = 0.05             # 평행이동 보정 상한 (m). 초기값이 실측이라 이 이상은 오수렴
MAX_ROT_FIX = 5.0                # 회전 보정 상한 (deg)
# 관측 불가 DOF 판정: 이보다 불확실하면 보정을 적용하지 않고 원래값을 유지한다
SIGMA_TRANS_LIMIT = 0.010        # m
SIGMA_ROT_LIMIT = 0.5            # deg
# 상대 기준도 같이 본다. 잡음 때문에 법선이 몇 도씩 흔들리면 구속되지 않은 방향에도
# 정보가 새어 들어와 절대 sigma 가 작게 나온다 (바닥만 보이는 장면에서 실측 2.9mm).
# 같은 그룹(평행이동/회전) 안에서 가장 잘 구속된 축보다 이 배수 이상 나쁘면 버린다.
SIGMA_RATIO_LIMIT = 15.0
# 불확실도 계산에 쓰는 측정 잡음 하한 (m). 라이다·RGBD 거리 잡음 수준.
# 달성된 rmse 만으로 스케일하면 안 된다: 구속되지 않은 방향도 잔차는 0 에 가깝게
# 맞춰지므로 sigma 가 0 으로 나오고 미끄러짐 검출기가 울리지 않는다.
SIGMA_MEAS_FLOOR = 0.005

# 보고용 DOF 이름. 앞 3개는 base_link 의 x, y, z 축을 중심으로 한 회전이며
# URDF 의 roll/pitch/yaw(내재 ZYX 오일러각)와 정확히 같은 양은 아니다.
# 보정량이 작을 때는 사실상 일치하지만, 이름을 rpy 로 쓰면 오해를 부른다.
DOF_NAMES = ["rot_x", "rot_y", "rot_z", "x", "y", "z"]


# ---------- 회전 표현 (URDF 규약: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)) ----------

def rpy_to_matrix(roll, pitch, yaw):
    return Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()


def matrix_to_rpy(R):
    yaw = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    roll = np.arctan2(R[2, 1], R[2, 2])
    return [float(roll), float(pitch), float(yaw)]


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def apply_transform(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]


def clamp_norm(v, limit):
    """벡터 크기를 limit 으로 제한한다 (방향 유지)."""
    n = float(np.linalg.norm(v))
    return v if n <= limit or n == 0.0 else v * (limit / n)


# ---------- 점군 유틸 ----------

def load_cloud(path):
    """평탄 배열 [x,y,z,x,y,z,...] 을 Nx3 으로 읽는다."""
    with open(path, "r", encoding="utf-8") as f:
        flat = np.asarray(json.load(f), dtype=np.float64)
    if flat.size % 3 != 0:
        raise ValueError("%s: 좌표 개수가 3의 배수가 아님 (%d)" % (os.path.basename(path), flat.size))
    return flat.reshape(-1, 3)


def voxel_downsample(pts, voxel):
    """voxel 격자별 무게중심. 밀도가 다른 두 센서를 같은 밀도로 맞추는 용도."""
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inv, pts)
    return sums / counts[:, None]


def estimate_normals(pts, k=NORMAL_K):
    """이웃 PCA 로 법선과 곡률을 구한다. 곡률이 큰 점(모서리·잡음)은 법선을 버린다."""
    if len(pts) < k:
        return None, None
    tree = cKDTree(pts)
    _, idx = tree.query(pts, k=k, workers=-1)
    nbrs = pts[idx]                                  # (N, k, 3)
    centered = nbrs - nbrs.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", centered, centered) / k
    evals, evecs = np.linalg.eigh(cov)               # 고유값 오름차순
    normals = evecs[:, :, 0]                         # 최소 고유값 방향 = 법선
    total = evals.sum(axis=1)
    curvature = np.where(total > 1e-12, evals[:, 0] / np.maximum(total, 1e-12), 1.0)
    return normals, curvature


def in_bbox(pts, ref, margin=BBOX_MARGIN):
    """ref 점군의 바운딩박스 + 여유 안의 점만 남긴다 = 시야 겹침 영역 선별."""
    if len(ref) == 0:
        return np.zeros(len(pts), dtype=bool)
    lo = ref.min(axis=0) - margin
    hi = ref.max(axis=0) + margin
    return np.all((pts >= lo) & (pts <= hi), axis=1)


# ---------- ICP ----------

def icp_point_to_plane(src, tgt, tgt_normals, center, max_corr):
    """소스를 타깃 평면에 맞추는 강체 변환을 구한다.

    반환: (T, stats). stats["info"] 는 정보행렬(6x6, [회전3, 평행이동3] 순).
    회전은 center 를 중심으로 매개화한다. 원점 기준으로 두면 회전과 평행이동이
    강하게 결합해 조건수가 나빠지고 DOF 별 불확실도 해석도 못 하게 된다.
    """
    T = np.eye(4)
    tree = cKDTree(tgt)
    stats = {"info": np.zeros((6, 6)), "rmse": float("inf"), "pairs": 0}

    for _ in range(MAX_ITER):
        moved = apply_transform(T, src)
        dist, idx = tree.query(moved, k=1, workers=-1)
        mask = dist < max_corr
        if mask.sum() < MIN_PAIRS:
            break

        p = moved[mask]
        q = tgt[idx[mask]]
        n = tgt_normals[idx[mask]]
        resid = np.einsum("ij,ij->i", p - q, n)

        # MAD 기반 이상치 제거. 평균/표준편차는 이상치 자체에 끌려가므로 쓰지 않는다.
        mad = np.median(np.abs(resid - np.median(resid)))
        if mad > 1e-9:
            keep = np.abs(resid) < TRIM_SIGMA * 1.4826 * mad
            if keep.sum() >= MIN_PAIRS:
                p, q, n, resid = p[keep], q[keep], n[keep], resid[keep]

        # p' = p + w x (p - center) + t 를 잔차에 대입해 선형화
        A = np.hstack([np.cross(p - center, n), n])
        ATA = A.T @ A
        ATb = A.T @ (-resid)
        try:
            x = np.linalg.solve(ATA + 1e-12 * np.eye(6), ATb)
        except np.linalg.LinAlgError:
            break

        w, t = x[:3], x[3:]
        # 한 걸음의 크기를 묶는다. 선형화는 작은 각도에서만 유효하고,
        # 큰 걸음을 허용하면 한 번에 다른 평면으로 건너뛴다.
        w = clamp_norm(w, STEP_ROT_CAP)
        t = clamp_norm(t, STEP_TRANS_CAP)

        R_d = Rotation.from_rotvec(w).as_matrix()
        T_step = make_transform(R_d, center - R_d @ center + t)   # center 기준 회전 후 평행이동
        T = T_step @ T

        stats = {"info": ATA, "rmse": float(np.sqrt(np.mean(resid ** 2))), "pairs": int(len(p)),
                 "diverged": False}

        if np.linalg.norm(T[:3, 3] + T[:3, :3] @ center - center) > DIVERGE_TRANS:
            stats["diverged"] = True
            break
        if np.linalg.norm(w) < CONV_ROT and np.linalg.norm(t) < CONV_TRANS:
            break

    return T, stats


def residual_rmse(src, tgt, tgt_normals, max_corr):
    """변환 없이 그대로일 때의 점-평면 잔차 RMSE. 보정 전후 비교용."""
    tree = cKDTree(tgt)
    dist, idx = tree.query(src, k=1, workers=-1)
    mask = dist < max_corr
    if mask.sum() < MIN_PAIRS:
        return None
    resid = np.einsum("ij,ij->i", src[mask] - tgt[idx[mask]], tgt_normals[idx[mask]])
    return float(np.sqrt(np.mean(resid ** 2)))


def dof_sigma(info, rmse):
    """정보행렬에서 DOF 별 1-sigma 불확실도를 뽑는다.

    공분산 = sigma_meas^2 * inv(info). 대각 제곱근이 각 DOF 의 표준편차다.
    info 의 어떤 방향 고유값이 0 에 가까우면(평면 위 미끄러짐) 그 DOF 의 sigma 가 폭발한다.
    이 값이 자동 정렬의 신뢰 여부를 판별하는 유일한 근거다.

    sigma_meas 는 rmse 와 측정 잡음 하한 중 큰 값을 쓴다. rmse 만 쓰면
    구속되지 않은 방향도 잔차가 0 에 가까워 sigma 가 0 으로 나오고, 검출기가 죽는다.
    """
    sigma_meas = max(rmse, SIGMA_MEAS_FLOOR)
    try:
        cov = (sigma_meas ** 2) * np.linalg.inv(info)
        diag = np.abs(np.diag(cov))
    except np.linalg.LinAlgError:
        diag = np.full(6, np.inf)
    sig = np.sqrt(diag)
    return {
        "rot_deg": [float(np.degrees(v)) for v in sig[:3]],
        "trans_m": [float(v) for v in sig[3:]],
    }


def observable_mask(sigmas, abs_limit):
    """축별 관측 가능 여부. 절대 기준과 상대 기준을 모두 만족해야 관측 가능으로 본다.

    절대 기준만 보면 잡음이 섞인 법선 때문에 미끄러지는 축도 통과한다.
    상대 기준만 보면 장면 전체가 나쁠 때(모든 축이 고르게 나쁨) 전부 통과한다.
    """
    sig = np.asarray(sigmas, dtype=np.float64)
    best = float(np.min(sig)) if np.all(np.isfinite(sig)) else 0.0
    ratio_limit = best * SIGMA_RATIO_LIMIT if best > 0 else np.inf
    return (sig <= abs_limit) & (sig <= ratio_limit)


# ---------- 카메라 1대 처리 ----------

def align_camera(cam, lidar_base):
    out = {"key": cam["key"], "ok": False, "reason": None, "dropped_dofs": []}

    R_init = rpy_to_matrix(*cam["rpy"])
    t_init = np.asarray(cam["xyz"], dtype=np.float64)
    T_init = make_transform(R_init, t_init)
    out["init_xyz"] = [float(v) for v in t_init]
    out["init_rpy"] = [float(v) for v in cam["rpy"]]
    out["xyz"] = out["init_xyz"]          # 거부되면 원래값이 그대로 남는다
    out["rpy"] = out["init_rpy"]

    mount = cam["mount"]
    T_mount = make_transform(
        Rotation.from_quat(mount["quat"]).as_matrix(),
        np.asarray(mount["xyz"], dtype=np.float64),
    )

    raw = load_cloud(os.path.join(ROOT, cam["file"]))
    cam_base = apply_transform(T_init @ T_mount, raw)     # 카메라 점군을 base_link 로

    # 겹침 영역만 남긴다. 카메라가 못 보는 방향의 라이다 점은 정합에 방해만 된다.
    lidar_overlap = lidar_base[in_bbox(lidar_base, cam_base)]
    out["lidar_in_view"] = int(len(lidar_overlap))
    if len(lidar_overlap) < MIN_PAIRS:
        out["reason"] = "시야가 겹치는 라이다 점 부족 (%d개)" % len(lidar_overlap)
        return out

    center = t_init.copy()                # 회전 매개화 중심 = 카메라 위치
    T_d = np.eye(4)
    stats = None

    planes_found = False
    for voxel in SCALES:
        tgt = voxel_downsample(cam_base, voxel)
        normals, curvature = estimate_normals(tgt)
        if normals is None:
            continue
        flat = curvature < MAX_CURVATURE
        if flat.sum() < MIN_PAIRS:
            continue
        tgt, normals = tgt[flat], normals[flat]
        planes_found = True

        src_raw = voxel_downsample(lidar_overlap, voxel)
        max_corr = min(CORR_MULT * voxel, MAX_CORR_CAP)
        if "initial_rmse" not in out:
            # 보정 전 잔차. 이 값이 이미 크면 정합 문제가 아니라 데이터 문제다
            # (카메라와 라이다 스냅샷의 촬영 시각이 다르거나, 로봇이 움직였거나).
            out["initial_rmse"] = residual_rmse(src_raw, tgt, normals, max_corr)

        T_step, stats = icp_point_to_plane(apply_transform(T_d, src_raw),
                                           tgt, normals, center, max_corr)
        T_d = T_step @ T_d
        if stats.get("diverged"):
            out["reason"] = "정합이 발산 (누적 보정 %.0fmm 초과) — 초기값이나 스냅샷을 의심" % (DIVERGE_TRANS * 1000)
            return out

    if not planes_found:
        out["reason"] = "카메라 점군에서 평면을 찾지 못함 (장면에 면 구조가 부족)"
        return out
    if stats is None or stats["pairs"] < MIN_PAIRS:
        # 평면은 있는데 그 근처에 라이다 점이 없다 = 초기 오차가 대응 거리보다 큰 경우.
        # 스냅샷 시각이 어긋났거나(로봇이 움직임) 초기 origin 이 많이 틀린 상태.
        out["reason"] = ("대응점 부족 (%d개 < %d) — 초기 오차가 대응 거리 %.0fmm 보다 큼"
                         % (0 if stats is None else stats["pairs"], MIN_PAIRS,
                            min(CORR_MULT * SCALES[-1], MAX_CORR_CAP) * 1000))
        return out

    src_fine = voxel_downsample(lidar_overlap, SCALES[-1])
    out["fitness"] = float(stats["pairs"] / max(len(src_fine), 1))
    out["inlier_rmse"] = stats["rmse"]
    out["pairs"] = stats["pairs"]
    out["sigma"] = dof_sigma(stats["info"], stats["rmse"])

    # 카메라에 줄 보정은 라이다를 움직인 변환의 역이다.
    T_new = np.linalg.inv(T_d) @ T_init
    R_new, t_new = T_new[:3, :3], T_new[:3, 3]

    # 보정량을 [roll,pitch,yaw,x,y,z] 6개로 분해한다. DOF 별 취소가 가능해야 한다.
    rotvec = Rotation.from_matrix(R_new @ R_init.T).as_rotvec()
    dtrans = t_new - t_init

    keep_rot = observable_mask(out["sigma"]["rot_deg"], SIGMA_ROT_LIMIT)
    keep_trans = observable_mask(out["sigma"]["trans_m"], SIGMA_TRANS_LIMIT)
    for i, ok in enumerate(list(keep_rot) + list(keep_trans)):
        if not ok:
            out["dropped_dofs"].append(DOF_NAMES[i])

    rotvec_masked = np.where(keep_rot, rotvec, 0.0)
    dtrans_masked = np.where(keep_trans, dtrans, 0.0)

    R_final = Rotation.from_rotvec(rotvec_masked).as_matrix() @ R_init
    t_final = t_init + dtrans_masked

    out["delta_trans_mm"] = [float(v * 1000.0) for v in dtrans_masked]
    out["delta_rot_deg"] = [float(np.degrees(v)) for v in rotvec_masked]
    trans_norm = float(np.linalg.norm(dtrans_masked))
    rot_norm = float(np.degrees(np.linalg.norm(rotvec_masked)))
    out["delta_trans_norm_mm"] = trans_norm * 1000.0
    out["delta_rot_norm_deg"] = rot_norm

    # 거부 기준. 조용히 통과시키면 자동화가 수동보다 위험해진다.
    if out["fitness"] < MIN_FITNESS:
        out["reason"] = "겹침 부족 (fitness %.2f < %.2f)" % (out["fitness"], MIN_FITNESS)
        return out
    if out["inlier_rmse"] > MAX_RMSE:
        out["reason"] = "정합 품질 미달 (rmse %.1fmm > %.0fmm)" % (out["inlier_rmse"] * 1000, MAX_RMSE * 1000)
        return out
    if trans_norm > MAX_TRANS_FIX:
        out["reason"] = "평행이동 보정 과대 (%.0fmm > %.0fmm) — 오수렴 의심" % (trans_norm * 1000, MAX_TRANS_FIX * 1000)
        return out
    if rot_norm > MAX_ROT_FIX:
        out["reason"] = "회전 보정 과대 (%.2fdeg > %.1fdeg) — 오수렴 의심" % (rot_norm, MAX_ROT_FIX)
        return out
    if len(out["dropped_dofs"]) == 6:
        out["reason"] = "6개 DOF 전부 관측 불가 — 장면에 직교하는 면이 필요"
        return out

    out["xyz"] = [float(v) for v in t_final]
    out["rpy"] = matrix_to_rpy(R_final)
    out["ok"] = True
    return out


def main():
    # 출력은 항상 UTF-8 로 쓴다. 거부 사유에 한글이 들어가므로, 로케일 인코딩에
    # 맡기면 cp949 같은 환경에서 인코딩 에러로 결과가 통째로 날아간다.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except (AttributeError, ValueError):
        pass

    if len(sys.argv) < 2:
        print(json.dumps({"error": "usage: auto_calib.py <request.json>"}), flush=True)
        return 2
    with open(sys.argv[1], "r", encoding="utf-8") as f:
        req = json.load(f)

    lidar = req["lidar"]
    T_lidar = make_transform(
        Rotation.from_quat(lidar["quat"]).as_matrix(),
        np.asarray(lidar["xyz"], dtype=np.float64),
    )
    lidar_base = apply_transform(T_lidar, load_cloud(os.path.join(ROOT, lidar["file"])))

    results = []
    for cam in req["cams"]:
        try:
            results.append(align_camera(cam, lidar_base))
        except Exception as e:          # 한 대가 죽어도 나머지는 계속
            results.append({"key": cam.get("key"), "ok": False,
                            "reason": "%s: %s" % (type(e).__name__, e)})

    print(json.dumps({"results": results}, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
