"""RGBD 카메라 외부 파라미터 자동 정렬.

웹 UI 의 '자동 정렬' 버튼이 calib_server.py 를 통해 이 스크립트를 호출한다.
단독 실행도 된다:  python3 auto_calib.py request.json

축마다 기준이 다르다
--------------------
라이다는 z=0.91 높이의 링 스캔이라 바닥을 못 본다 (실측: base_link 기준 z<0.25 구간에
라이다 점 0개). 그래서 라이다 하나로 6 DOF 를 전부 잡으려 하면 roll, pitch, z 가
구속되지 않은 채로 아무 값이나 나온다. 축을 기준에 맞춰 나눈다.

  1단계 — 바닥 평면 (절대 기준)   -> rot_x, rot_y, z
      평지에 선 로봇이면 바닥은 base_link 에서 z=0, 기울기 0 이어야 한다.
      카메라 자신이 보는 바닥을 RANSAC 으로 골라 그 조건에 맞춘다.
      URDF 주석의 과거 보정 이력(floor-plane recalib)이 쓴 방법과 같다.

  2단계 — 라이다 ICP (상대 기준)  -> x, y, rot_z
      바닥은 이 세 축을 구속하지 못한다 (평면 위를 미끄러진다).
      라이다가 보는 수직 구조(벽·기물)에 point-to-plane ICP 로 맞춘다.
      1단계 결과는 고정하고 세 축만 푼다.

두 단계는 독립으로 판정한다. 라이다 겹침이 얇아 2단계가 거부돼도 1단계만 적용할 수 있고,
그것만으로도 바닥이 그라운드에서 가라앉는 문제는 사라진다.

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

출력(JSON): 카메라별 제안 origin + 단계별 판정 근거. 파일 저장은 하지 않는다.
"""

import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = os.path.dirname(os.path.abspath(__file__))
EZ = np.array([0.0, 0.0, 1.0])

# ---- 1단계: 바닥 평면 ----
FLOOR_BAND = 0.40            # 바닥 후보 높이 상한 (m). base_link 원점이 바닥 위 0 이라고 본다
FLOOR_THICK = 0.02           # 평면 두께 허용 (m)
FLOOR_MAX_TILT = 20.0        # 수평에서 이 이상 기운 평면은 바닥이 아니다 (deg)
FLOOR_ITERS = 400            # RANSAC 반복
MIN_FLOOR_PTS = 500          # 인라이어 하한
MAX_FLOOR_RMS = 0.015        # 바닥 평면 두께 상한 (m). 넘으면 바닥이 아닌 걸 잡았다
MAX_TILT_FIX = 5.0           # 기울기 보정 상한 (deg)
MAX_Z_FIX = 0.10             # z 보정 상한 (m)

# ---- 2단계: 라이다 ICP (x, y, rot_z) ----
SCALES = [0.05, 0.02]        # 멀티스케일 voxel 크기 (m)
CORR_MULT = 2.0              # 대응점 최대 거리 = voxel * 이 값
MAX_CORR_CAP = 0.12          # 대응점 최대 거리 상한 (m). 넘으면 다른 평면으로 건너뛴다
STEP_TRANS_CAP = 0.02        # 반복 1회 평행이동 상한 (m)
STEP_ROT_CAP = 0.0175        # 반복 1회 회전 상한 (rad, 약 1도)
DIVERGE_TRANS = 0.10         # 누적 보정이 이만큼 넘으면 발산으로 보고 중단 (m)
MAX_ITER = 40                # 스케일별 최대 반복
CONV_ROT = 1e-6
CONV_TRANS = 1e-6
BBOX_MARGIN = 0.30           # 시야 겹침 판정용 바운딩박스 여유 (m)
NORMAL_K = 20                # 법선 추정 이웃 개수
MAX_CURVATURE = 0.08         # 곡률이 이보다 크면 평면이 아니므로 법선을 버린다
VERTICAL_MIN_TILT = 30.0     # 2단계는 이 이상 기운 면만 쓴다 (deg). 바닥은 x,y,yaw 를
                             # 구속하지 못하므로 넣으면 대응점만 늘고 해가 나빠진다
TRIM_SIGMA = 3.0             # 잔차 MAD 기준 이상치 제거 배수
MIN_PAIRS = 80               # 대응점 하한
MIN_FITNESS = 0.30           # 겹침 비율 하한
MAX_RMSE = 0.02              # 정합 품질 상한 (m)
MAX_XY_FIX = 0.05            # x, y 보정 상한 (m)
MAX_YAW_FIX = 5.0            # rot_z 보정 상한 (deg)

# 관측 불가 DOF 판정 (2단계). 절대·상대 기준을 모두 본다.
SIGMA_TRANS_LIMIT = 0.010    # m
SIGMA_ROT_LIMIT = 0.5        # deg
SIGMA_RATIO_LIMIT = 15.0     # 같은 그룹 최량 축의 이 배수를 넘으면 버린다
SIGMA_MEAS_FLOOR = 0.005     # 불확실도 계산에 쓰는 측정 잡음 하한 (m)

RNG = np.random.default_rng(0)


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
    nbrs = pts[idx]
    centered = nbrs - nbrs.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", centered, centered) / k
    evals, evecs = np.linalg.eigh(cov)
    normals = evecs[:, :, 0]
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


# ---------- 1단계: 바닥 평면 ----------

def plane_metrics(normal, centroid):
    """평면을 사람이 읽는 값으로: x·y 방향 기울기(deg)와 z 오프셋(m)."""
    n = normal / np.linalg.norm(normal)
    if n[2] < 0:
        n = -n
    return {
        "slope_x_deg": float(np.degrees(np.arctan(-n[0] / n[2]))),
        "slope_y_deg": float(np.degrees(np.arctan(-n[1] / n[2]))),
        "z_off_m": float((centroid @ n) / n[2]),
        "tilt_deg": float(np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0)))),
    }


def segment_floor(pts):
    """RANSAC 으로 바닥 평면을 고른다.

    z 밴드 최소제곱으로 대신하면 안 된다: 벽 밑동이 섞여 기울기가 통째로 틀어진다
    (실측 데이터에서 rear 가 0.2도 대신 -10.3도로 나왔다).
    """
    cand = pts[pts[:, 2] < FLOOR_BAND]
    if len(cand) < MIN_FLOOR_PTS:
        return None, "바닥 후보 점 부족 (%d개)" % len(cand)

    best = None
    for _ in range(FLOOR_ITERS):
        idx = RNG.choice(len(cand), 3, replace=False)
        p0, p1, p2 = cand[idx]
        n = np.cross(p1 - p0, p2 - p0)
        nn = float(np.linalg.norm(n))
        if nn < 1e-9:
            continue
        n = n / nn
        if n[2] < 0:
            n = -n
        if np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0))) > FLOOR_MAX_TILT:
            continue
        inl = np.abs(cand @ n - float(n @ p0)) < FLOOR_THICK
        cnt = int(inl.sum())
        if best is None or cnt > best[0]:
            best = (cnt, inl)

    if best is None or best[0] < MIN_FLOOR_PTS:
        return None, "바닥 평면을 찾지 못함 (수평 면이 %d점 미만)" % MIN_FLOOR_PTS

    inl_pts = cand[best[1]]
    centroid = inl_pts.mean(axis=0)
    _, _, vt = np.linalg.svd(inl_pts - centroid, full_matrices=False)
    normal = vt[2]
    if normal[2] < 0:
        normal = -normal
    resid = (inl_pts - centroid) @ normal

    info = plane_metrics(normal, centroid)
    info["normal"] = normal
    info["centroid"] = centroid
    info["rms_m"] = float(np.sqrt(np.mean(resid ** 2)))
    info["count"] = int(len(inl_pts))
    return info, None


def level_correction(floor, center):
    """바닥을 z=0 수평으로 만드는 보정. center 를 중심으로 회전한다.

    반환: (R_level, dz). 카메라 위치의 x, y 는 건드리지 않는다 — 그 두 축은 바닥이
    구속하지 못하므로 2단계(라이다)에 남겨야 한다.
    """
    n = floor["normal"]
    axis = np.cross(n, EZ)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-12:
        R_level = np.eye(3)
    else:
        angle = float(np.arccos(np.clip(n @ EZ, -1.0, 1.0)))
        R_level = Rotation.from_rotvec(axis / axis_norm * angle).as_matrix()
    leveled_centroid = center + R_level @ (floor["centroid"] - center)
    return R_level, float(-leveled_centroid[2])


# ---------- 2단계: 라이다 ICP (x, y, rot_z) ----------

def build_rows(p, n, center):
    """x, y, rot_z 세 축에 대한 야코비 행. 나머지 축은 1단계가 정한 값을 지킨다.

    p' = p + w_z * (ez x (p - center)) + [tx, ty, 0]
    """
    rel = p - center
    rot_z = np.cross(rel, n)[:, 2]          # (rel x n)_z = ez . (rel x n)
    return np.column_stack([rot_z, n[:, 0], n[:, 1]])


def step_transform(x, center):
    """해 [w_z, tx, ty] 를 4x4 변환으로. center 기준 z축 회전 + xy 평행이동."""
    w = clamp_norm(np.array([0.0, 0.0, x[0]]), STEP_ROT_CAP)
    t = clamp_norm(np.array([x[1], x[2], 0.0]), STEP_TRANS_CAP)
    R_d = Rotation.from_rotvec(w).as_matrix()
    return make_transform(R_d, center - R_d @ center + t)


def icp_xy_yaw(src, tgt, tgt_normals, center, max_corr):
    """소스를 타깃 평면에 맞춘다. x, y, rot_z 만 움직인다."""
    T = np.eye(4)
    tree = cKDTree(tgt)
    stats = {"info": np.zeros((3, 3)), "rmse": float("inf"), "pairs": 0, "diverged": False}

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

        A = build_rows(p, n, center)
        ATA = A.T @ A
        try:
            x = np.linalg.solve(ATA + 1e-12 * np.eye(3), A.T @ (-resid))
        except np.linalg.LinAlgError:
            break

        T = step_transform(x, center) @ T
        stats = {"info": ATA, "rmse": float(np.sqrt(np.mean(resid ** 2))),
                 "pairs": int(len(p)), "diverged": False}

        moved_center = T[:3, 3] + T[:3, :3] @ center - center
        if np.linalg.norm(moved_center) > DIVERGE_TRANS:
            stats["diverged"] = True
            break
        if abs(x[0]) < CONV_ROT and np.linalg.norm(x[1:]) < CONV_TRANS:
            break

    return T, stats


def dof_sigma(info, rmse):
    """정보행렬에서 축별 1-sigma 불확실도. 순서는 [rot_z, x, y].

    공분산 = sigma_meas^2 * inv(info). 어떤 방향 고유값이 0 에 가까우면
    (평면 위 미끄러짐) 그 축의 sigma 가 폭발한다.

    sigma_meas 는 rmse 와 측정 잡음 하한 중 큰 값이다. rmse 만 쓰면 구속되지 않은
    방향도 잔차가 0 에 가까워 sigma 가 0 으로 나오고 검출기가 죽는다.
    """
    sigma_meas = max(rmse, SIGMA_MEAS_FLOOR)
    try:
        diag = np.abs(np.diag((sigma_meas ** 2) * np.linalg.inv(info)))
    except np.linalg.LinAlgError:
        diag = np.full(3, np.inf)
    sig = np.sqrt(diag)
    return {"rot_z_deg": float(np.degrees(sig[0])),
            "x_m": float(sig[1]),
            "y_m": float(sig[2])}


def azimuth_deg(v):
    """xy 평면 방위각 (deg). base_link 의 +x 가 0, 반시계가 +."""
    return float(np.degrees(np.arctan2(v[1], v[0])))


def signed_delta_deg(frm, to):
    """frm 에서 to 로 가는 최소 회전 (deg, -180~180). +는 반시계."""
    return float(((to - frm + 180.0) % 360.0) - 180.0)


def vertical_plane_clusters(pts, voxel=0.05, min_pts=150, tol_deg=25.0):
    """점군에서 수직 면을 찾아 '법선 방향'별로 묶는다.

    벽 하나는 법선 방향 하나다. 서로 다른 법선 방향이 두 개 이상 있어야
    x, y, rot_z 가 모두 구속된다 (벽 하나면 그 벽에 평행한 축이 미끄러진다).
    법선은 앞뒤 구분이 없으므로 180도 주기로 본다.
    """
    down = voxel_downsample(pts, voxel)
    normals, curvature = estimate_normals(down)
    if normals is None:
        return []
    tilt = np.degrees(np.arccos(np.clip(np.abs(normals @ EZ), -1.0, 1.0)))
    keep = (curvature < MAX_CURVATURE) & (tilt > VERTICAL_MIN_TILT)
    if keep.sum() < min_pts:
        return []
    n, p = normals[keep], down[keep]
    az = np.degrees(np.arctan2(n[:, 1], n[:, 0])) % 180.0

    clusters = []
    left = np.ones(len(az), dtype=bool)
    while left.any():
        idx = np.flatnonzero(left)
        # 남은 점 중 이웃이 가장 많은 방향을 씨앗으로 잡는다
        diffs = np.abs(((az[idx][:, None] - az[idx][None, :] + 90.0) % 180.0) - 90.0)
        seed = idx[np.argmax((diffs <= tol_deg).sum(axis=1))]
        near = idx[np.abs(((az[idx] - az[seed] + 90.0) % 180.0) - 90.0) <= tol_deg]
        left[near] = False
        if len(near) < min_pts:
            continue
        # 180도 주기 평균은 각을 두 배로 늘려 평균하고 반으로 되돌린다
        mean_az = float(np.degrees(np.angle(np.mean(np.exp(2j * np.radians(az[near]))))) / 2.0) % 180.0
        centroid = p[near].mean(axis=0)
        clusters.append({"normal_az_deg": mean_az,
                         "position_az_deg": azimuth_deg(centroid),
                         "distance_m": float(np.linalg.norm(centroid[:2])),
                         "points": int(len(near))})
    clusters.sort(key=lambda c: -c["points"])
    return clusters


def camera_forward(pose_rpy, T_mount):
    """카메라 광축(optical frame +z)이 base_link 에서 향하는 단위벡터."""
    v = (rpy_to_matrix(*pose_rpy) @ T_mount[:3, :3]) @ EZ
    return v / np.linalg.norm(v)


def camera_look(pose_rpy, T_mount):
    """광축의 방위각과 내림각(deg). 내림각이 크면 회전해도 벽이 시야에 안 들어온다."""
    v = camera_forward(pose_rpy, T_mount)
    return azimuth_deg(v), float(np.degrees(np.arcsin(np.clip(v[2], -1.0, 1.0))))


def observable_mask(sigmas, abs_limits):
    """축별 관측 가능 여부. 절대 기준과 상대 기준을 모두 만족해야 관측 가능.

    절대 기준만 보면 잡음 섞인 법선 때문에 미끄러지는 축도 통과한다.
    상대 기준만 보면 장면 전체가 고르게 나쁠 때 전부 통과한다.
    """
    sig = np.asarray(sigmas, dtype=np.float64)
    lim = np.asarray(abs_limits, dtype=np.float64)
    scaled = sig / lim                       # 단위가 다른 축을 한 자에 올린다
    best = float(np.min(scaled)) if np.all(np.isfinite(scaled)) else 0.0
    ratio_limit = best * SIGMA_RATIO_LIMIT if best > 0 else np.inf
    return (scaled <= 1.0) & (scaled <= ratio_limit)


# ---------- 3단계: 이웃 카메라 기준 정합 ----------
# 라이다는 실측에서 base_link 기준 z=0.96m 아래를 전혀 보지 못한다 (아래로 향하는 빔이
# 없다). 반면 내림각이 큰 카메라는 벽에서 0.3m 까지 붙어도 0.6~0.7m 높이까지만 본다.
# 두 시야는 어떤 거리에서도 겹치지 않으므로, 이 카메라들의 x, y, rot_z 는 라이다로
# 맞출 수 없다. 대신 이미 라이다로 맞춰진 이웃 카메라를 기준으로 삼는다.
#
# 주의: 기준의 오차가 그대로 전달된다 (연쇄). 그래서 2단계를 통과한 카메라만 기준으로 쓴다.

NEIGHBOR_FLOOR_EXCLUDE = 0.06    # 이 높이 이하(바닥)는 제외. 바닥은 x, y, rot_z 를 못 잡는다
NEIGHBOR_MIN_TARGET = 300        # 기준 점군의 수직면 점이 이보다 적으면 포기


def align_to_neighbors(out, refs):
    """라이다로 못 맞춘 카메라를 이미 맞춰진 카메라에 붙인다. x, y, rot_z 만 움직인다.

    refs: [(key, 기준 카메라 점군(base_link, 정렬 완료))] — 2단계를 통과한 것만
    """
    info = {"ok": False, "references": [r[0] for r in refs]}
    out["neighbor"] = info

    if not refs:
        info["reason"] = "라이다로 정렬된 기준 카메라가 없음 — 먼저 다른 카메라의 2단계를 통과시켜야 함"
        return

    src_all = out.pop("_cloud", None)
    if src_all is None or len(src_all) == 0:
        info["reason"] = "점군이 없음"
        return

    R_cur = rpy_to_matrix(*out["rpy"])
    t_cur = np.asarray(out["xyz"], dtype=np.float64)
    center = t_cur.copy()

    src = src_all[src_all[:, 2] > NEIGHBOR_FLOOR_EXCLUDE]
    if len(src) < MIN_PAIRS:
        info["reason"] = "바닥 위 점이 부족 (%d개) — 벽이나 기물이 시야에 필요" % len(src)
        return

    ref_pts = np.vstack([c for _, c in refs])
    ref_pts = ref_pts[ref_pts[:, 2] > NEIGHBOR_FLOOR_EXCLUDE]
    ref_pts = ref_pts[in_bbox(ref_pts, src)]
    info["overlap_points"] = int(len(ref_pts))
    if len(ref_pts) < NEIGHBOR_MIN_TARGET:
        info["reason"] = ("기준 카메라와 겹치는 영역이 부족 (%d점) — 두 카메라가 같은 벽을 보게 하십시오"
                          % len(ref_pts))
        return

    T_d = np.eye(4)
    stats = None
    vertical_found = False
    for voxel in SCALES:
        tgt = voxel_downsample(ref_pts, voxel)
        normals, curvature = estimate_normals(tgt)
        if normals is None:
            continue
        tilt = np.degrees(np.arccos(np.clip(np.abs(normals @ EZ), -1.0, 1.0)))
        keep = (curvature < MAX_CURVATURE) & (tilt > VERTICAL_MIN_TILT)
        if keep.sum() < MIN_PAIRS:
            continue
        tgt, normals = tgt[keep], normals[keep]
        vertical_found = True
        # 여기서는 소스가 카메라 자신이다. 라이다를 움직이던 2단계와 달리 역변환이 필요 없다.
        moved = apply_transform(T_d, voxel_downsample(src, voxel))
        T_step, stats = icp_xy_yaw(moved, tgt, normals, center,
                                   min(CORR_MULT * voxel, MAX_CORR_CAP))
        T_d = T_step @ T_d
        if stats["diverged"]:
            break

    if not vertical_found:
        info["reason"] = "겹치는 영역에 수직 면이 없음 — 두 카메라가 같은 벽(또는 기물)을 보게 하십시오"
        return
    if stats is None or stats["pairs"] < MIN_PAIRS:
        info["reason"] = "대응점 부족 (%d개 < %d)" % (0 if stats is None else stats["pairs"], MIN_PAIRS)
        return
    if stats["diverged"]:
        info["reason"] = "정합이 발산 (누적 보정 %.0fmm 초과)" % (DIVERGE_TRANS * 1000)
        return

    src_fine = voxel_downsample(src, SCALES[-1])
    fitness = float(stats["pairs"] / max(len(src_fine), 1))
    sigma = dof_sigma(stats["info"], stats["rmse"])
    info.update({"fitness": fitness, "rmse_mm": stats["rmse"] * 1000,
                 "pairs": stats["pairs"], "sigma": sigma})

    yaw_fix = float(Rotation.from_matrix(T_d[:3, :3]).as_rotvec()[2])
    xy_fix = (T_d[:3, 3] + T_d[:3, :3] @ center - center)[:2]

    keep_dof = observable_mask([sigma["rot_z_deg"], sigma["x_m"], sigma["y_m"]],
                               [SIGMA_ROT_LIMIT, SIGMA_TRANS_LIMIT, SIGMA_TRANS_LIMIT])
    dropped = [n for n, k in zip(["rot_z", "x", "y"], keep_dof) if not k]
    yaw_fix = yaw_fix if keep_dof[0] else 0.0
    xy_fix = np.array([xy_fix[0] if keep_dof[1] else 0.0,
                       xy_fix[1] if keep_dof[2] else 0.0])
    info["dropped_dofs"] = dropped
    info.update({"fix_x_mm": float(xy_fix[0] * 1000), "fix_y_mm": float(xy_fix[1] * 1000),
                 "fix_yaw_deg": float(np.degrees(yaw_fix))})

    xy_norm = float(np.linalg.norm(xy_fix))
    if fitness < MIN_FITNESS:
        info["reason"] = "겹침 부족 (fitness %.2f < %.2f)" % (fitness, MIN_FITNESS)
        return
    if stats["rmse"] > MAX_RMSE:
        info["reason"] = "정합 품질 미달 (rmse %.1fmm > %.0fmm)" % (stats["rmse"] * 1000, MAX_RMSE * 1000)
        return
    if xy_norm > MAX_XY_FIX:
        info["reason"] = "x, y 보정 과대 (%.0fmm > %.0fmm) — 오수렴 의심" % (xy_norm * 1000, MAX_XY_FIX * 1000)
        return
    if abs(np.degrees(yaw_fix)) > MAX_YAW_FIX:
        info["reason"] = "rot_z 보정 과대 (%.2fdeg > %.1fdeg) — 오수렴 의심" % (np.degrees(yaw_fix), MAX_YAW_FIX)
        return
    if len(dropped) == 3:
        info["reason"] = "x, y, rot_z 전부 관측 불가 — 서로 다른 방향의 수직 면이 필요"
        return

    R_new = Rotation.from_rotvec([0.0, 0.0, yaw_fix]).as_matrix() @ R_cur
    out["xyz"] = [float(v) for v in (t_cur + np.array([xy_fix[0], xy_fix[1], 0.0]))]
    out["rpy"] = matrix_to_rpy(R_new)
    out["ok"] = True
    info["ok"] = True


# ---------- 장면 가이드 ----------
# 사용자가 다음에 무엇을 해야 하는지 알려주기 위한 계산. 라이다는 360도를 보므로
# 벽이 어디 있는지 알 수 있고, 카메라 광축 방위와 비교하면 필요한 회전량이 나온다.

CAM_HFOV_DEG = 80.0          # RGBD 수평 화각 (보수적으로 잡음). 절반이 시야 반각
CAM_VFOV_DEG = 58.0          # 수직 화각
ROTATE_ROUND_DEG = 5.0       # 회전 안내는 이 단위로 반올림한다
WALL_DIR_MIN_SEP = 30.0      # 두 벽 방향이 이만큼 달라야 서로 다른 방향으로 센다
CORNER_MIN_ANGLE = 60.0      # 두 벽 법선이 이만큼 이상 벌어져야 '구석'으로 센다
CORNER_MAX_DIST = 6.0        # 이보다 먼 구석은 안내에 쓰지 않는다 (m)
WALL_BAND_MIN = 0.15         # 벽에서 이 높이만큼은 시야에 들어와야 면으로 잡힌다 (m)


def find_corners(wall_clusters):
    """수직 면 두 개가 직각으로 만나는 지점(구석)의 위치를 구한다.

    구석으로 접근하면 직각인 두 벽이 한 시야에 들어온다 = x, y, rot_z 가 한 번에 구속된다.
    박스를 놓을 수 없는 현장에서는 이게 유일한 방법이다.

    벽을 xy 평면의 직선 n·x = n·c 으로 보고 두 직선의 교점을 푼다.
    """
    corners = []
    for i in range(len(wall_clusters)):
        for j in range(i + 1, len(wall_clusters)):
            a, b = wall_clusters[i], wall_clusters[j]
            sep = abs(((a["normal_az_deg"] - b["normal_az_deg"] + 90.0) % 180.0) - 90.0)
            if sep < CORNER_MIN_ANGLE:
                continue
            na = np.array([np.cos(np.radians(a["normal_az_deg"])), np.sin(np.radians(a["normal_az_deg"]))])
            nb = np.array([np.cos(np.radians(b["normal_az_deg"])), np.sin(np.radians(b["normal_az_deg"]))])
            # 법선·위치로 각 벽의 원점 거리를 구한다 (위치 방위와 거리로 중심점 복원)
            ca = a["distance_m"] * np.array([np.cos(np.radians(a["position_az_deg"])),
                                             np.sin(np.radians(a["position_az_deg"]))])
            cb = b["distance_m"] * np.array([np.cos(np.radians(b["position_az_deg"])),
                                             np.sin(np.radians(b["position_az_deg"]))])
            try:
                xy = np.linalg.solve(np.vstack([na, nb]), np.array([na @ ca, nb @ cb]))
            except np.linalg.LinAlgError:
                continue
            dist = float(np.linalg.norm(xy))
            if dist > CORNER_MAX_DIST:
                continue
            corners.append({"position_az_deg": azimuth_deg(xy), "distance_m": dist,
                            "angle_deg": float(sep),
                            "points": a["points"] + b["points"]})
    corners.sort(key=lambda c: c["distance_m"])
    return corners


def max_wall_distance(cam_height, cam_elev):
    """아래를 보는 카메라가 벽 밑동을 볼 수 있는 최대 거리 (m).

    시야 상단 광선이 지평선보다 아래를 향하면(내림각 > 수직화각/2) 그 광선이 바닥에
    닿는 거리까지만 볼 수 있다. 벽이 그보다 가까워야 밑동이 시야에 들어온다.
    WALL_BAND_MIN 만큼의 높이는 확보해야 면으로 인식된다.
    """
    top_elev = cam_elev + CAM_VFOV_DEG / 2.0
    if top_elev >= 0.0:
        return None                      # 지평선을 보므로 거리 제약이 없다
    usable = cam_height - WALL_BAND_MIN
    if usable <= 0:
        return None
    return float(usable / np.tan(np.radians(-top_elev)))


def turn_words(turn):
    """회전 안내 문구: (방향, 반올림한 각도)."""
    return ("반시계" if turn > 0 else "시계",
            int(round(abs(turn) / ROTATE_ROUND_DEG) * ROTATE_ROUND_DEG))


CLOSEST_PRACTICAL_M = 0.30       # 로봇을 벽에 이보다 더 붙이기는 어렵다


def wall_guidance(cam_clusters, cam_az, cam_elev, cam_height, lidar_clusters, corners,
                  lidar_min_z=None):
    """카메라 한 대에 대한 장면 가이드. 벽만으로 해결하는 방법을 안내한다.

    cam_clusters: 그 카메라가 실제로 보고 있는 수직 면 (법선 방향별)
    cam_elev: 광축 내림각. 크면 벽 밑동만 보이므로 접근 거리 제약이 생긴다
    lidar_clusters: 라이다가 360도에서 본 수직 면 = 주변 벽의 방위와 거리
    corners: 직각으로 만나는 두 벽의 교점. 구석 하나로 두 방향을 동시에 얻는다
    """
    seen = [c["normal_az_deg"] for c in cam_clusters]
    distinct = []
    for az in seen:
        if all(abs(((az - d + 90.0) % 180.0) - 90.0) >= WALL_DIR_MIN_SEP for d in distinct):
            distinct.append(az)

    reach = max_wall_distance(cam_height, cam_elev)
    out = {"walls_seen": len(distinct),
           "wall_dirs_deg": [round(a, 1) for a in distinct],
           "camera_azimuth_deg": round(cam_az, 1),
           "camera_elevation_deg": round(cam_elev, 1),
           "turn_deg": 0}
    if reach is not None:
        out["max_wall_distance_m"] = round(reach, 2)

    if len(distinct) >= 2:
        out["level"] = "ok"
        out["text"] = "벽 방향 2개 이상 보임 — x, y, rot_z 모두 구속 가능"
        return out

    # 라이다와 시야 대역이 아예 겹치지 않는 카메라는 접근해도 소용이 없다.
    # 실측: 라이다는 z=0.96m 아래를 보지 못하고, 내림각 54도 카메라는 벽에 0.3m 까지
    # 붙어도 0.60m 위를 못 본다. 이 경우 3단계(이웃 카메라 기준)로 넘긴다.
    top_elev = cam_elev + CAM_VFOV_DEG / 2.0
    if lidar_min_z is not None and top_elev < 0.0:
        best_reach_z = cam_height + CLOSEST_PRACTICAL_M * np.tan(np.radians(top_elev))
        if best_reach_z < lidar_min_z:
            out["level"] = "lidar_blind"
            out["text"] = ("라이다는 z=%.2fm 아래를 보지 못하고, 이 카메라는 벽에 %.1fm 까지 붙어도 "
                           "z=%.2fm 위를 못 봅니다 — x, y, rot_z 를 라이다로 맞출 방법이 없습니다. "
                           "이미 라이다로 정렬된 카메라와 같은 벽·기물을 함께 보게 배치하십시오 "
                           "(3단계가 자동으로 그 카메라를 기준으로 붙입니다)"
                           % (lidar_min_z, CLOSEST_PRACTICAL_M, best_reach_z))
            return out

    # 구석을 우선 노린다. 직각인 두 벽이 한 시야에 들어오면 세 축이 한 번에 구속된다.
    target, kind = None, None
    if corners:
        best = min(corners, key=lambda c: abs(signed_delta_deg(cam_az, c["position_az_deg"])))
        target, kind = best, "corner"
    if target is None:
        usable = [w for w in lidar_clusters
                  if not distinct
                  or abs(((w["normal_az_deg"] - distinct[0] + 90.0) % 180.0) - 90.0) >= WALL_DIR_MIN_SEP]
        if usable:
            target = min(usable, key=lambda w: abs(signed_delta_deg(cam_az, w["position_az_deg"])))
            kind = "wall"

    if target is None:
        out["level"] = "blocked"
        out["text"] = ("라이다 점군에서 쓸 수 있는 벽을 찾지 못했습니다 — 로봇을 벽이 있는 곳으로 "
                       "옮긴 뒤 '데이터 갱신' 하고 다시 실행하십시오")
        return out

    turn = signed_delta_deg(cam_az, target["position_az_deg"])
    way, rounded = turn_words(turn)
    out["turn_deg"] = int(round(turn / ROTATE_ROUND_DEG) * ROTATE_ROUND_DEG)
    out["target"] = {"kind": kind, "azimuth_deg": round(target["position_az_deg"], 1),
                     "distance_m": round(target["distance_m"], 2)}

    where = "구석" if kind == "corner" else "벽"
    aim = ("" if rounded == 0 else "로봇을 %s %d도 회전해 " % (way, rounded))
    now = "현재 거리 %.1fm" % target["distance_m"]

    # 내림각이 커서 벽 밑동만 보이는 카메라는 접근 거리를 반드시 지켜야 한다.
    if reach is not None:
        need_closer = target["distance_m"] > reach
        out["level"] = "approach" if need_closer else ("one_wall" if distinct else "no_wall")
        out["text"] = (
            "내림각 %.0f도라 벽 밑동만 시야에 들어옵니다 — %s 이내로 접근해야 벽이 잡힙니다 (%s). "
            "%s%s을 정면으로 보게 하십시오. %s이면 직각인 두 벽이 함께 들어와 x, y, rot_z 가 한 번에 구속됩니다"
            % (-cam_elev, "%.1fm" % reach, now, aim, where, where))
        return out

    if abs(turn) <= CAM_HFOV_DEG / 2.0 and distinct:
        out["level"] = "weak"
        out["text"] = ("벽이 시야 안(%s, %s)에 있으나 두 번째 방향 면이 잡히지 않습니다 — "
                       "%s쪽으로 더 가까이 붙거나 정면으로 보게 하십시오" % (where, now, where))
        return out

    out["level"] = "one_wall" if distinct else "no_wall"
    head = ("벽 하나만 보임 — 그 벽에 평행한 축은 제외됩니다. "
            if distinct else "이 카메라 시야에 벽이 없습니다. ")
    out["text"] = (head + "%s%s(%s)을 시야에 넣고 '데이터 갱신' 후 다시 실행하십시오%s"
                   % (aim, where, now,
                      " — 구석이면 세 축이 한 번에 구속됩니다" if kind == "corner" else ""))
    return out


def scene_summary(results):
    """전체 요약. 한 자세에서 모든 카메라를 만족시킬 수 없는 경우를 안내한다."""
    need = [r for r in results if r.get("advice", {}).get("level", "ok") != "ok"]
    ok = [r["key"] for r in results if r.get("advice", {}).get("level") == "ok"]
    if not need:
        return {"text": "모든 카메라가 서로 다른 방향의 벽을 2개 이상 보고 있습니다 — x, y, rot_z 판정 가능",
                "ready": True}

    lines = []
    if ok:
        lines.append("벽을 충분히 보는 카메라: %s" % ", ".join(ok))
    lines.append("장면이 부족한 카메라: %s" % ", ".join(r["key"] for r in need))

    # 라이다와 시야가 아예 겹치지 않는 카메라는 3단계(이웃 카메라 기준)로만 해결된다.
    blind = [r["key"] for r in need if r["advice"].get("level") == "lidar_blind"]
    if blind:
        lines.append("%s 는 라이다와 시야 대역이 겹치지 않습니다 — 이미 정렬된 카메라와 같은 벽을 "
                     "함께 보게 두면 3단계가 그 카메라를 기준으로 붙입니다" % ", ".join(blind))

    # 접근 거리가 필요한 카메라를 먼저 알린다. 회전만으로는 해결되지 않기 때문이다.
    approach = [r for r in need
                if r["advice"].get("max_wall_distance_m")
                and r["advice"].get("level") != "lidar_blind"]
    if approach:
        worst = min(approach, key=lambda r: r["advice"]["max_wall_distance_m"])
        lines.append("%s 는 내림각이 커서 벽에서 %.1fm 이내로 접근해야 합니다"
                     % (worst["key"], worst["advice"]["max_wall_distance_m"]))

    turns = sorted((abs(r["advice"]["turn_deg"]), r["advice"]["turn_deg"], r["key"])
                   for r in need if r["advice"].get("turn_deg"))
    if turns:
        _, turn, key = turns[0]
        way = "반시계" if turn > 0 else "시계"
        lines.append("가장 적은 회전으로 해결되는 것은 %s — 로봇을 %s %d도 회전"
                     % (key, way, abs(turn)))

    lines.append("구석을 노리십시오. 직각인 두 벽이 한 시야에 들어오면 x, y, rot_z 가 한 번에 구속됩니다. "
                 "한 자세에서 5대를 모두 만족시키기는 어려우므로, 이동·회전 → 데이터 갱신 → 자동 정렬을 "
                 "반복하면 됩니다. 조건을 만족한 카메라만 값이 바뀌고 나머지는 유지되므로 결과가 누적됩니다.")
    return {"text": " / ".join(lines), "ready": False}


# ---------- 카메라 1대 처리 ----------

def align_camera(cam, lidar_base):
    out = {"key": cam["key"], "ok": False, "floor": {"ok": False}, "lidar": {"ok": False}}

    R_cur = rpy_to_matrix(*cam["rpy"])
    t_cur = np.asarray(cam["xyz"], dtype=np.float64)
    out["init_xyz"] = [float(v) for v in t_cur]
    out["init_rpy"] = [float(v) for v in cam["rpy"]]

    T_mount = make_transform(
        Rotation.from_quat(cam["mount"]["quat"]).as_matrix(),
        np.asarray(cam["mount"]["xyz"], dtype=np.float64))
    raw = load_cloud(os.path.join(ROOT, cam["file"]))

    center = t_cur.copy()          # 회전 매개화 중심 = 카메라 위치 (회전·평행이동 분리)

    # ---- 1단계: 바닥 평면으로 rot_x, rot_y, z ----
    cam_base = apply_transform(make_transform(R_cur, t_cur) @ T_mount, raw)
    floor, err = segment_floor(cam_base)
    if floor is None:
        out["floor"]["reason"] = err
    else:
        out["floor"].update({
            "before": {"slope_x_deg": floor["slope_x_deg"],
                       "slope_y_deg": floor["slope_y_deg"],
                       "z_off_mm": floor["z_off_m"] * 1000},
            "points": floor["count"],
            "plane_rms_mm": floor["rms_m"] * 1000,
        })
        R_level, dz = level_correction(floor, center)
        tilt_fix = float(np.degrees(np.linalg.norm(
            Rotation.from_matrix(R_level).as_rotvec())))
        out["floor"].update({"tilt_fix_deg": tilt_fix, "dz_mm": dz * 1000})

        if floor["rms_m"] > MAX_FLOOR_RMS:
            out["floor"]["reason"] = ("바닥 평면이 너무 두꺼움 (%.1fmm > %.0fmm) — 바닥이 아닌 면을 잡았을 수 있음"
                                      % (floor["rms_m"] * 1000, MAX_FLOOR_RMS * 1000))
        elif tilt_fix > MAX_TILT_FIX:
            out["floor"]["reason"] = ("기울기 보정 과대 (%.2fdeg > %.1fdeg)" % (tilt_fix, MAX_TILT_FIX))
        elif abs(dz) > MAX_Z_FIX:
            out["floor"]["reason"] = ("z 보정 과대 (%.0fmm > %.0fmm)" % (dz * 1000, MAX_Z_FIX * 1000))
        else:
            R_cur = R_level @ R_cur
            t_cur = t_cur + np.array([0.0, 0.0, dz])
            out["floor"]["ok"] = True
            # 보정 후 바닥을 다시 재서 결과를 보고한다 (기울기 0, z 0 에 가까워야 한다)
            cam_base = apply_transform(make_transform(R_cur, t_cur) @ T_mount, raw)
            after, _ = segment_floor(cam_base)
            if after is not None:
                out["floor"]["after"] = {"slope_x_deg": after["slope_x_deg"],
                                         "slope_y_deg": after["slope_y_deg"],
                                         "z_off_mm": after["z_off_m"] * 1000}

    # ---- 2단계: 라이다 ICP 로 x, y, rot_z ----
    lidar_overlap = lidar_base[in_bbox(lidar_base, cam_base)]
    out["lidar"]["in_view"] = int(len(lidar_overlap))
    if len(lidar_overlap) < MIN_PAIRS:
        out["lidar"]["reason"] = "시야가 겹치는 라이다 점 부족 (%d개)" % len(lidar_overlap)
    else:
        T_d = np.eye(4)
        stats = None
        vertical_found = False
        for voxel in SCALES:
            tgt = voxel_downsample(cam_base, voxel)
            normals, curvature = estimate_normals(tgt)
            if normals is None:
                continue
            # 바닥은 x, y, rot_z 를 구속하지 못한다. 수직 구조만 남긴다.
            tilt = np.degrees(np.arccos(np.clip(np.abs(normals @ EZ), -1.0, 1.0)))
            keep = (curvature < MAX_CURVATURE) & (tilt > VERTICAL_MIN_TILT)
            if keep.sum() < MIN_PAIRS:
                continue
            tgt, normals = tgt[keep], normals[keep]
            vertical_found = True

            max_corr = min(CORR_MULT * voxel, MAX_CORR_CAP)
            src = apply_transform(T_d, voxel_downsample(lidar_overlap, voxel))
            T_step, stats = icp_xy_yaw(src, tgt, normals, center, max_corr)
            T_d = T_step @ T_d
            if stats["diverged"]:
                break

        if not vertical_found:
            out["lidar"]["reason"] = "카메라 시야에 수직 면이 없음 — x, y, rot_z 를 구속할 구조가 필요"
        elif stats is None or stats["pairs"] < MIN_PAIRS:
            out["lidar"]["reason"] = ("대응점 부족 (%d개 < %d)"
                                      % (0 if stats is None else stats["pairs"], MIN_PAIRS))
        elif stats["diverged"]:
            out["lidar"]["reason"] = "정합이 발산 (누적 보정 %.0fmm 초과)" % (DIVERGE_TRANS * 1000)
        else:
            src_fine = voxel_downsample(lidar_overlap, SCALES[-1])
            fitness = float(stats["pairs"] / max(len(src_fine), 1))
            sigma = dof_sigma(stats["info"], stats["rmse"])
            out["lidar"].update({"fitness": fitness, "rmse_mm": stats["rmse"] * 1000,
                                 "pairs": stats["pairs"], "sigma": sigma})

            # 카메라에 줄 보정은 라이다를 움직인 변환의 역이다
            T_fix = np.linalg.inv(T_d)
            yaw_fix = float(Rotation.from_matrix(T_fix[:3, :3]).as_rotvec()[2])
            xy_fix = (T_fix[:3, 3] + T_fix[:3, :3] @ center - center)[:2]

            keep = observable_mask([sigma["rot_z_deg"], sigma["x_m"], sigma["y_m"]],
                                   [SIGMA_ROT_LIMIT, SIGMA_TRANS_LIMIT, SIGMA_TRANS_LIMIT])
            dropped = [name for name, k in zip(["rot_z", "x", "y"], keep) if not k]
            yaw_fix = yaw_fix if keep[0] else 0.0
            xy_fix = np.array([xy_fix[0] if keep[1] else 0.0,
                               xy_fix[1] if keep[2] else 0.0])
            out["lidar"]["dropped_dofs"] = dropped
            out["lidar"].update({"fix_x_mm": float(xy_fix[0] * 1000),
                                 "fix_y_mm": float(xy_fix[1] * 1000),
                                 "fix_yaw_deg": float(np.degrees(yaw_fix))})

            xy_norm = float(np.linalg.norm(xy_fix))
            if fitness < MIN_FITNESS:
                out["lidar"]["reason"] = "겹침 부족 (fitness %.2f < %.2f)" % (fitness, MIN_FITNESS)
            elif stats["rmse"] > MAX_RMSE:
                out["lidar"]["reason"] = ("정합 품질 미달 (rmse %.1fmm > %.0fmm)"
                                          % (stats["rmse"] * 1000, MAX_RMSE * 1000))
            elif xy_norm > MAX_XY_FIX:
                out["lidar"]["reason"] = ("x, y 보정 과대 (%.0fmm > %.0fmm) — 오수렴 의심"
                                          % (xy_norm * 1000, MAX_XY_FIX * 1000))
            elif abs(np.degrees(yaw_fix)) > MAX_YAW_FIX:
                out["lidar"]["reason"] = ("rot_z 보정 과대 (%.2fdeg > %.1fdeg) — 오수렴 의심"
                                          % (np.degrees(yaw_fix), MAX_YAW_FIX))
            elif len(dropped) == 3:
                out["lidar"]["reason"] = "x, y, rot_z 전부 관측 불가 — 서로 다른 방향의 수직 면이 필요"
            else:
                R_yaw = Rotation.from_rotvec([0.0, 0.0, yaw_fix]).as_matrix()
                R_cur = R_yaw @ R_cur
                t_cur = t_cur + np.array([xy_fix[0], xy_fix[1], 0.0])
                out["lidar"]["ok"] = True

    out["xyz"] = [float(v) for v in t_cur]
    out["rpy"] = matrix_to_rpy(R_cur)
    out["ok"] = bool(out["floor"]["ok"] or out["lidar"]["ok"])

    # 장면 진단: 이 카메라가 어느 방향의 벽을 보고 있는지. 가이드 계산에 쓴다.
    # 3단계(이웃 카메라 기준)에서 쓸 최종 점군. main 이 JSON 으로 내보내기 전에 지운다.
    out["_cloud"] = apply_transform(make_transform(R_cur, t_cur) @ T_mount, raw)

    cam_az, cam_elev = camera_look(out["rpy"], T_mount)
    out["scene"] = {
        "vertical_planes": vertical_plane_clusters(cam_base),
        "camera_azimuth_deg": round(cam_az, 1),
        "camera_elevation_deg": round(cam_elev, 1),
        "camera_height_m": float(t_cur[2]),
    }
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
        np.asarray(lidar["xyz"], dtype=np.float64))
    lidar_base = apply_transform(T_lidar, load_cloud(os.path.join(ROOT, lidar["file"])))

    results = []
    extra_refs = []
    for cam in req["cams"]:
        # "reference": true 인 카메라는 계산하지 않고 기준으로만 쓴다. 이전 회차에서
        # 이미 정렬해 잠근 카메라를 화면이 이렇게 넘긴다 (3단계의 기준이 된다).
        if cam.get("reference"):
            try:
                T_ref = make_transform(rpy_to_matrix(*cam["rpy"]),
                                       np.asarray(cam["xyz"], dtype=np.float64))
                T_m = make_transform(
                    Rotation.from_quat(cam["mount"]["quat"]).as_matrix(),
                    np.asarray(cam["mount"]["xyz"], dtype=np.float64))
                extra_refs.append((cam["key"], apply_transform(
                    T_ref @ T_m, load_cloud(os.path.join(ROOT, cam["file"])))))
                results.append({"key": cam["key"], "role": "reference", "ok": False,
                                "floor": {"ok": False}, "lidar": {"ok": False}})
            except Exception as e:
                results.append({"key": cam.get("key"), "role": "reference", "ok": False,
                                "floor": {"ok": False}, "lidar": {"ok": False},
                                "error": "%s: %s" % (type(e).__name__, e)})
            continue
        try:
            results.append(align_camera(cam, lidar_base))
        except Exception as e:          # 한 대가 죽어도 나머지는 계속
            results.append({"key": cam.get("key"), "ok": False,
                            "floor": {"ok": False}, "lidar": {"ok": False},
                            "error": "%s: %s" % (type(e).__name__, e)})

    # 3단계: 라이다로 못 맞춘 카메라를 이미 맞춰진 카메라에 붙인다.
    refs = extra_refs + [(r["key"], r["_cloud"]) for r in results
                         if r.get("lidar", {}).get("ok") and r.get("_cloud") is not None]
    for r in results:
        if r.get("lidar", {}).get("ok") or "_cloud" not in r:
            continue
        try:
            align_to_neighbors(r, [(k, c) for k, c in refs if k != r["key"]])
        except Exception as e:
            r["neighbor"] = {"ok": False, "reason": "%s: %s" % (type(e).__name__, e)}
    for r in results:
        r.pop("_cloud", None)        # 점군은 JSON 으로 내보내지 않는다

    lidar_walls = vertical_plane_clusters(lidar_base, voxel=0.05, min_pts=120)
    corners = find_corners(lidar_walls)
    for r in results:
        scene = r.get("scene")
        if scene:
            r["advice"] = wall_guidance(scene["vertical_planes"],
                                        scene["camera_azimuth_deg"],
                                        scene["camera_elevation_deg"],
                                        scene["camera_height_m"], lidar_walls, corners,
                                        lidar_min_z=float(lidar_base[:, 2].min()))

    print(json.dumps({
        "results": results,
        "lidar_walls": [{"normal_az_deg": round(w["normal_az_deg"], 1),
                         "position_az_deg": round(w["position_az_deg"], 1),
                         "distance_m": round(w["distance_m"], 2),
                         "points": w["points"]} for w in lidar_walls],
        "corners": [{"azimuth_deg": round(c["position_az_deg"], 1),
                     "distance_m": round(c["distance_m"], 2),
                     "angle_deg": round(c["angle_deg"], 1)} for c in corners],
        "guidance": scene_summary(results),
    }, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
