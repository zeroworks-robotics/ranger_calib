"""index.html 의 상수로 auto_calib.py 입력 request.json 을 만든다.

웹 UI 없이 자동 정렬을 돌려보기 위한 도구다. UI 가 보내는 것과 같은 내용을
index.html 에서 직접 뽑아내므로, 상수를 여기에 복사해 두지 않는다.

  python3 tests/make_request.py [출력경로]      # 기본값: tests/request.json
"""

import json
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HTML = os.path.join(ROOT, "index.html")

CHILD_LINK = {
    "front": "camera_frontRGBD_link",
    "fdown": "camera_fdownRGBD_link",
    "right": "camera_rightRGBD_link",
    "rear": "camera_rearRGBD_link",
    "left": "camera_leftRGBD_link",
}


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def rpy_to_quat(roll, pitch, yaw):
    """URDF 규약 R = Rz(yaw) Ry(pitch) Rx(roll) 을 쿼터니언 [x,y,z,w] 로."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy]


def build(out_path):
    with open(HTML, encoding="utf-8") as f:
        src = f.read()

    cams_js = src.split("const CAMERAS = [", 1)[1].split("\n];", 1)[0]
    entries = re.findall(
        r'key:"(\w+)".*?file:"([^"]+)".*?mountT:\[([^\]]+)\].*?mountQ1:\[([^\]]+)\]',
        cams_js, re.S)
    if len(entries) != len(CHILD_LINK):
        raise SystemExit("index.html 에서 카메라 %d대만 찾음 (기대 %d대)"
                         % (len(entries), len(CHILD_LINK)))
    mount_q2 = [float(v) for v in
                re.search(r"const MOUNT_Q2 = \[([^\]]+)\]", src).group(1).split(",")]

    def origin_of(link):
        m = re.search(r'<child link="%s"/>\s*<origin xyz="([^"]+)" rpy="([^"]+)"' % link, src)
        if not m:
            raise SystemExit("index.html 내장 URDF 에서 %s origin 을 못 찾음" % link)
        return ([float(v) for v in m.group(1).split()],
                [float(v) for v in m.group(2).split()])

    cams = []
    for key, cloud_file, mount_t, mount_q1 in entries:
        xyz, rpy = origin_of(CHILD_LINK[key])
        q1 = [float(v) for v in mount_q1.split(",")]
        cams.append({
            "key": key,
            "file": cloud_file,
            "xyz": xyz,
            "rpy": rpy,
            "mount": {"xyz": [float(v) for v in mount_t.split(",")],
                      "quat": quat_mul(q1, mount_q2)},
        })

    lidar_xyz, lidar_rpy = origin_of("rslidar")
    req = {
        "lidar": {"file": "data/rslidar_points.json",
                  "xyz": lidar_xyz,
                  "quat": rpy_to_quat(*lidar_rpy)},
        "cams": cams,
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(req, f, indent=2)
    return req


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "tests", "request.json")
    r = build(out)
    print("%s 생성 — 카메라 %s" % (out, ", ".join(c["key"] for c in r["cams"])))
