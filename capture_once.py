import os, json, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

# 설치 위치에 상관없이 동작하도록 이 스크립트가 있는 디렉터리 기준으로 유도한다.
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
MAX_POINTS = 20000
TIMEOUT_SEC = 8.0
# RGBD 드라이버가 무효 깊이 픽셀을 NaN이 아니라 (0,0,0)으로 내보내므로
# skip_nans 로는 안 걸러진다. 센서 최소 측정거리보다 가까운 점은 전부 무효로 본다.
MIN_RANGE = 0.05

TOPICS = {
    "rslidar_points": "rslidar_points.json",
    "camera_frontRGBD/depth/points": "camera_frontRGBD_depth_points.json",
    "camera_fdownRGBD/depth/points": "camera_fdownRGBD_depth_points.json",
    "camera_rightRGBD/depth/points": "camera_rightRGBD_depth_points.json",
    "camera_rearRGBD/depth/points": "camera_rearRGBD_depth_points.json",
    "camera_leftRGBD/depth/points": "camera_leftRGBD_depth_points.json",
}

def make_qos(reliability):
    return QoSProfile(reliability=reliability, history=HistoryPolicy.KEEP_LAST, depth=3, durability=DurabilityPolicy.VOLATILE)

def valid_points(arr):
    """원점 근처(무효 깊이 픽셀)를 제거한 Nx3 배열을 돌려준다."""
    if arr.size == 0:
        return arr.reshape(0, 3)
    return arr[np.linalg.norm(arr, axis=1) > MIN_RANGE]

class Once(Node):
    def __init__(self, topic, fname):
        super().__init__('capture_once_' + topic.replace('/', '_'))
        self.topic = topic
        self.fname = fname
        self.done = False
        self.create_subscription(PointCloud2, '/' + topic, self.cb, make_qos(ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PointCloud2, '/' + topic, self.cb, make_qos(ReliabilityPolicy.RELIABLE))

    def cb(self, msg):
        if self.done:
            return
        pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        arr = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        raw_n = arr.shape[0]
        arr = valid_points(arr)
        if arr.shape[0] == 0:
            # 빈/전량 무효 프레임. 직전 정상 스냅샷을 덮지 말고 다음 메시지를 기다린다.
            print(f"{self.topic}: empty frame ({raw_n} raw), waiting for next", flush=True)
            return
        self.done = True
        if arr.shape[0] > MAX_POINTS:
            idx = np.random.choice(arr.shape[0], MAX_POINTS, replace=False)
            arr = arr[idx]
        rounded = [round(float(v), 4) for v in arr.flatten()]
        tmp_path = os.path.join(OUT_DIR, self.fname + ".tmp")
        final_path = os.path.join(OUT_DIR, self.fname)
        with open(tmp_path, "w") as f:
            json.dump(rounded, f, separators=(",", ":"))
        os.replace(tmp_path, final_path)
        print(f"{self.topic}: {arr.shape[0]} points written (raw {raw_n}) -> {final_path}", flush=True)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    nodes = [Once(t, f) for t, f in TOPICS.items()]
    t0 = time.time()
    while time.time() - t0 < TIMEOUT_SEC:
        for n in nodes:
            rclpy.spin_once(n, timeout_sec=0.2)
        if all(n.done for n in nodes):
            break
    ok = 0
    for n in nodes:
        status = "OK" if n.done else "TIMEOUT"
        print(f"RESULT {n.topic}: {status}", flush=True)
        if n.done:
            ok += 1
        n.destroy_node()
    rclpy.shutdown()
    print(f"DONE {ok}/{len(nodes)}", flush=True)


if __name__ == "__main__":
    main()
