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
REFRESH_SEC = 4.0
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
    return QoSProfile(
        reliability=reliability,
        history=HistoryPolicy.KEEP_LAST,
        depth=3,
        durability=DurabilityPolicy.VOLATILE,
    )

def valid_points(arr):
    """원점 근처(무효 깊이 픽셀)를 제거한 Nx3 배열을 돌려준다."""
    if arr.size == 0:
        return arr.reshape(0, 3)
    return arr[np.linalg.norm(arr, axis=1) > MIN_RANGE]

class Loop(Node):
    def __init__(self):
        super().__init__('ranger_calib_snapshot_loop')
        self.latest = {}
        for topic in TOPICS:
            self.create_subscription(PointCloud2, '/' + topic, self._make_cb(topic), make_qos(ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(PointCloud2, '/' + topic, self._make_cb(topic), make_qos(ReliabilityPolicy.RELIABLE))
        self.timer = self.create_timer(REFRESH_SEC, self.write_snapshots)
        self.get_logger().info(f"watching {len(TOPICS)} topics, writing every {REFRESH_SEC}s to {OUT_DIR}")

    def _make_cb(self, topic):
        def cb(msg):
            self.latest[topic] = msg
        return cb

    def write_snapshots(self):
        t0 = time.time()
        n_written = 0
        for topic, fname in TOPICS.items():
            msg = self.latest.get(topic)
            if msg is None:
                continue
            pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            arr = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
            arr = valid_points(arr)
            if arr.shape[0] == 0:
                continue
            if arr.shape[0] > MAX_POINTS:
                idx = np.random.choice(arr.shape[0], MAX_POINTS, replace=False)
                arr = arr[idx]
            rounded = [round(float(v), 4) for v in arr.flatten()]
            tmp_path = os.path.join(OUT_DIR, fname + ".tmp")
            final_path = os.path.join(OUT_DIR, fname)
            with open(tmp_path, "w") as f:
                json.dump(rounded, f, separators=(",", ":"))
            os.replace(tmp_path, final_path)
            n_written += 1
        self.get_logger().info(f"snapshot: wrote {n_written}/{len(TOPICS)} files in {time.time()-t0:.2f}s")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    node = Loop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
