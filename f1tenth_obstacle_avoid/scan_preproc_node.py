# scan_preproc_node.py
import rclpy, math, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

def moving_average(x: np.ndarray, k: int):
    if k <= 1: return x
    cumsum = np.cumsum(np.insert(x, 0, 0.0))
    ma = (cumsum[k:] - cumsum[:-k]) / k
    pad = k//2
    return np.pad(ma, (pad, len(x)-len(ma)-pad), mode='edge')

class ScanPreproc(Node):
    def __init__(self):
        super().__init__('scan_preproc')
        self.declare_parameter('ma_k', 5)            # 이동평균 창
        self.declare_parameter('median_k', 3)        # 간단한 미디언 필터(홀수)
        self.declare_parameter('downsample', 1)      # 빔 다운샘플링(1은 그대로)
        self.k  = int(self.get_parameter('ma_k').value)
        self.mk = int(self.get_parameter('median_k').value)
        self.ds = max(1, int(self.get_parameter('downsample').value))

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, qos)
        self.pub = self.create_publisher(LaserScan, '/scan_clean', 10)

        self.last_t = None

    def cb(self, msg: LaserScan):
        # 45 Hz로 쓰로틀링
        now = self.get_clock().now()
        if self.last_t and (now - self.last_t).nanoseconds < int(1e9/45):
            return
        self.last_t = now

        arr = np.array(msg.ranges, dtype=float)
        # 0/NaN/Inf → range_max
        invalid = (~np.isfinite(arr)) | (arr <= 0.01)
        arr[invalid] = msg.range_max

        # 간단 미디언(옵션)
        if self.mk >= 3 and self.mk % 2 == 1:
            pad = self.mk//2
            padded = np.pad(arr, (pad, pad), mode='edge')
            arr = np.array([np.median(padded[i:i+self.mk]) for i in range(len(arr))], dtype=float)

        # 이동평균(옵션)
        if self.k > 1:
            arr = moving_average(arr, self.k)

        # 다운샘플(옵션)
        if self.ds > 1:
            arr = arr[::self.ds]
            angle_inc = msg.angle_increment * self.ds
        else:
            angle_inc = msg.angle_increment

        # 퍼블리시
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_min + angle_inc*(len(arr)-1)
        out.angle_increment = angle_inc
        out.time_increment = msg.time_increment * self.ds
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, 0.01)
        out.range_max = msg.range_max
        out.ranges = arr.tolist()
        out.intensities = []  # 필요시 유지
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ScanPreproc())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
