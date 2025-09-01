# 파일 경로: ~/sim_ws/src/f1tenth_obstacle_avoid/f1tenth_obstacle_avoid/aeb_node.py

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool # <-- 변경/추가된 부분

class AEBNode(Node):
    def __init__(self):
        super().__init__('aeb_node')
        # 런치 파일에서 이 값을 1.0 ~ 1.2 정도로 설정하는 것을 강력히 추천합니다.
        self.declare_parameter('front_angle_deg', 30.0)
        self.declare_parameter('stop_dist', 0.7)

        self.front_angle = math.radians(self.get_parameter('front_angle_deg').value)
        self.stop_dist   = float(self.get_parameter('stop_dist').value)
        self.last_scan   = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb_scan, qos)
        # '/drive' 발행자 대신 비상 정지 신호 발행자를 생성합니다.
        self.pub_emergency = self.create_publisher(Bool, '/aeb_emergency_stop', 10) # <-- 변경/추가된 부분

        # AEB는 빠르게 감시 (100 Hz)
        self.timer = self.create_timer(1.0/100.0, self.on_timer)

    def cb_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_timer(self):
        if self.last_scan is None:
            return

        msg = self.last_scan
        ranges = list(msg.ranges)
        # 유효화: 0/NaN/Inf -> range_max로 교체
        for i, r in enumerate(ranges):
            if (not math.isfinite(r)) or (r <= 0.01):
                ranges[i] = msg.range_max

        n = len(ranges)
        idx_zero = int(round((0.0 - msg.angle_min)/msg.angle_increment))
        half_span = int(round(self.front_angle / msg.angle_increment / 2))
        i0 = max(0, idx_zero - half_span)
        i1 = min(n-1, idx_zero + half_span)

        d_min = min(ranges[i0:i1+1]) if i1 >= i0 else float('inf')

        stop_signal = Bool() # <-- 변경/추가된 부분
        if d_min < self.stop_dist:
            # 위험할 때 True 신호를 보냅니다.
            stop_signal.data = True # <-- 변경/추가된 부분
        else:
            # 안전할 때는 False 신호를 보냅니다.
            stop_signal.data = False # <-- 변경/추가된 부분
        
        self.pub_emergency.publish(stop_signal) # <-- 변경/추가된 부분

def main():
    rclpy.init()
    rclpy.spin(AEBNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
