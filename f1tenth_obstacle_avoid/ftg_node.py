# 파일 경로: ~/sim_ws/src/f1tenth_obstacle_avoid/f1tenth_obstacle_avoid/ftg_node.py

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool # <-- 변경/추가된 부분

def moving_average(x: np.ndarray, k: int):
    if k <= 1: return x
    cumsum = np.cumsum(np.insert(x, 0, 0.0))
    ma = (cumsum[k:] - cumsum[:-k]) / k
    pad_l = k//2
    pad_r = len(x) - len(ma) - pad_l
    return np.pad(ma, (pad_l, pad_r), mode='edge')

class FTGNode(Node):
    def __init__(self):
        super().__init__('ftg_node')
        # 기본 FTG 파라미터
        self.declare_parameter('ma_k', 5)
        self.declare_parameter('bubble_radius', 0.6)
        self.declare_parameter('min_gap_th', 0.8)
        self.declare_parameter('max_steer_deg', 25.0)
        self.declare_parameter('v_min', 0.8)
        self.declare_parameter('v_max', 3.0)
        self.declare_parameter('v_scale_dist', 3.0)
        
        # 보강 파라미터 (코너링 안정성을 위해 기본값 상향 조정)
        self.declare_parameter('inflate_m', 0.30)           # 차폭 여유 (0.25 -> 0.30)
        self.declare_parameter('disparity_th', 0.4)          # 거리 급변 임계
        self.declare_parameter('disparity_pad_deg', 10.0)    # 모서리 주변 막기 각도 (8.0 -> 12.0)
        self.declare_parameter('angle_brake_gain', 0.8)      # 큰 조향일수록 감속 (0.6 -> 0.8)
        self.declare_parameter('steer_smooth', 0.4)          # 조향 스무딩
        self.declare_parameter('side_clear_m', 0.35)         # 한쪽 최소 여유 목표
        self.declare_parameter('side_k', 0.15)               # 반발 강도 (0.12 -> 0.15)

        self._read_params()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb_scan, qos)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # AEB 비상 정지 신호를 구독합니다.
        self.sub_aeb = self.create_subscription(Bool, '/aeb_emergency_stop', self.cb_aeb, 10) # <-- 변경/추가된 부분

        self.timer = self.create_timer(1.0/45.0, self.on_timer)
        self.add_on_set_parameters_callback(self._on_param)

        self.last_scan = None
        self.steer_prev = 0.0
        self.aeb_active = False # <-- 변경/추가된 부분

    def _read_params(self):
        gp = self.get_parameter
        # ... (이 함수 내용은 변경 없음) ...
        self.k         = int(gp('ma_k').value)
        self.bub_r     = float(gp('bubble_radius').value)
        self.gap_th    = float(gp('min_gap_th').value)
        self.max_delta = math.radians(float(gp('max_steer_deg').value))
        self.v_min     = float(gp('v_min').value)
        self.v_max     = float(gp('v_max').value)
        self.v_scale   = float(gp('v_scale_dist').value)
        self.inflate   = float(gp('inflate_m').value)
        self.disp_th   = float(gp('disparity_th').value)
        self.disp_pad  = math.radians(float(gp('disparity_pad_deg').value))
        self.ang_gain  = float(gp('angle_brake_gain').value)
        self.smooth    = float(gp('steer_smooth').value)
        self.side_clear= float(gp('side_clear_m').value)
        self.side_k    = float(gp('side_k').value)

    def _on_param(self, params):
        self._read_params()
        return SetParametersResult(successful=True)
    
    # AEB 신호를 받았을 때 호출될 콜백 함수
    def cb_aeb(self, msg: Bool): # <-- 변경/추가된 부분
        self.aeb_active = msg.data

    def cb_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_timer(self):
        # AEB가 활성화되면, 모든 계산을 중단하고 즉시 정지 명령을 내립니다.
        if self.aeb_active: # <-- 변경/추가된 부분
            cmd = AckermannDriveStamped()
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = self.steer_prev # 이전 조향각 유지를 통해 안정성 확보
            self.pub.publish(cmd)
            return

        if self.last_scan is None:
            return

        # ... (이하 on_timer의 나머지 로직은 기존과 동일) ...
        msg = self.last_scan
        ranges = np.array(msg.ranges, dtype=float)
        invalid = (~np.isfinite(ranges)) | (ranges <= 0.01)
        ranges[invalid] = msg.range_max
        ranges = moving_average(ranges, self.k)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        min_idx = int(np.argmin(ranges))
        end_x = ranges * np.cos(angles)
        end_y = ranges * np.sin(angles)
        obs_x, obs_y = end_x[min_idx], end_y[min_idx]
        dist_to_min = np.hypot(end_x - obs_x, end_y - obs_y)
        ranges[dist_to_min < self.bub_r] = 0.01
        ranges = np.maximum(ranges - self.inflate, 0.0)
        disp = np.abs(np.diff(ranges))
        edges = np.where(disp > self.disp_th)[0]
        pad = max(1, int(self.disp_pad / msg.angle_increment))
        for e in edges:
            i0 = max(0, e - pad); i1 = min(len(ranges)-1, e + pad)
            ranges[i0:i1+1] = 0.01
        valid = ranges > self.gap_th
        if not np.any(valid):
            target_angle = 0.0
        else:
            best_len = 0; best_s = 0; cur_len = 0; cur_s = 0
            for i, ok in enumerate(list(np.append(valid, False))):
                if ok:
                    if cur_len == 0: cur_s = i
                    cur_len += 1
                else:
                    if cur_len > best_len:
                        best_len = cur_len; best_s = cur_s
                    cur_len = 0
            best_e = best_s + best_len - 1
            seg = ranges[best_s:best_e+1]
            seg_idx = int(np.argmax(seg)) + best_s
            target_angle = float(angles[seg_idx])
        side_win = max(1, int(math.radians(15.0) / msg.angle_increment))
        def sector_min(center_rad):
            idx_c = int(round((center_rad - msg.angle_min)/msg.angle_increment))
            i0 = max(0, idx_c - side_win); i1 = min(len(ranges)-1, idx_c + side_win)
            return float(np.min(ranges[i0:i1+1])) if i1 >= i0 else msg.range_max
        left_min  = sector_min(+math.pi/2)
        right_min = sector_min(-math.pi/2)
        if self.side_clear > 0.0:
            diff_norm = np.clip((left_min - right_min)/self.side_clear, -1.0, 1.0)
            target_angle += self.side_k * diff_norm
        steer = float(np.clip(target_angle, -self.max_delta, self.max_delta))
        alpha = self.smooth
        steer = (1.0 - alpha) * steer + alpha * self.steer_prev
        self.steer_prev = steer
        dmin = float(np.min(ranges[valid])) if np.any(valid) else 0.0
        v_base = float(np.clip(self.v_max * (dmin / self.v_scale), self.v_min, self.v_max))
        v = max(self.v_min, min(self.v_max, v_base * (1.0 - self.ang_gain * abs(steer)/self.max_delta)))
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = steer
        cmd.drive.speed = v
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(FTGNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
