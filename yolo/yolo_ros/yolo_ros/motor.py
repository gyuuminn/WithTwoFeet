import rclpy, time
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from rclpy.qos import QoSProfile
from std_msgs.msg import UInt8

#yolo 값 읽어온 후 사람과의 Distance에 따라 전진, 후진, 좌우 판단.

class Humanfollower(Node):
    def __init__(self):
        super().__init__('human_follower')
        
        qos_profile = QoSProfile(depth=10)
        
        # ---- 파라미터 (카메라 사양·사람 키) -----------------
        self.declare_parameter('image_width', 640.0)   # 화면 가로(px)
        self.declare_parameter('focal_px', 548.66)    # camera_info로 얻은 초점거리(px)
        self.declare_parameter('real_height_m', 1.70) # 추정 사람 키(m)
        self.declare_parameter('target_dist', 3.0)   # 유지하고 싶은 거리(m)

        # ── 상태 변수 -----------------------------------------------
        self.is_rotating = False                     # 현재 회전 중인가?
        self.rotation_direction = 0  # +1: left, -1: right
        self.last_move_log_time = 0.0
        self.last_rotation_log_time = 0.0  # 마지막 회전 로그 시간
        self.last_no_det_time = 0.0
        self.move_log_interval = 1.5 
        self.rotation_log_interval = 1.5   # 초 단위 (1.5초마다 한 번 출력)

        # Mode publisher
        self.mode_pub = self.create_publisher(UInt8,'cmd_mode', qos_profile)

        # Detection subscriber
        self.create_subscription(Detection2DArray,'/yolo/detections', self.cmd_based_dis,10)

        # 로봇 파라미터
        self.wheel_base   = 0.30   # 바퀴 사이 간격(m)
        self.wheel_radius = 0.05   # 바퀴 반지름(m)

    def cmd_based_dis(self, msg: Detection2DArray):
        if not msg.detections: #아무것도 없을 때
            self.get_logger().warn("No Detection2DArray")
            self.last_no_det_time = time.time()
            return
        # 파라미터 load
        f_px   = self.get_parameter('focal_px').value
        H_real = self.get_parameter('real_height_m').value
        img_w    = self.get_parameter('image_width').value
        center   = img_w * 0.5

        # 경계 값 계산(비율 기반)-히스테리시스 구간 (테스트 해보면서 변경)
        delta_start = img_w * 0.10     # 10 % 밖에서 회전 시작
        delta_stop  = img_w * 0.08     # 8 % 안에서 회전 멈춤
        left_start  = center - delta_start
        right_start = center + delta_start
        left_stop   = center - delta_stop
        right_stop  = center + delta_stop

        best_det = None
        best_num = float("inf")

        for det in msg.detections:
            if not det.results:
                continue

            # 거리 추정 -------------------------------------------------------
            label = det.results[0].hypothesis.class_id.lower()  # "person1"

            if not label.startswith("person"):
                continue
            try:
                num = int(label.replace("person", ""))
            except ValueError:
                continue
            if num < best_num:
                best_num, best_det = num, det
        if best_det is None:
            self.get_logger().warn("No valid person label found")
            return
        # 거리, 방향 계산 -----------------------------------------------------
        h_px  = det.bbox.size_y
        if h_px <= 0:
            return
        
        dist  = (H_real * f_px) / h_px      # 단안(세로) 거리 추정

        # 로그
        #self.get_logger().info(f"[{label}]  bbox_h={h_px:.1f}px  ⇒  {dist:.2f} m")

        tgt_d  = self.get_parameter('target_dist').value

        err_d  = dist - tgt_d             # +면 멀다 / -면 가깝다

        # dead-zone(±10 cm) 안이면 멈춤
        if abs(err_d) < 0.10:
            deadzone_stop = True 
        else:
            deadzone_stop = False
        # ── 화면 X 좌표로 회전 방향 결정 -------------------------
        cx = det.bbox.center.position.x

        if not self.is_rotating:                     # 1. 회전 시작 조건
            if cx < left_start:
                self.is_rotating = True
                self.rotation_direction = +1 
                direction = +1                      # 왼쪽 회전(+, CCW)
                self.get_logger().info(f"왼쪽 회전 시작!!")

            elif cx > right_start:
                self.is_rotating = True
                self.rotation_direction = -1
                direction = -1                      # 오른쪽 회전(-, CW)
                self.get_logger().info(f"오른쪽 회전 시작!!")

            else:
                direction = 0
                self.rotation_direction = 0

        else:                                       # 2. 회전 유지/종료 조건
            if left_stop <= cx <= right_stop:
                self.is_rotating = False
                self.rotation_direction = 0                      # 멈춤
                direction = 0     
                self.get_logger().info("회전 멈춤")

            else:
                direction = +1 if cx < center else -1
                self.rotation_direction = direction
                
                # 1.5초마다 로그 한 번씩만 찍기
                now = time.time()
                if now - self.last_rotation_log_time > self.rotation_log_interval:
                    if self.rotation_direction == +1:
                        self.get_logger().info("왼쪽 회전중...")

                    elif self.rotation_direction == -1:
                        self.get_logger().info("오른쪽 회전중...")
                    self.last_rotation_log_time = now

        # 최종 모드 번호 -------------------------------------
        if self.is_rotating:                       # 회전이 최우선
            mode = 8 if direction > 0 else 9
        else:
            if   deadzone_stop:  mode = 5 
            else:
                if   err_d > 0.10: mode = 6               # 전진
                elif err_d < -0.10: mode = 7               # 후진
                else:            mode = 5             # 정지

            # ── 1.5초마다 이동/정지 로그 --------------------------- ★
            now = time.time()
            if now - self.last_move_log_time > self.move_log_interval:
                if   mode == 6:
                    self.get_logger().info(f"전진 중... 현재 거리{dist:.2f}m")
                elif mode == 7:
                    self.get_logger().info(f"후진 중...현재 거리{dist:.2f}m")
                elif mode == 5:
                    self.get_logger().info(f"목표 거리 도달, 정지!! 현재 거리{dist:.2f}m")
                self.last_move_log_time = now

                self.mode_pub.publish(UInt8(data=mode))
        
            # 디버그 로그 -----------------------------------------------------
            #self.get_logger().info(f"[{label}] d={dist:.2f} m  err={err_d:+.2f} "f"→ v={v:+.2f} m/s, ω={omega:+.2f} rad/s")
            # 한 프레임에 여러 사람이면 number가 가장 낮은 한 명만 제어하고 break

        
        
            
def main():
    rclpy.init()
    node = Humanfollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()