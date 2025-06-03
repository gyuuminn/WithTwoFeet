import serial, rclpy, struct
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, UInt8
from rclpy.qos import QoSProfile

class JoyTwistTeleop(Node):
    def __init__(self):
        super().__init__("joy_twist_teleop")
        qos_profile = QoSProfile(depth=10)

         # ────── 시리얼 설정 파라미터 ──────
        self.declare_parameter("serial_port", "/tmp/ttyV0")
        self.declare_parameter("baud_rate",   115200)
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value

        #시리얼 포트 관련
        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            self.get_logger().info(f"Opened serial {port} @ {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.ser = None

        self.retry_timer = self.create_timer(1.0, self.try_open_serial)
        #self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01) #Serial 포트 open
        # On/off subscriber
        self.bool_sub = self.create_subscription(Bool, 'robot_enable', self.bool_cb, 10)

        # Mode subscriber
        self.mode_sub = self.create_subscription(UInt8, 'cmd_mode', self.mode_cb, 10)

        # Joy subscriber
        self.joy_sub = self.create_subscription(Joy,'joy', self.joy_cb, qos_profile)

        #self.cmd_sub = self.create_subscription(Twist,'init_cmd_vel',self,auto_cb, qos_profile )
        # 버튼·축 매핑 파라미터
        self.declare_parameter("btn_cross", 1)
        self.declare_parameter("btn_circle", 2)
        self.declare_parameter("btn_triangle", 3)
        self.declare_parameter("btn_square", 0)
        self.declare_parameter("axis_speed", 1)
        self.declare_parameter("axis_turn", 0)
        self.declare_parameter("baxis_speed", 1) #십자
        self.declare_parameter("baxis_turn", 0)

    def joy_cb(self, msg: Joy):
        # 파라미터 읽기
        btn_cr  = self.get_parameter("btn_cross").value
        btn_cir  = self.get_parameter("btn_circle").value
        btn_tr = self.get_parameter("btn_triangle").value
        btn_sq = self.get_parameter("btn_square").value
        ax_spd = self.get_parameter("axis_speed").value
        ax_turn = self.get_parameter("axis_turn").value
        bax_spd = self.get_parameter("baxis_speed").value # 십자
        bax_turn = self.get_parameter("baxis_turn").value

        # 1) 아날로그 스틱 값
        speed_raw = msg.axes[self.ax_spd]     # -1.0 ~ 1.0
        turn_raw  = msg.axes[self.ax_turn]

        # 2) D-패드 값 (눌렸을 때만  ±1.0 / 안 눌리면 0.0)
        dpad_speed = msg.axes[self.bax_spd]
        dpad_turn  = msg.axes[self.bax_turn]

        # 3) D-패드가 입력을 ‘덮어쓰기’하도록 우선순위 부여
        if abs(dpad_speed) > 0.1:     # 0.1 은 데드존
            speed_raw = dpad_speed
        if abs(dpad_turn) > 0.1:
            turn_raw = dpad_turn

        # 조이스틱 축 값 → 속도/조향값
        speed = int(msg.axes[speed_raw]  * 100)   # -100 ~ 100
        turn  = int(msg.axes[turn_raw] * 100)   # -100 ~ 100

        # 기본 모드
        mode = 0

        # mode 결정 (우선순위: speed > turn)
        if speed ==0 and turn == 0: mode = 5
        else: 
            if abs(speed) > abs(turn):
                if speed > 0: mode = 6
                elif speed < 0: mode = 7
            elif abs(turn) > abs(speed):
                if turn > 0: mode = 8
                elif turn < 0: mode = 9
        self.send_mode(mode)
        
        # 출력 및 실행
        self.get_logger().info(f"Speed: {speed}, Turn: {turn}, Mode: {mode}")

        # # 버튼 처리
        # # 전진, 후진, 좌, 우, Enable/disable, camera shutter
        # if msg.buttons[btn_cr]:
        #     self.get_logger().info("Cross")
        #     self.send_mode(11)              # 전진모드
        # elif msg.buttons[btn_cir]:
        #     self.get_logger().info("Circle")
        #     self.send_mode(12)              # 후진모드
        # elif msg.buttons[btn_tr]:
        #     self.get_logger().info("Triangle")
        #     self.send_mode(13)              # 정지
        # elif msg.buttons[btn_sq]:
        #     self.get_logger().info("Square")
        #     self.send_mode(14) 

    #def auto_cb(self):

    # ───────── 시리얼 전송 헬퍼 ─────────
    def send_mode(self, mode: int):
        if self.ser is None or not self.ser.is_open:
            return
        frame = struct.pack("<BBB", 0xAA, 0x10, mode)
        self.ser.write(frame)
        self.get_logger().debug(f"TX → {frame.hex(' ')}")   # 헥사로 확인

    def try_open_serial(self):
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        if self.ser is not None and self.ser.is_open:
            self.retry_timer.cancel()           # 이미 열렸으면 타이머 종료
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            self.get_logger().info("Serial re-opened!")
        except serial.SerialException:
            # 아직도 실패 → 다음 타이머 tick 때 또 시도
            pass

def main():
    rclpy.init()
    node = JoyTwistTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()