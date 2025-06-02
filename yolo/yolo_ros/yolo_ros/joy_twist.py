import serial, rclpy, struct
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
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
            raise SystemExit
        
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

    def joy_cb(self, msg: Joy):
        # 파라미터 읽기
        btn_cr  = self.get_parameter("btn_cross").value
        btn_cir  = self.get_parameter("btn_circle").value
        btn_tr = self.get_parameter("btn_triangle").value
        btn_sq = self.get_parameter("btn_square").value
        ax_spd = self.get_parameter("axis_speed").value
        ax_turn = self.get_parameter("axis_turn").value

        # 조이스틱 축 값 → 속도/조향값
        speed = int(msg.axes[ax_spd]  * 100)   # -100 ~ 100
        turn  = int(msg.axes[ax_turn] * 100)   # -100 ~ 100

        # 기본 모드
        mode = 0

        # mode 결정 (우선순위: speed > turn)
        if abs(speed) > abs(turn):
            if speed > 0:
                mode = 6
            elif speed < 0:
                mode = 7
        elif abs(turn) > abs(speed):
            if turn > 0:
                mode = 8
            elif turn < 0:
                mode = 9

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
        self.ser.write(struct.pack("<B", 0xAA))   # 헤더
        self.ser.write(struct.pack("<B", 0x10))   # CMD_MODE
        self.ser.write(struct.pack("<B", mode))


def main():
    rclpy.init()
    node = JoyTwistTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()