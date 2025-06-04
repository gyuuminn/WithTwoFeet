import serial, rclpy, struct
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, UInt8
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class JoyTwist(Node):
    HEADER = 0xAA
    def __init__(self):
        super().__init__("joy_twist")
        # QoS: cmd_mode 는 10-개 버퍼, enable 은 latch(최근 1개 저장)
        qos_cmd  = QoSProfile(depth=10)
        qos_latch = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Mode Publisher
        self.mode_pub = self.create_publisher(UInt8, 'cmd_mode_manual', qos_cmd)

        # Enable Publisher
        self.en_pub   = self.create_publisher(Bool,  'robot_enable', qos_latch)

        # Joy subscriber
        self.joy_sub = self.create_subscription(Joy,'joy', self.joy_cb, qos_cmd)

        # 버튼·축 매핑 파라미터
        self.declare_parameter("btn_cross", 1)
        self.declare_parameter("btn_circle", 2)
        self.declare_parameter("btn_triangle", 3)
        self.declare_parameter("btn_square", 0)
        self.declare_parameter("axis_speed", 1)
        self.declare_parameter("axis_turn", 0)
        self.declare_parameter("baxis_speed", 1) #십자
        self.declare_parameter("baxis_turn", 0)

        self.drive_enabled = True
        self.en_pub.publish(Bool(data=True))       # 처음엔 ON 상태로 래치

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

        # -------- Enable 토글 --------
        # 버튼 edge 검출
        if msg.buttons[btn_cr] == 1 and hasattr(self, '_prev_toggle') and self._prev_toggle == 0:
            self.drive_enabled = not self.drive_enabled
            self.en_pub.publish(Bool(data=self.drive_enabled))
            self.get_logger().info(f'DRIVE {"ENABLED" if self.drive_enabled else "DISABLED"}')
        self._prev_toggle = msg.buttons[btn_cr]

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

        self.mode_pub.publish(UInt8(data=mode))
        
        # 출력 및 실행
        self.get_logger().info(f"Speed: {speed}, Turn: {turn}, Mode: {mode}")


def main():
    rclpy.init()
    node = JoyTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()