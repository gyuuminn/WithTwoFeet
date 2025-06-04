import serial, rclpy, struct
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8

class MotorBridge(Node):
    HEADER   = 0xAA
    CMD_MODE = 0x10      # 모드 패킷: [AA 10 <mode>]
    def __init__(self):
        super().__init__("motor_bridge")

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

        # ─── 상태 변수 ───
        self.drive_enabled = True    # latched 토픽이 즉시 덮어씀

        # On/off subscriber
        self.bool_sub = self.create_subscription(Bool, 'robot_enable', self.en_cb, 10)

        # Mode subscriber
        self.mode_sub = self.create_subscription(UInt8, 'cmd_mode', self.mode_cb, 10)

    # Enable/Disable 토픽
    def en_cb(self, msg: Bool):
        self.drive_enabled = msg.data
        self.get_logger().info(f'Enable={self.drive_enabled}')
        if not self.drive_enabled:
            self.send_mode(5)       # 안전 정지

    # 모드 토픽
    def mode_cb(self, msg: UInt8):
        if not self.drive_enabled:
            return                   # 차단
        self.send_mode(int(msg.data))

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
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()