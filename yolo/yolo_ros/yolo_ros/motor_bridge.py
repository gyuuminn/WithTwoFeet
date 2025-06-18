import serial, rclpy, struct
from rclpy.node import Node
from std_msgs.msg import UInt8

class MotorBridge(Node):
    HEADER   = 0xAA
    CMD_MODE = 0x10      # 모드 패킷: [AA 10 <mode>]
    def __init__(self):
        super().__init__("motor_bridge")

        # ────── 시리얼 설정 파라미터 ──────
        self.declare_parameter("serial_port", "/dev/ttyUSB0") #/dev/ttyUSB0 /tmp/ttyV1
        self.declare_parameter("baud_rate",   115200)
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value

        #시리얼 포트 관련
        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            self.serial_connected = True
            self.get_logger().info(f"Opened serial {port} @ {baud}")
            self.send_mode(1)
            self.get_logger().info("Sent initial mode=1 after opening serial")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.ser = None

        self.serial_connected = self.ser is not None and self.ser.is_open
        self.retry_timer = self.create_timer(1.0, self.try_open_serial)
        #self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01) #Serial 포트 open

        # Integrated Mode subscriber
        self.mode_sub = self.create_subscription(UInt8, 'cmd_mode', self.mode_cb, 10)

    # 모드 토픽
    def mode_cb(self, msg: UInt8):   
        self.send_mode(int(msg.data))

    # ───────── 시리얼 전송 헬퍼 ─────────
    def send_mode(self, mode: int):
        if self.ser is None or not self.ser.is_open:
            return
        # mode는 5,6,7,8,9 같은 수치라고 가정
        # 문자열 "6" → ASCII 바이트 0x36로 보내면 펌웨어의  if (RxTemp == '6') 와 매칭.
        msg = str(mode).encode('ascii')   # ex: mode=6 → b"6"
        frame   = bytes([self.HEADER, self.CMD_MODE]) + msg
        self.ser.write(frame)

        # 디버그 로그 (aa 10 36)
        self.get_logger().info(f"TX → {frame.hex(' ')}")
        

        # # 디버그
        # self.get_logger().info(f"TX ASCII → {msg!r}")
        
        # frame = struct.pack("<BBB", 0xAA, 0x10, mode)
        # self.ser.write(frame)
        # self.get_logger().debug(f"TX → {frame.hex(' ')}")   # 헥사로 확인

    def try_open_serial(self):
        if self.serial_connected:
            return
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            self.serial_connected = True
            self.get_logger().info("Serial re-opened!")
            self.send_mode(1)
            self.get_logger().info("Sent mode=1 after re-opening serial")
        except serial.SerialException:
            # 아직도 실패 → 다음 타이머 tick 때 또 시도
            self.serial_connected = False

def main():
    rclpy.init()
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()