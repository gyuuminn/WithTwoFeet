import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool

class ModeMux(Node):
    def __init__(self):
        super().__init__('mode_mux')
        self.manual = 0
        self.auto   = 0
        # # ─── 상태 변수 ───
        self.drive_enabled = True    # latched 토픽이 즉시 덮어씀
        #Integrated mode publisher
        self.pub = self.create_publisher(UInt8, 'cmd_mode', 10)
        
        #Mode Manual subscriber
        self.create_subscription(UInt8, 'cmd_mode_manual', self.man_cb, 10)
        #Mode auto subscriber
        self.create_subscription(UInt8, 'cmd_mode_auto', self.auto_cb, 10)

        # On/off subscriber
        self.bool_sub = self.create_subscription(Bool, 'robot_enable', self.en_cb, 10)

    def man_cb(self, msg: UInt8):
        self.manual = msg.data
        self.publish()

    def auto_cb(self, msg: UInt8):
        self.auto = msg.data
        self.publish()

    # Enable/Disable 토픽
    def en_cb(self, msg: Bool):
        self.drive_enabled = msg.data
        self.get_logger().info(f'Enable={self.drive_enabled}')
        self.publish()

    def publish(self):
        if not self.drive_enabled:
            if self.manual != 0:
                chosen = self.manual
                self.get_logger().info(f"[DISABLED] Manual mode used: {chosen}")
            else:
                chosen = 5
                self.get_logger().info(f"[DISABLED] Manual=0 → Force STOP (mode 5)")
        else:
            if self.manual == 5:
                chosen = self.auto
                self.get_logger().info(f"[ENABLED] AUTO OVERRIDE: Manual=5 → Auto={self.auto}")
            else:
                chosen = self.auto if self.manual == 0 else self.manual
                self.get_logger().info(f"[ENABLED] Selected Mode: {chosen} (Manual: {self.manual}, Auto: {self.auto})")

        self.pub.publish(UInt8(data=chosen))

def main():
    rclpy.init()
    node = ModeMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
