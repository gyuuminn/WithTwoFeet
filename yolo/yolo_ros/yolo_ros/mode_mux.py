#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

class ModeMux(Node):
    def __init__(self):
        super().__init__('mode_mux')
        self.manual = 0
        self.auto   = 0
        self.pub = self.create_publisher(UInt8, 'cmd_mode', 10)
        
        self.create_subscription(UInt8, 'cmd_mode_manual', self.man_cb, 10)

        self.create_subscription(UInt8, 'cmd_mode_auto'  , self.auto_cb, 10)

    def man_cb(self, msg: UInt8):
        self.manual = msg.data
        self.publish()

    def auto_cb(self, msg: UInt8):
        self.auto = msg.data
        self.publish()

    def publish(self):
        chosen = self.auto if self.manual == 0 else self.manual
        self.pub.publish(UInt8(data=chosen))

def main():
    rclpy.init()
    rclpy.spin(ModeMux())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
