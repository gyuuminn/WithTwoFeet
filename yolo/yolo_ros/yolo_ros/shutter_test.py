#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess

class ShutterBridge(Node):
    def __init__(self):
        super().__init__('shutter_bridge')
        self.declare_parameter("btn_square", 3) # 0이 x
        self.declare_parameter("target_device", "192.168.4.171:5555")  # 스마트폰 ADB ID
        self.prev_pressed = False
        
        self.create_subscription(Joy, '/joy', self.cb, 10)

        
    def cb(self, msg: Joy):
        # msg.buttons[index] 가 1일 때만 한 번 트리거
        btn_sq = self.get_parameter("btn_square").value
        target = self.get_parameter("target_device").value
        pressed = msg.buttons[btn_sq] == 1

        #self.get_logger().info(f"Button index: {btn_sq}, value: {msg.buttons[btn_sq]}")
        #self.get_logger().info(f"Pressed: {pressed}, Prev pressed: {self.prev_pressed}")

        if pressed and not self.prev_pressed:
            try:
                subprocess.run(["adb", "-s", target, "shell", "input", "keyevent", "25"], check=True)
                self.get_logger().info("Sent ADB shutter (volume down) command")
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"ADB command failed: {e}")
        self.prev_pressed = pressed

def main():
    rclpy.init()
    node = ShutterBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
