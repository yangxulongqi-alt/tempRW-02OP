#!/usr/bin/env python3
"""
键盘控制Joy节点 - SSH兼容版本
通过监测键盘方向键发送Joy消息
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import termios
import tty
import select
import atexit


class KeyboardJoyNode(Node):
    def __init__(self):
        super().__init__('keyboard_joy_node')
        
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        
        # 按键状态 - 使用字典跟踪持续按下的键
        self.axes = [0.0] * 6
        self.buttons = [0] * 12
        self.key_states = {'w': False, 's': False, 'a': False, 'd': False}
        self.last_key_time = self.get_clock().now()
        self.key_timeout = 0.15  # 150ms超时释放
        
        # 保存并设置终端
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        atexit.register(self.restore_terminal)
        
        # 设置终端为raw模式
        new_settings = termios.tcgetattr(self.fd)
        new_settings[3] = new_settings[3] & ~(termios.ICANON | termios.ECHO)
        new_settings[6][termios.VMIN] = 0
        new_settings[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, new_settings)
        
        self.get_logger().info('键盘Joy节点已启动 (SSH兼容)')
        self.get_logger().info('控制: W/S=前后, A/D=左右, Space=按钮0, Q=退出')

    def restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        """非阻塞读取键盘"""
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # 转义序列
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    ch += sys.stdin.read(1)
                    if select.select([sys.stdin], [], [], 0.01)[0]:
                        ch += sys.stdin.read(2)
            return ch.lower() if len(ch) == 1 else ch
        return None

    def timer_callback(self):
        key = self.get_key()
        now = self.get_clock().now()
        
        if key:
            self.last_key_time = now
            
            # 映射方向键到WASD
            key_map = {'\x1b[A': 'w', '\x1b[B': 's', '\x1b[C': 'd', '\x1b[D': 'a'}
            if key in key_map:
                key = key_map[key]
            
            if key == 'q':
                self.get_logger().info('退出...')
                raise SystemExit
            elif key == ' ':
                self.buttons[0] = 1
            elif key in self.key_states:
                self.key_states[key] = True
        
        # 超时释放所有键
        elapsed = (now - self.last_key_time).nanoseconds / 1e9
        if elapsed > self.key_timeout:
            for k in self.key_states:
                self.key_states[k] = False
            self.buttons = [0] * 12
        
        # 计算axes
        self.axes[1] = (1.0 if self.key_states['w'] else 0.0) - (1.0 if self.key_states['s'] else 0.0)
        self.axes[0] = (1.0 if self.key_states['d'] else 0.0) - (1.0 if self.key_states['a'] else 0.0)
        
        # 发布
        joy_msg = Joy()
        joy_msg.header.stamp = now.to_msg()
        joy_msg.axes = self.axes
        joy_msg.buttons = self.buttons
        self.joy_pub.publish(joy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJoyNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()