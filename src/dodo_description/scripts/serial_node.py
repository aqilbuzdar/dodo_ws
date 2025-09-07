#!/usr/bin/env python3
import rclpy, time, glob
from rclpy.node import Node
import serial
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def pick_port():
    # Prefer stable udev symlink if you created it
    if os_path_exists('/dev/arduino'):
        return '/dev/arduino'
    acms = sorted(glob.glob('/dev/ttyACM*'))
    if not acms:
        raise RuntimeError("No Arduino serial port found (/dev/arduino or /dev/ttyACM*)")
    return acms[0]

def os_path_exists(p):
    try:
        import os
        return os.path.exists(p)
    except Exception:
        return False

class HoverSerialNode(Node):
    def __init__(self):
        super().__init__('hover_serial_node')

        port = pick_port()
        self.get_logger().info(f"✅ Using serial port: {port}")
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        time.sleep(2.0)  # UNO reset wait

        # Publisher (Arduino → ROS)
        self.pub_fb = self.create_publisher(String, 'hover_feedback', 10)

        # Subscribers (ROS → Arduino)
        self.sub_str  = self.create_subscription(String, 'cmd_hover', self.cmd_str_cb, 10)
        self.sub_twst = self.create_subscription(Twist,  'cmd_vel',   self.cmd_twist_cb, 10)

        # Gains (tune as needed)
        self.linear_gain  = 300   # linear.x multiplier → speed
        self.angular_gain = 200   # angular.z multiplier → turn

        self.last_spd  = 0
        self.last_turn = 0

        # timers
        self.tx_timer = self.create_timer(0.10, self.send_keepalive)  # 10 Hz
        self.rx_timer = self.create_timer(0.05, self.read_serial)

    def write_line(self, s: str):
        data = (s + "\r\n").encode('utf-8')  # CRLF
        self.ser.write(data)
        self.ser.flush()

    # /cmd_hover (manual)
    def cmd_str_cb(self, msg: String):
        line = msg.data.strip()
        self.write_line(line)
        self.get_logger().info(f"➡️ Sent (String): {line}")

    # /cmd_vel (teleop)
    def cmd_twist_cb(self, msg: Twist):
        spd  = int(msg.linear.x  * self.linear_gain)
        turn = int(msg.angular.z * self.angular_gain)
        spd  = clamp(spd,  -1000, 1000)
        turn = clamp(turn, -1000, 1000)
        self.last_spd, self.last_turn = spd, turn
        cmd = f"speed {spd} turn {turn}"
        self.write_line(cmd)
        self.get_logger().info(f"➡️ Sent (Twist): {cmd}")

    # keepalive
    def send_keepalive(self):
        self.write_line(f"speed {self.last_spd} turn {self.last_turn}")

    # Arduino → ROS feedback
    def read_serial(self):
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                msg = String(); msg.data = line
                self.pub_fb.publish(msg)
                if line.startswith("RX:") or line.startswith("ACK") or ',' in line:
                    self.get_logger().info(f"⬅️ {line}")
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HoverSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
