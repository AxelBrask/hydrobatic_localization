#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from smarc_msgs.msg import PercentStamped

class DualSineGenerator(Node):
    def __init__(self):
        super().__init__('dual_sine_generator')

        self.declare_parameter('vbs_hold_time', 20.0)   # seconds
        self.declare_parameter('vbs_hold_value', 100.0)   # percent
        self.declare_parameter('vbs_frequency', 0.02)   
        self.declare_parameter('vbs_publish_rate', 50.0)   
        self.declare_parameter('lcg_frequency', 0.15)   
        self.declare_parameter('lcg_publish_rate', 20.0)   

        self.hold_time    = self.get_parameter('vbs_hold_time').value
        self.hold_value   = self.get_parameter('vbs_hold_value').value
        self.vbs_freq     = self.get_parameter('vbs_frequency').value
        self.vbs_rate_hz  = self.get_parameter('vbs_publish_rate').value
        self.lcg_freq     = self.get_parameter('lcg_frequency').value
        self.lcg_rate_hz  = self.get_parameter('lcg_publish_rate').value

        self.pub_vbs = self.create_publisher(
            PercentStamped, 'sam_auv_v1/core/vbs_cmd',  10)
        self.pub_lcg = self.create_publisher(
            PercentStamped, 'sam_auv_v1/core/lcg_cmd', 10)

        now = self.get_clock().now().nanoseconds * 1e-9
        self._start_time = now
        self._t0_lcg     = now

        self.create_timer(1.0/self.vbs_rate_hz, self._timer_vbs)
        self.create_timer(1.0/self.lcg_rate_hz, self._timer_lcg)

    def _timer_vbs(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._start_time

        msg = PercentStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if elapsed < self.hold_time:
            msg.value = self.hold_value
        else:
            t = elapsed - self.hold_time
            msg.value = 70 + 25.0 * math.sin(2.0 * math.pi * self.vbs_freq * t)

        self.pub_vbs.publish(msg)

    def _timer_lcg(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self._t0_lcg
        frac = 0.5 + 0.5 * math.sin(2.0 * math.pi * self.lcg_freq * t)

        msg = PercentStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.value = frac * 100.0
        self.pub_lcg.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DualSineGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()