#!/usr/bin/env python3
import rclpy
import json
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, PoseStamped
from smarc_msgs.msg import DVL, DVLBeam
from sensor_msgs.msg import Imu
from collections import deque
class DVLConverterNode(Node):
    def __init__(self):
        super().__init__('dvl_converter')

        self.declare_parameter('beam_angle_deg', 22.5)
        self.declare_parameter('frame_id', 'dvl_link')
        beam_deg = self.get_parameter('beam_angle_deg').value
        self.beam_angle = np.deg2rad(beam_deg)
        self.frame_id  = self.get_parameter('frame_id').value
        self.angulat_vel = np.array([])
        self.phi = np.deg2rad(np.array([225., 135., 45., 315.]))
        self.Imu_Base = np.array([0.573, 0.0, -0.063])

        self.hampel = HampelFilter(window_size=9, n_sigmas=3)
        # subscriber to raw JSON
        self.sub = self.create_subscription(
            String,
            'sam/core/dvl_raw_output',
            self.cb_raw,
            10
        )

        self.pub = self.create_publisher(
            DVL,
            '/sam/core/dvl_new',
            10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/sam/core/imu',self.imu_cb,10
        )
        self.get_logger().info('DVLConverterNode started.')

    def cb_raw(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON parse error: {e}')
            return

        if data.get('velocity_valid', False):
            vx, vy, vz = data['vx'], data['vy'], data['vz']
        else:
            beams = [b for b in data.get('transducers', []) if b.get('beam_valid')]
            if len(beams) < 3:
                self.get_logger().warn(f'{len(beams)} valid beams (<3); skipping')
                return
            ids    = [b['id'] for b in beams]
            b_vels = np.array([b['velocity'] for b in beams])

            sa = np.sin(self.beam_angle)
            ca = np.cos(self.beam_angle)
            H = np.vstack([
                [sa * np.cos(self.phi[i]),
                 sa * np.sin(self.phi[i]),
                 ca]
                for i in ids
            ]) 

            V, *_ = np.linalg.lstsq(H, b_vels, rcond=None)
            vx, vy, vz = V

        now = self.get_clock().now().to_msg()
        out = DVL()
        out.header.stamp = now
        out.header.frame_id = self.frame_id

        out.velocity = Vector3(x=vx, y=vy, z=vz)


        out.altitude = float(data.get('altitude', 0.0))

        out.beams = []
        for b in data.get('transducers', []):
            beam = DVLBeam()
            beam.range = float(b.get('distance', -1.0))
            beam.range_covariance = 0.0 if b.get('beam_valid', False) else -1.0
            beam.velocity = float(b.get('velocity', 0.0))
            beam.velocity_covariance = 0.0 if b.get('beam_valid', False) else -1.0

            beam.pose = PoseStamped()
            beam.pose.header.stamp = now
            beam.pose.header.frame_id = self.frame_id

            out.beams.append(beam)
        velocity = np.array([out.velocity.x,
                             out.velocity.y,
                             out.velocity.z])
        cross_product = np.cross(self.angulat_vel, self.Imu_Base)
        baselink_vel = velocity - cross_product

        baselink_vel = velocity - cross_product 
        vel_filtered = self.hampel.filter(baselink_vel)  
        out.velocity.x, out.velocity.y, out.velocity.z = vel_filtered   

        self.pub.publish(out)

    def imu_cb(self,msg):
        self.angulat_vel = np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])


class HampelFilter:
    def __init__(self, window_size=7, n_sigmas=3):
        self.window = deque(maxlen=window_size)
        self.k = n_sigmas

    def filter(self, x_new):
        self.window.append(x_new)
        w = np.array(self.window)
        m = np.median(w, axis=0)
        mad = np.median(np.abs(w - m), axis=0)
        # avoid divide‐by‐zero
        thresh = self.k * (mad if np.all(mad>0) else np.ones_like(mad)*1e-6)
        # for each component, replace if too far
        out = np.where(np.abs(x_new - m) > thresh, m, x_new)
        return out

def main(args=None):
    rclpy.init(args=args)
    node = DVLConverterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
