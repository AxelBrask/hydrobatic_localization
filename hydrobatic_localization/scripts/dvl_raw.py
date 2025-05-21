#!/usr/bin/env python3
import rclpy
import json
import numpy as np

from collections import deque
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, PoseStamped
from smarc_msgs.msg import DVL, DVLBeam
from sensor_msgs.msg import Imu

class DVLConverterNode(Node):
    def __init__(self):
        super().__init__('dvl_converter')

        # parameters
        self.declare_parameter('beam_angle_deg', 22.5)
        self.declare_parameter('frame_id', 'dvl_link')
        beam_deg = self.get_parameter('beam_angle_deg').value
        self.beam_angle = np.deg2rad(beam_deg)
        self.frame_id  = self.get_parameter('frame_id').value
        self.angulat_vel = np.zeros(3)
        self.phi = np.deg2rad(np.array([225., 135., 45., 315.]))
        self.Imu_Base = np.array([0.573, 0.0, -0.063])

        self.window_size = 4
        self.vel_buffer = deque(maxlen=self.window_size)
        

        self.sub = self.create_subscription(
            String, 'sam/core/dvl_raw_output', self.cb_raw, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/sam/core/imu', self.imu_cb, 10
        )

        self.pub = self.create_publisher(DVL, '/sam/core/dvl_new', 10)
        self.raw_pub = self.create_publisher(
            Vector3, '/sam/core/dvl_baselink_raw', 10
        )

        self.get_logger().info('DVLConverterNode started.')

    def cb_raw(self, msg: String):
        # parse JSON
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
            sa, ca = np.sin(self.beam_angle), np.cos(self.beam_angle)
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

        vel_body = np.array([vx, vy, vz])
        cross_prod = np.cross(self.angulat_vel, self.Imu_Base)
        baselink_vel = vel_body - cross_prod

        raw_msg = Vector3(
            x=baselink_vel[0],
            y=baselink_vel[1],
            z=baselink_vel[2]
        )
        self.raw_pub.publish(raw_msg)

        self.vel_buffer.append(baselink_vel)
        if len(self.vel_buffer) == self.window_size:
            med = np.median(np.stack(self.vel_buffer), axis=0)
        else:
            med = baselink_vel

        out.velocity = Vector3(x=med[0], y=med[1], z=med[2])
        self.pub.publish(out)

    def imu_cb(self, msg: Imu):
        self.angulat_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

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
