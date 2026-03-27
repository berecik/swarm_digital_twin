#!/usr/bin/env python3
"""
Wind Perturbation ROS Node - Digital Twin
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>

Publishes wind perturbation forces to a ROS 2 topic for Gazebo integration.
Uses the same WindField model as the standalone simulation.

Usage:
    ros2 run gazebo wind_node.py --ros-args -p wind_speed:=5.0
"""

import sys
import os

# Add simulation directory to path so we can import wind_model
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'simulation'))

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Vector3Stamped, WrenchStamped, PoseStamped
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("WARNING: rclpy not available. Wind node requires ROS 2.")

import numpy as np
from wind_model import WindField
from drone_physics import AeroCoefficients, Atmosphere


class WindNode(Node if ROS_AVAILABLE else object):
    """ROS 2 node that publishes wind velocity and force."""

    def __init__(self):
        super().__init__('wind_node')

        # Parameters
        self.declare_parameter('wind_speed', 0.0)
        self.declare_parameter('wind_direction_x', 1.0)
        self.declare_parameter('wind_direction_y', 0.0)
        self.declare_parameter('wind_direction_z', 0.0)
        self.declare_parameter('gust_intensity', 0.0)
        self.declare_parameter('turbulence_type', 'constant')
        self.declare_parameter('publish_rate', 50.0)

        speed = self.get_parameter('wind_speed').value
        dir_x = self.get_parameter('wind_direction_x').value
        dir_y = self.get_parameter('wind_direction_y').value
        dir_z = self.get_parameter('wind_direction_z').value
        gust = self.get_parameter('gust_intensity').value
        turb = self.get_parameter('turbulence_type').value
        rate = self.get_parameter('publish_rate').value

        self.declare_parameter('base_altitude_msl', 4500.0)

        self.base_alt_msl = self.get_parameter('base_altitude_msl').value

        self.wind = WindField(
            wind_speed=speed,
            wind_direction=np.array([dir_x, dir_y, dir_z]),
            gust_intensity=gust,
            turbulence_type=turb,
        )
        self.aero = AeroCoefficients()
        self.atmo = Atmosphere(altitude_msl=self.base_alt_msl)
        self.t0 = self.get_clock().now()

        # Drone position (updated by subscription)
        self.drone_pos = np.zeros(3)

        # Publishers
        self.vel_pub = self.create_publisher(
            Vector3Stamped, '/wind/velocity', 10,
        )
        self.force_pub = self.create_publisher(
            WrenchStamped, '/wind/force', 10,
        )

        # Subscribe to drone position for altitude-dependent wind
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            10,
        )

        dt = 1.0 / rate
        self.timer = self.create_timer(dt, self.publish_wind)
        self.get_logger().info(
            f'Wind node started: speed={speed} m/s, type={turb}, '
            f'base_alt={self.base_alt_msl}m MSL'
        )

    def _pose_callback(self, msg: 'PoseStamped'):
        """Update drone position from MAVROS local position."""
        p = msg.pose.position
        self.drone_pos = np.array([p.x, p.y, p.z])

    def _get_altitude_density(self) -> float:
        """Compute air density at current drone altitude using ISA model.

        Combines the base MSL altitude with the drone's local Z (AGL)
        to get the true MSL altitude, then computes ISA density.
        """
        alt_msl = self.base_alt_msl + self.drone_pos[2]
        return Atmosphere(altitude_msl=alt_msl).rho

    def publish_wind(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        pos = self.drone_pos

        # Velocity
        vel = self.wind.get_wind_velocity(t, pos)
        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = now.to_msg()
        vel_msg.header.frame_id = 'world'
        vel_msg.vector.x = float(vel[0])
        vel_msg.vector.y = float(vel[1])
        vel_msg.vector.z = float(vel[2])
        self.vel_pub.publish(vel_msg)

        # Force — use altitude-dependent air density
        rho = self._get_altitude_density()
        force = self.wind.get_force(t, pos, self.aero, rho)
        force_msg = WrenchStamped()
        force_msg.header.stamp = now.to_msg()
        force_msg.header.frame_id = 'world'
        force_msg.wrench.force.x = float(force[0])
        force_msg.wrench.force.y = float(force[1])
        force_msg.wrench.force.z = float(force[2])
        self.force_pub.publish(force_msg)


def main(args=None):
    if not ROS_AVAILABLE:
        print("ERROR: ROS 2 (rclpy) is required to run the wind node.")
        print("Install ROS 2 or use the standalone wind_model.py directly.")
        sys.exit(1)

    rclpy.init(args=args)
    node = WindNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
