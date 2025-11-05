#!/usr/bin/env python3
#
# Copyright 2025 Space Station OS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class LiveOrbitPlotter(Node):
    def __init__(self):
        super().__init__('live_orbit_plotter')

        self.sub = self.create_subscription(
            Vector3,
            '/gnc/pos_eci',
            self.eci_callback,
            10
        )

        self.eci_x = []
        self.eci_y = []
        self.eci_z = []

        # Init plot
        plt.ion()
        self.fig = plt.figure(figsize=(10, 8))
        self.ax3d = self.fig.add_subplot(121, projection='3d')
        self.ax2d = self.fig.add_subplot(122)
        self.ax3d.set_box_aspect([1, 1, 1])
        self.setup_earth()

        # Timer for updating
        self.timer = self.create_timer(0.5, self.update_plot)

    def setup_earth(self):
        # Plot Earth once
        earth_radius = 6371e3
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = earth_radius * np.outer(np.cos(u), np.sin(v))
        y = earth_radius * np.outer(np.sin(u), np.sin(v))
        z = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))
        self.ax3d.plot_surface(
            x,
            y,
            z,
            color='blue',
            alpha=0.3,
            edgecolor='none')

    def eci_callback(self, msg: Vector3):
        if not all(np.isfinite([msg.x, msg.y, msg.z])):
            self.get_logger().warn('Received invalid ECI position; skipping')
            return

        # Optional: skip duplicate or zero points
        if len(self.eci_x) > 0:
            last_x = self.eci_x[-1]
            last_y = self.eci_y[-1]
            last_z = self.eci_z[-1]
            if (msg.x == last_x and msg.y == last_y and msg.z == last_z):
                return

        self.eci_x.append(msg.x)
        self.eci_y.append(msg.y)
        self.eci_z.append(msg.z)

        # Limit array size for performance (e.g. last 500 points)
        max_points = 500
        if len(self.eci_x) > max_points:
            self.eci_x = self.eci_x[-max_points:]
            self.eci_y = self.eci_y[-max_points:]
            self.eci_z = self.eci_z[-max_points:]

    def update_plot(self):
        if len(self.eci_x) < 2:
            return

        self.ax3d.clear()
        self.ax2d.clear()

        # Re-plot Earth
        self.setup_earth()

        # Plot orbit trace
        x, y, z = np.array(
            self.eci_x), np.array(
            self.eci_y), np.array(
            self.eci_z)
        self.ax3d.plot(x, y, z, color='red', linewidth=1.5)
        self.ax3d.set_xlabel('ECI X [m]')
        self.ax3d.set_ylabel('ECI Y [m]')
        self.ax3d.set_zlabel('ECI Z [m]')
        self.ax3d.set_title('3D Orbit')

        # Plot motion radius
        r = np.sqrt(x**2 + y**2 + z**2)
        self.ax2d.plot(r, color='green')
        self.ax2d.set_title('Motion Radius')
        self.ax2d.set_xlabel('Simulation Step')
        self.ax2d.set_ylabel('Radius [m]')

        self.fig.subplots_adjust(left=0.05, right=0.95, wspace=0.3)

        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = LiveOrbitPlotter()
    try:
        plt.show(block=False)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
