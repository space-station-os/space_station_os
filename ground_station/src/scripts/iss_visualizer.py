#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Or another interactive backend that suits your system
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import os

class SpaceStationVisualizer(Node):
    def __init__(self):
        super().__init__('space_station_visualizer')
        self.subscription = self.create_subscription(
            PoseStamped,
            'iss_position',
            self.pose_callback,
            10
        )

        self.positions = deque(maxlen=300)
        self.earth_radius = 6378.0  # in km
        self.get_logger().info("SpaceStationVisualizer started.")

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.get_logger().info(f"Received station position: ({x:.2f}, {y:.2f}, {z:.2f})")
        self.positions.append((x, y, z))

def main(args=None):
    rclpy.init(args=args)
    viz = SpaceStationVisualizer()

    # Make sure /tmp directory is writable
    os.makedirs("/tmp", exist_ok=True)
    output_file = "/tmp/iss_plot.png"

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 1) Draw Earth (navy blue)
    earth_radius = viz.earth_radius
    u = np.linspace(0, 2*np.pi, 30)
    v = np.linspace(0, np.pi, 15)
    x_sphere = earth_radius * np.outer(np.cos(u), np.sin(v))
    y_sphere = earth_radius * np.outer(np.sin(u), np.sin(v))
    z_sphere = earth_radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.1, color='navy')

    # 2) Show a "historical orbit" circle
    orbit_radius = 6778.0
    theta = np.linspace(0, 2*np.pi, 200)
    hx = orbit_radius * np.cos(theta)
    hy = orbit_radius * np.sin(theta)
    hz = np.zeros_like(theta)
    historical_line, = ax.plot(
        hx, hy, hz,
        color='gold',
        linewidth=2,
        label='Historical Orbit'
    )

    # 3) The real-time path line (teal)
    real_time_line, = ax.plot(
        [], [], [],
        linestyle='--',
        linewidth=2,
        color='teal',
        label='Real-Time Path'
    )

    # 4) Marker for current station position (red)
    station_marker, = ax.plot(
        [], [], [],
        marker='>',
        markersize=8,
        color='red',
        label='Station Position'
    )

    ax.set_xlim([-8000, 8000])
    ax.set_ylim([-8000, 8000])
    ax.set_zlim([-8000, 8000])
    ax.set_xlabel('X (km)')
    ax.set_ylabel('Y (km)')
    ax.set_zlabel('Z (km)')
    ax.set_yticklabels([])
    ax.set_xticklabels([])
    ax.set_zticklabels([])
    ax.set_title("Space Station Trajectory (Real-Time)")
    ax.legend()
    ax.view_init(elev=20, azim=45)

    try:
        while rclpy.ok():
            rclpy.spin_once(viz, timeout_sec=0.01)

            # Update real-time path
            if len(viz.positions) > 0:
                arr = np.array(viz.positions)
                real_time_line.set_xdata(arr[:, 0])
                real_time_line.set_ydata(arr[:, 1])
                real_time_line.set_3d_properties(arr[:, 2])

                # Mark the most recent station position
                last_x, last_y, last_z = arr[-1]
                station_marker.set_xdata([last_x])
                station_marker.set_ydata([last_y])
                station_marker.set_3d_properties([last_z])

            plt.draw()
            plt.pause(0.1)

            # --- NEW: Save the figure to /tmp so React can fetch it ---
            fig.savefig("/tmp/iss_plot.png", dpi=100)
            # ---

    except KeyboardInterrupt:
        pass
    finally:
        viz.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
