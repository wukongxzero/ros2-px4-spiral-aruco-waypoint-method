#!/usr/bin/env python3

import asyncio
import math
import rclpy
from rclpy.node import Node
from mavsdk import System


class SpiralDrone(Node):
    def __init__(self):
        super().__init__('spiral_drone')

        # Start the asyncio loop
        self.get_logger().info("Starting asyncio loop")
        asyncio.ensure_future(self.run())

    async def run(self):
        drone = System()
        await drone.connect(system_address="udp://:14540")

        self.get_logger().info("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info(f"-- Connected to drone!")
                break

        self.get_logger().info("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("-- Global position state is good enough for flying.")
                break

        self.get_logger().info("Fetching amsl altitude at home location....")
        async for terrain_info in drone.telemetry.home():
            absolute_altitude = terrain_info.absolute_altitude_m
            break

        self.get_logger().info("-- Arming")
        await drone.action.arm()

        self.get_logger().info("-- Taking off")
        await drone.action.takeoff()

        # Set the radius, number of rotations, and velocity for the spiral
        radius = 10.0  # meters
        num_rotations = 2.0  # complete rotations
        forward_velocity = 3.0  # meters/second

        # Calculate the tangential velocity and period of the spiral
        tangential_velocity = forward_velocity / (2 * math.pi * radius / num_rotations)
        period = 1 / (num_rotations * tangential_velocity)

        self.get_logger().info(f"-- Spiral Trajectory: radius={radius}, num_rotations={num_rotations}, forward_velocity={forward_velocity}")

        # Run the spiral trajectory for the specified number of rotations
        t = 0.0
        async for _ in drone.telemetry.position_velocity_ned():
            x = radius * math.sin(2 * math.pi * num_rotations * t)
            y = forward_velocity * t
            z = radius * math.cos(2 * math.pi * num_rotations * t)
            vx = tangential_velocity * math.sin(2 * math.pi * num_rotations * t)
            vy = forward_velocity
            vz = tangential_velocity * math.cos(2 * math.pi * num_rotations * t)

            await drone.action.set_actuator_control([0.0, 0.0, 0.0, 0.0, vx, vy, vz, 0.0])
            await drone.action.goto_location(x, y, absolute_altitude + z, 0)
            await asyncio.sleep(period / 100.0)  # 100 Hz loop rate

            t += period / 100.0
            if t >= 1.0:
                break

        self.get_logger().info("-- Landing")
        await drone.action.land()

        while True:
            self.get_logger().info("Staying connected, press Ctrl-C to exit")
            await asyncio.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    spiral_drone = SpiralDrone()

    rclpy.spin(spiral_drone)

    # Shut down ROS when the script is exited
    spiral_drone.destroy_node()


if __name__ == '__main__':
    main()

