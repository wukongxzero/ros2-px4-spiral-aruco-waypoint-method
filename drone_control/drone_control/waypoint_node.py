#!/usr/bin/env python3

import asyncio
import math
import rclpy
from rclpy.node import Node
from mavsdk import System, OffboardError
from mavsdk.offboard import PositionNedYaw


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

    async def run(self):
        # Connect to the drone via MAVSDK
        drone = System()
        await drone.connect(system_address="udp://:14540")

        self.get_logger().info("Waiting for vehicle to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("-- Connected to vehicle!")
                break

        # Wait for GPS fix
        self.get_logger().info("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("-- Global position state is good enough for flying.")
                break

        # Arm the drone
        self.get_logger().info("-- Arming")
        try:
            await drone.action.arm()
        except OffboardError as error:
            self.get_logger().error(f"Failed to arm the drone with error code: {error._result.result}")
            return

        # Set offboard mode
        self.get_logger().info("-- Setting offboard mode")
        try:
            await drone.offboard.set_velocity_ned(PositionNedYaw(0.0, 0.0, 0.0, math.radians(0.0)))
            await drone.offboard.start()
        except OffboardError as error:
            self.get_logger().error(f"Failed to set offboard mode with error code: {error._result.result}")
            return

        # Take off
        self.get_logger().info("-- Taking off")
        try:
            await drone.offboard.set_velocity_ned(PositionNedYaw(0.0, 0.0, -2.0, math.radians(0.0)))
            await asyncio.sleep(5) # wait 5 seconds
        except OffboardError as error:
            self.get_logger().error(f"Failed to take off with error code: {error._result.result}")
            return

        # Fly to a location
        self.get_logger().info("-- Flying to the destination")
        try:
            await drone.offboard.set_position_ned(PositionNedYaw(47.397606, 8.543060, -22.0, math.radians(0.0)))
            await asyncio.sleep(30) # wait 30 seconds
        except OffboardError as error:
            self.get_logger().error(f"Failed to fly to the destination with error code: {error._result.result}")
            return

        # Disarm the drone
        self.get_logger().info("-- Disarming")
        try:
            await drone.action.land()
        except OffboardError as error:
            self.get_logger().error(f"Failed to disarm the drone with error code: {error._result.result}")
            return


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    asyncio.get_event_loop().run_until_complete(offboard_control.run())

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
