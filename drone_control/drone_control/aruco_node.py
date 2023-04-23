#!/usr/bin/env python3

import asyncio
import rclpy
from rclpy.node import Node
from mavsdk import System, aruco


class ArucoLandNode(Node):
    def __init__(self):
        super().__init__("aruco_land_node")
        self.create_timer(0.1, self.run)

    async def run(self):
        drone = System()
        await drone.connect(system_address="udp://:14540")

        self.get_logger().info("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("-- Connected to drone!")
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

        await asyncio.sleep(1)
        # To fly drone 20m above the ground plane
        flying_alt = absolute_altitude + 20.0
        # goto_location() takes Absolute MSL altitude
        await drone.action.goto_location(47.397606, 8.543060, flying_alt, 0)

        # Start the ArUco detection
        await drone.camera.start_video_streaming()
        self.get_logger().info("Starting ArUco detection...")

        async for detection in drone.camera.video_stream():
            # Check if an ArUco marker was detected
            if detection.aruco_detection:
                self.get_logger().info(f"ArUco marker detected with ID {detection.aruco_detection.id}")
                # Land on the detected ArUco marker
                await drone.action.land_on_aruco(detection.aruco_detection.id, 0.2)
                break

        self.get_logger().info("Stopping ArUco detection...")
        await drone.camera.stop_video_streaming()

        self.get_logger().info("Landing...")
        await drone.action.land()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
