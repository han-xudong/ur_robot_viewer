#!/usr/bin/env python3

"""
This script streams the robot state over ZeroMQ for visualization in rerun.
Press "CTRL+C" to exit the program.
"""

import argparse
import time
import rtde_receive
import yaml
import zmq
from ur_robot_viewer.protobuf import robot_pb2


class RobotPublisher:
    def __init__(self) -> None:
        """Initialize the ZeroMQ publisher."""

        # Initialize the ZeroMQ context and publisher socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")

        # Initialize the robot protobuf message
        self.robot = robot_pb2.Robot()

    def publish_robot_state(self, robot_state: dict) -> None:
        """Publish the robot state data over ZeroMQ."""

        # Update the robot protobuf message with the robot state
        self.robot.joint_angles[:] = robot_state["joint_angles"]
        self.robot.joint_velocities[:] = robot_state["joint_velocities"]
        self.robot.tcp_pose[:] = robot_state["tcp_pose"]
        self.robot.tcp_velocity[:] = robot_state["tcp_velocity"]
        self.robot.tcp_force[:] = robot_state["tcp_force"]

        # Send the serialized robot state over the socket
        self.socket.send(self.robot.SerializeToString())


def read_robot_data(rtde_r) -> dict:
    """Read the robot state from RTDE interface."""

    joint_angles = rtde_r.getActualQ()
    joint_velocities = rtde_r.getActualQd()
    tcp_pose = rtde_r.getActualTCPPose()
    tcp_velocity = rtde_r.getActualTCPSpeed()
    tcp_force = rtde_r.getActualTCPForce()

    return {
        "joint_angles": joint_angles,
        "joint_velocities": joint_velocities,
        "tcp_pose": tcp_pose,
        "tcp_velocity": tcp_velocity,
        "tcp_force": tcp_force,
    }


def main(fps: int = 100) -> None:
    """Main function to stream robot state."""

    # Set up the RTDE connection
    with open("config/address.yaml", "r") as f:
        robot_ip = yaml.load(f.read(), Loader=yaml.Loader)["robot_ip"]
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

    # Initialize the publisher
    robot_publisher = RobotPublisher()
    # Stream the robot data
    start_time = time.time()
    loop_time = time.time()
    count = 0
    try:
        while True:
            loop_time = time.time()

            # Read the robot data
            robot_state = read_robot_data(rtde_r)
            robot_publisher.publish_robot_state(robot_state)

            # Update the frame count and print FPS
            count += 1
            if count >= fps:
                print(f"FPS: {count / (time.time() - start_time):.2f}")
                start_time = time.time()
                count = 0

            # Control the frame rate
            time.sleep(max(1.0 / fps - (time.time() - loop_time), 0))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    # Command line argument parsing
    parser = argparse.ArgumentParser(
        description="Stream robot state over ZeroMQ for visualization."
    )
    parser.add_argument(
        "--fps", type=int, default=100, help="Frames per second for streaming."
    )
    args = parser.parse_args()

    # Run the main function
    main(args.fps)
