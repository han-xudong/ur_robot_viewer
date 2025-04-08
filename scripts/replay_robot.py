#!/usr/bin/env python3

"""
This script is to replay the recorded robot data on the robot, and visualize it in rerun.
"""

import time
import yaml
import zmq
import h5py
import rtde_control
import rtde_receive
from ur_robot_viewer.protobuf import robot_pb2


class RobotPublisher:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")

        self.robot = robot_pb2.Robot()

    def publish_robot_data(
        self, joint_angles: list, joint_velocities: list, tcp_pose: list
    ):
        self.robot.joint_angles[:] = joint_angles
        self.robot.joint_velocities[:] = joint_velocities
        self.robot.tcp_pose[:] = tcp_pose

        self.socket.send(self.robot.SerializeToString())


# Create a function to replay the robot state
def replay_robot_state(rtde_c, recorded_data):
    count = 0
    start_time = time.time()
    for data in recorded_data:
        timestamp = data["timestamp"]
        joint_angles = data["joint_angles"]

        # Move the robot to the joint angles
        rtde_c.moveJ(joint_angles)

        time.sleep(max(0, timestamp - (time.time() - start_time)))

        count += 1

        if count % 30 == 0:
            print(f"Frame rate: {count / (time.time() - start_time):.2f} Hz")
            count = 0
            start_time = time.time()


# Create a function to load the recorded data from the hdf5 file
def load_recorded_data(file_path):
    recorded_data = []
    with h5py.File(file_path, "r") as f:
        timestamps = f["timestamps"][:]
        joint_angles = f["joint_angles"][:]
        joint_velocities = f["joint_velocities"][:]
        tcp_poses = f["tcp_poses"][:]
        for i in range(len(timestamps)):
            recorded_data.append(
                {
                    "timestamp": timestamps[i],
                    "joint_angles": joint_angles[i],
                    "joint_velocities": joint_velocities[i],
                    "tcp_pose": tcp_poses[i],
                }
            )
    return recorded_data


def run():
    # Load the robot address from the YAML file
    with open("../config/address.yaml", "r") as f:
        address = yaml.load(f.read(), Loader=yaml.Loader)

    # Initialize the RTDE control and receive interfaces
    rtde_c = rtde_control.RTDEControlInterface(address["robot_ip"])
    rtde_r = rtde_receive.RTDEReceiveInterface(address["robot_ip"])

    # Initialize the publisher
    publisher = RobotPublisher(address["robot_state"])

    # Load the recorded data from the hdf5 file
    print("Loading recorded data...")
    recorded_data = load_recorded_data("data/recorded_data.hdf5")

    replay_robot_state(rtde_c, recorded_data)
    print("Replay finished.")
    # Stop the RTDE control and receive interfaces
    rtde_c.disconnect()
    rtde_r.disconnect()

    print("Program terminated.")


if __name__ == "__main__":
    run()
