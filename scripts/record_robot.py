#!/usr/bin/env python3

"""
This script is to record the robot state into hdf5 file, and visualize it in rerun.
Press "r" to record the robot state, and "s" to stop recording.
Press "ESC" to exit the program.
"""

import os
import time
import yaml
import zmq
import h5py
import multiprocessing as mp
import numpy as np
from pynput import keyboard
import rtde_receive
from ur_robot_viewer.protobuf import robot_pb2

# Global variables
global stop_flag
stop_flag = False

# Global variables for recording
global is_recording, recorded_data, start_time
is_recording = False
recorded_data = None


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


# Create a function to record the robot state
def record_robot_state(rtde_r, publisher: RobotPublisher):
    global stop_flag, is_recording, recorded_data

    count = 0
    start = time.time()
    while not stop_flag:
        # Get the robot state
        joint_angles = rtde_r.getActualQ()
        joint_velocities = rtde_r.getActualQd()
        tcp_pose = rtde_r.getActualTCPPose()
        tcp_velocity = rtde_r.getActualTCPSpeed()
        tcp_force = rtde_r.getActualTCPForce()

        # Publish the robot data
        publisher.publish_robot_state(
            {
                "joint_angles": joint_angles,
                "joint_velocities": joint_velocities,
                "tcp_pose": tcp_pose,
                "tcp_velocity": tcp_velocity,
                "tcp_force": tcp_force,
            }
        )

        # Record the robot state if recording is enabled
        if is_recording:
            # Append the data to the recorded_data list
            if recorded_data is not None:
                timestamp = time.time() - start_time
                joint_angles = np.array(joint_angles).flatten()
                joint_velocities = np.array(joint_velocities).flatten()
                tcp_pose = np.array(tcp_pose).flatten()
                tcp_velocity = np.array(tcp_velocity).flatten()
                tcp_force = np.array(tcp_force).flatten()

                recorded_data.append(
                    {
                        "timestamp": timestamp,
                        "joint_angles": joint_angles,
                        "joint_velocities": joint_velocities,
                        "tcp_pose": tcp_pose,
                        "tcp_velocity": tcp_velocity,
                        "tcp_force": tcp_force,
                    }
                )

        time.sleep(0.01)

        count += 1
        if count % 30 == 0:
            print("Frame rate: ", count / (time.time() - start), "Hz")
            count = 0
            start = time.time()


# Create a function to start recording
def start_recording():
    global is_recording, recorded_data, start_time
    if is_recording:
        print("Recording is already started.")
        return
    is_recording = True
    recorded_data = []
    start_time = time.time()

    print("Recording started...")


# Create a function to stop recording
def stop_recording():
    global is_recording
    if not is_recording:
        print("Recording is already stopped.")
        return
    is_recording = False

    print("Recording stopped.")
    print(f"Recorded {len(recorded_data)} data points.")
    save_recorded_data(f"data/{time.strftime('%Y%m%d_%H%M%S')}.h5")


# Create a function to save the recorded data to an HDF5 file
def save_recorded_data(file_path):
    if not os.path.exists("data"):
        os.makedirs("data")

    global recorded_data
    with h5py.File(file_path, "w") as f:
        f.create_dataset(
            "joint_angles", data=[data["joint_angles"] for data in recorded_data]
        )
        f.create_dataset(
            "joint_velocities",
            data=[data["joint_velocities"] for data in recorded_data],
        )
        f.create_dataset("tcp_pose", data=[data["tcp_pose"] for data in recorded_data])
        f.create_dataset(
            "tcp_velocity", data=[data["tcp_velocity"] for data in recorded_data]
        )
        f.create_dataset(
            "tcp_force", data=[data["tcp_force"] for data in recorded_data]
        )
        f.create_dataset(
            "timestamps", data=[data["timestamp"] for data in recorded_data]
        )
    print(f"Recorded data saved to {file_path}")


# Create a function to handle keyboard events
def on_press(key):
    global stop_flag, is_recording

    try:
        if key.char == "r":
            start_recording()
        elif key.char == "s":
            stop_recording()
        elif key == keyboard.Key.esc:
            stop_flag = True
            return False
    except AttributeError:
        pass


def run():
    global stop_flag
    stop_flag = False

    # Load the robot address from the YAML file
    with open("../config/address.yaml", "r") as f:
        address = yaml.load(f.read(), Loader=yaml.Loader)

    # Initialize the RTDE control and receive interfaces
    rtde_r = rtde_receive.RTDEReceiveInterface(address["robot_ip"])

    # Initialize the publisher
    publisher = RobotPublisher(address["robot_state"])

    # Start the recording process in a separate process
    record_process = mp.Process(target=record_robot_state, args=(rtde_r, publisher))
    record_process.daemon = True
    record_process.start()

    # Start listening for keyboard events
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

    while not stop_flag:
        time.sleep(0.01)

    rtde_r.disconnect()

    print("Program terminated.")


if __name__ == "__main__":
    run()
