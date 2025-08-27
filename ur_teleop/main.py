#!/usr/bin/env python3


import argparse
import time
import yaml
import numpy as np
import multiprocessing as mp
from typing import Optional, Tuple
from multiprocessing import Queue, Event
from devices.realsense_t265 import T265Tracker
from devices.magiclaw import MagiClaw
from devices.ur5e import UR5eController
from scipy.spatial.transform import Slerp, Rotation as R

def mirror_pose(pos: np.ndarray, rot: R) -> Tuple[np.ndarray, R]:
    """
    Mirror the pose by x-z plane.
    
    Args:
        pos (np.ndarray): Position vector.
        rot (R): Rotation object.
        mirror_mat (np.ndarray): Mirror matrix to apply to the position and rotation.
    
    Returns:
        Tuple[np.ndarray, R]: Mirrored position and rotation.
    """
    
    # Apply the mirror matrix to the position
    mirrored_pos = np.array([-pos[0], -pos[1], pos[2]])

    # Apply the mirror matrix to the rotation
    mirrored_rot = R.as_euler(rot, 'xyz', degrees=True)
    mirrored_rot[1] = -mirrored_rot[1]  # Mirror the y-axis rotation
    mirrored_rot[2] = -mirrored_rot[2]  # Mirror the z-axis rotation
    mirrored_rot = R.from_euler('xyz', mirrored_rot, degrees=True)
    
    return mirrored_pos, mirrored_rot
    
    


def magiclaw_process(
    host: str, data_queue: Queue, stop_event: Event, loop_rate=50
) -> None:
    """
    Process to handle the MagiClaw subscriber, retrieving pose data and putting it into a queue.

    Args:
        data_queue (Queue): Queue to store the pose data.
        stop_event (Event): Event to signal stopping the process.
        loop_rate (int): The rate at which to retrieve pose data (default is 50 Hz).
    """

    print("Starting MagiClaw subscriber process...")
    # Initialize the MagiClaw subscriber
    magiclaw_subscriber = MagiClaw(host)

    # Set the loop time based on the loop rate
    loop_time = 1.0 / loop_rate

    try:
        while not stop_event.is_set():
            start_time = time.time()

            delta_pos, delta_rot = magiclaw_subscriber.get_delta_pose()
            
            # Send the filtered delta pose to the queue
            data_queue.put((delta_pos.tolist(), delta_rot.as_quat()))

            # Sleep to maintain the loop rate
            elapsed_time = time.time() - start_time
            if elapsed_time < loop_time:
                time.sleep(loop_time - elapsed_time)

        print("Stopping MagiClaw subscriber...")
        # Stop the MagiClaw subscriber when the process is stopped
        magiclaw_subscriber.stop()
    except KeyboardInterrupt:
        print("Stopping MagiClaw subscriber...")

        # Set the stop event to signal other processes to stop
        if not stop_event.is_set():
            stop_event.set()

        # Ensure the MagiClaw subscriber is stopped properly
        magiclaw_subscriber.stop()


def t265_process(data_queue: Queue, stop_event: Event, loop_rate=50) -> None:
    """
    Process to handle the T265 tracker, retrieving pose data and putting it into a queue.

    Args:
        data_queue (Queue): Queue to store the pose data.
        stop_event (Event): Event to signal stopping the process.
        loop_rate (int): The rate at which to retrieve pose data (default is 30 Hz).
    """

    print("Starting T265 tracker process...")
    # Initialize the T265 tracker
    t265 = T265Tracker()

    # Set the loop time based on the loop rate
    loop_time = 1.0 / loop_rate

    try:
        while not stop_event.is_set():
            start_time = time.time()

            delta_pos, delta_rot = t265.get_delta_pose()

            # Send the filtered delta pose to the queue
            data_queue.put((delta_pos.tolist(), delta_rot.as_quat()))

            # Sleep to maintain the loop rate
            elapsed_time = time.time() - start_time
            if elapsed_time < loop_time:
                time.sleep(loop_time - elapsed_time)

        print("Stopping T265 tracker...")
        # Stop the T265 tracker when the process is stopped
        t265.stop()
    except KeyboardInterrupt:
        print("Stopping T265 tracker...")

        # Set the stop event to signal other processes to stop
        if not stop_event.is_set():
            stop_event.set()

        # Ensure the T265 tracker is stopped properly
        t265.stop()


def ur_control_process(
    robot_host,
    data_queue: Queue,
    stop_event: Event,
    init_joint: Optional[np.ndarray] = None,
    mirror: bool = False,
    loop_rate=100,
) -> None:
    """
    Process to handle the UR5e controller, retrieving pose data from the queue and controlling the robot.

    Args:
        robot_host (str): The host address of the UR5e robot.
        data_queue (Queue): Queue to retrieve pose data.
        stop_event (Event): Event to signal stopping the process.
        loop_rate (int): The rate at which to control the robot (default is 50 Hz).
    """

    print("Starting UR5e controller process...")
    # Initialize the UR5e controller
    ur5e = UR5eController(robot_host, init_joint)

    # Initialize last delta pose and rotation
    last_delta_pos = np.zeros(3)
    last_delta_rot = R.from_quat([0, 0, 0, 1])

    # Set the control loop time
    loop_time = 1.0 / loop_rate

    try:
        while not stop_event.is_set():
            start_time = time.time()
            # Non-blocking check for data in the queue
            while not data_queue.empty():
                delta_pos, delta_rot_quat = data_queue.get()
                last_delta_pos = np.array(delta_pos)
                last_delta_rot = R.from_quat(delta_rot_quat)
                
                if mirror:
                    # Mirror the pose if required
                    last_delta_pos, last_delta_rot = mirror_pose(last_delta_pos, last_delta_rot)
            print(f"Delta Position: {last_delta_pos}, Delta Rotation: {last_delta_rot.as_quat()}")
            target_pose = ur5e.get_target_pose(last_delta_pos, last_delta_rot)
            ur5e.servo_to(target_pose)

            # Sleep to maintain the loop rate
            elapsed_time = time.time() - start_time
            if elapsed_time < loop_time:
                time.sleep(loop_time - elapsed_time)

        print("Stopping UR5e controller...")
        # Stop the UR5e controller when the process is stopped
        ur5e.stop()
    except KeyboardInterrupt:
        print("Stopping UR5e controller...")

        # Set the stop event to signal other processes to stop
        if not stop_event.is_set():
            stop_event.set()

        # Ensure the UR5e controller is stopped properly
        ur5e.stop()


def teleoperation(
    device: str,
    robot_host: str,
    magiclaw_host: str,
    robot_init_joint: Optional[np.ndarray] = None,
    mirror: bool = False,
) -> None:
    """
    Main function to start the teleoperation processes for the T265 tracker and UR5e controller.

    Args:
        robot_host (str): The host address of the UR5e robot.
    """

    print("Starting teleoperation...")

    # Create a multiprocessing queue and event for inter-process communication
    mp.set_start_method("spawn", force=True)
    data_queue = Queue()
    stop_event = Event()

    # Initialize the initial joint configuration if provided
    if robot_init_joint is not None:
        print(f"Initial joint configuration: {robot_init_joint}")
    ur_control_proc = mp.Process(
        target=ur_control_process,
        args=(robot_host, data_queue, stop_event, robot_init_joint, mirror),
    )
    ur_control_proc.start()

    time.sleep(1)  # Give some time for the UR5e controller to initialize

    if device == "t265":
        print("Using T265 tracker for teleoperation.")
        
        device_proc = mp.Process(target=t265_process, args=(data_queue, stop_event))
        device_proc.start()
    elif device == "magiclaw":
        print("Using MagiClaw for teleoperation.")
        
        device_proc = mp.Process(
            target=magiclaw_process, args=(magiclaw_host, data_queue, stop_event)
        )
        device_proc.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping processes...")
        stop_event.set()
    finally:
        device_proc.join()
        ur_control_proc.join()
        print("Processes stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UR Teleoperation")
    parser.add_argument(
        "--device",
        type=str,
        default="magiclaw",
        choices=["magiclaw", "t265"],
        help="Device to use for teleoperation (default: magiclaw)",
    )
    args = parser.parse_args()    
    
    with open("config/address.yaml", "r") as f:
        config = yaml.load(f.read(), Loader=yaml.Loader)
        robot_host = config["robot_host"]
        magiclaw_host = config["magiclaw_host"]
        print(f"Robot host: {robot_host}")

    robot_init_joint = np.array([-90.0, -60.0, -140.0, 20.0, 90.0, 0.0]) * np.pi / 180.0


    teleoperation(args.device, robot_host, magiclaw_host, robot_init_joint=robot_init_joint, mirror=True)
