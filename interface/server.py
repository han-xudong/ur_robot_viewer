#!/usr/bin/env python3

import argparse
import sys
import time
import yaml
import zmq
import numpy as np
from rerun_loader_urdf import URDFLogger
from scipy.spatial.transform import Rotation
from protobuf import robot_pb2
from common import (
    log_angle_rot,
    link_to_world_transform,
)
import rerun as rr
from rerun.blueprint import (
    Blueprint,
    Horizontal,
    Vertical,
    Spatial3DView,
    TimeSeriesView,
    Tabs,
    SelectionPanel,
    TimePanel,
)


class RobotSubscriber:
    def __init__(self, address: str) -> None:
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect(address)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.joint_angles = np.zeros(6)
        self.joint_velocities = np.zeros(6)
        self.tcp_pose = np.zeros(6)
        self.tcp_velocity = np.zeros(6)
        self.tcp_force = np.zeros(6)

    def receive_message(self):
        robot = robot_pb2.Robot()
        robot.ParseFromString(self.subscriber.recv())

        self.joint_angles = np.array(robot.joint_angles).flatten()
        self.joint_velocities = np.array(robot.joint_velocities).flatten()
        self.tcp_pose = np.array(robot.tcp_pose).flatten()
        self.tcp_velocity = np.array(robot.tcp_velocity).flatten()
        self.tcp_force = np.array(robot.tcp_force).flatten()


class RobotVis:
    def __init__(self):
        self.prev_joint_origins = None

    def log_robot_states(
        self,
        joint_angles: np.ndarray,
        entity_to_transform: dict[str, tuple[np.ndarray, np.ndarray]],
    ):
        joint_origins = []
        for joint_idx, angle in enumerate(joint_angles):
            transform = link_to_world_transform(
                entity_to_transform, joint_angles, joint_idx + 1
            )
            joint_org = (transform @ np.array([0.0, 0.0, 0.0, 1.0]))[:3]
            joint_origins.append(joint_org)

            log_angle_rot(entity_to_transform, joint_idx + 1, angle)

        self.prev_joint_origins = joint_origins

    def log_action_dict(
        self,
        tcp_pose: np.ndarray = np.array([0, 0, 0, 0, 0, 0]),
        tcp_velocity: np.ndarray = np.array([0, 0, 0, 0, 0, 0]),
        tcp_force: np.ndarray = np.array([0, 0, 0, 0, 0, 0]),
        joint_velocities: np.ndarray = np.array([0, 0, 0, 0, 0, 0]),
    ):

        for i, val in enumerate(tcp_pose):
            rr.log(f"/action_dict/tcp_pose/{i}", rr.Scalar(val))
            
        for i, val in enumerate(tcp_velocity):
            rr.log(f"/action_dict/tcp_velocity/{i}", rr.Scalar(val))
            
        for i, val in enumerate(tcp_force):
            rr.log(f"/action_dict/tcp_force/{i}", rr.Scalar(val))

        for i, val in enumerate(joint_velocities):
            rr.log(f"/action_dict/joint_velocity/{i}", rr.Scalar(val))

    def run(
        self,
        entity_to_transform: dict[str, tuple[np.ndarray, np.ndarray]],
    ):
        with open("../config/address.yaml", "r") as f:
            address = yaml.load(f.read(), Loader=yaml.Loader)["robot_state"]
        subscriber = RobotSubscriber(address=address)

        joint_angles = np.array([90, -70, 110, -130, -90, 0]) / 180 * np.pi
        self.log_robot_states(joint_angles, entity_to_transform)
        count = 0
        start_time = time.time()
        while True:
            subscriber.receive_message()
            joint_angles = subscriber.joint_angles
            joint_velocities = subscriber.joint_velocities
            tcp_pose = subscriber.tcp_pose
            tcp_velocity = subscriber.tcp_velocity
            tcp_force = subscriber.tcp_force
            self.log_robot_states(joint_angles, entity_to_transform)
            self.log_action_dict(tcp_pose=tcp_pose, tcp_velocity=tcp_velocity, tcp_force=tcp_force, joint_velocities=joint_velocities)

            count += 1

            if count % 30 == 0:
                print("FPS: %.2f" % (count / (time.time() - start_time)))
                start_time = time.time()
                count = 0

def blueprint():

    return Blueprint(
        Horizontal(
            Spatial3DView(name="3D Scene", origin="/", contents=["/**"]),
            Vertical(
                Tabs(
                    Vertical(
                        *(
                            TimeSeriesView(
                                origin=f"/action_dict/joint_velocity/{i}"
                            )
                            for i in range(6)
                        ),
                        name="joint velocity",
                    ),
                    Vertical(
                        *(
                            TimeSeriesView(origin=f"/action_dict/tcp_pose/{i}")
                            for i in range(6)
                        ),
                        name="tcp pose",
                    ),
                    Vertical(
                        *(
                            TimeSeriesView(origin=f"/action_dict/tcp_velocity/{i}")
                            for i in range(6)
                        ),
                        name="tcp velocity",
                    ),
                    Vertical(
                        *(
                            TimeSeriesView(origin=f"/action_dict/tcp_force/{i}")
                            for i in range(6)
                        ),
                        name="tcp force",
                    ),
                    active_tab=0,
                ),
            ),
            column_shares=[2, 1],
        ),
        SelectionPanel(state="hidden"),
        TimePanel(state="collapsed"),
    )


def rerun_server(
    robot_urdf: str,
):
    print("Starting rerun server...")
    rr.init("Robot Interface", spawn=True)
    rr.send_blueprint(blueprint())
    
    urdf_logger = URDFLogger(filepath=robot_urdf)
    urdf_logger.log()
    
    robot_vis = RobotVis()
    robot_vis.run(urdf_logger.entity_to_transform)


def main(robot) -> None:
    print("\033[91mPRESS ESC TO EXIT, AND STOP RECORDING DATA FIRST!!!\033[0m")

    try:
        rerun_server(robot_urdf=f"ur_description/{robot}.urdf")
    except KeyboardInterrupt:
        print("Exiting...")
        sys.exit(0)


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Robot Interface")
    parser.add_argument(
        "--robot",
        type=str,
        default="ur5e",
        help="Robot type (ur10e, ur5e, etc.)",
    )
    args = parser.parse_args()
    
    main(args.robot)
