import rtde_control
import rtde_receive
import numpy as np
from typing import Optional, Tuple
from scipy.spatial.transform import Rotation as R

class UR5eController:
    def __init__(self, host='192.168.0.100', init_joint: Optional[np.ndarray] = None):
        self.rtde_c = rtde_control.RTDEControlInterface(host)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(host)
        if init_joint is not None:
            self.rtde_c.moveJ(init_joint.tolist(), 0.5, 1.2)
        self.initial_pose = np.array(self.rtde_r.getActualTCPPose())
        self.initial_rot = R.from_rotvec(self.initial_pose[3:6])

    def get_target_pose(self, delta_trans: np.ndarray, delta_rot: R) -> np.ndarray:
        new_trans = self.initial_pose[:3] + delta_trans
        new_rot = delta_rot * self.initial_rot
        rotvec = new_rot.as_rotvec()

        return np.concatenate([new_trans, rotvec])

    def servo_to(self, target_pose: Optional[np.ndarray] = None, target_joint: Optional[np.ndarray] = None, speed=0.2, acceleration=1.0) -> None:
        """
        Move the robot to a target pose using servo control.
        
        Args:
            target_pose (np.ndarray): Target TCP pose as a 6-element array [x, y, z, rx, ry, rz].
            target_joint (np.ndarray): Target joint configuration as a 6-element array [j1, j2, j3, j4, j5, j6].
            speed (float): Speed for the servo movement.
            acceleration (float): Acceleration for the servo movement.
        Raises:
            ValueError: If both target_pose and target_joint are provided, or if neither is provided.
        """
        
        # Validate input parameters
        if target_pose is None and target_joint is None:
            raise ValueError("Either target_pose or target_joint must be provided.")
        elif target_joint is not None and target_pose is not None:
            raise ValueError("Only one of target_pose or target_joint should be provided.")
        
        # If target_joint is provided, move to joint configuration
        if target_joint is not None:
            self.rtde_c.servoJ(
                target_joint.tolist(),
                speed,
                acceleration,
                .02,
                0.1,
                300.0
            )
            return
        # If target_pose is provided, move to TCP pose
        if target_pose is not None:
            self.rtde_c.servoL(
                target_pose.tolist(),
                speed,
                acceleration,
                .02,
                0.1,
                300.0
            )
        
    def move_to(self, target_pose: Optional[np.ndarray] = None, target_joint: Optional[np.ndarray] = None, speed=0.25, acceleration=1.2) -> None:
        """
        Move the robot to a target pose or joint configuration.
        
        Args:
            target_pose (np.ndarray): Target TCP pose as a 6-element array [x, y, z, rx, ry, rz].
            target_joint (np.ndarray): Target joint configuration as a 6-element array [j1, j2, j3, j4, j5, j6].
            speed (float): Speed for the movement.
            acceleration (float): Acceleration for the movement.
        Raises:
            ValueError: If both target_pose and target_joint are provided, or if neither is provided.
        """
        
        # Validate input parameters
        if target_pose is None and target_joint is None:
            raise ValueError("Either target_pose or target_joint must be provided.")
        elif target_joint is not None and target_pose is not None:
            raise ValueError("Only one of target_pose or target_joint should be provided.")
        
        # If target_joint is provided, move to joint configuration
        if target_joint is not None:
            self.rtde_c.moveJ(
                target_joint.tolist(),
                speed,
                acceleration,
            )
            return
        # If target_pose is provided, move to TCP pose
        if target_pose is not None:
            self.rtde_c.moveL(
                target_pose.tolist(),
                speed,
                acceleration,
            )
        
    def get_current_pose(self):
        current_pose = self.rtde_r.getActualTCPPose()
        return np.array(current_pose)
    
    def get_current_speed(self):
        current_speed = self.rtde_r.getActualTCPSpeed()
        return np.array(current_speed)

    def stop(self):
        self.rtde_c.servoStop()
        self.rtde_c.stopScript()

if __name__ == "__main__":
    controller = UR5eController("192.168.31.10")
    
    init_pose = controller.get_current_pose()
    print(f"Initial TCP Pose: {init_pose}")

    target_joint = np.array([-90.0, -60.0, -140.0, 20.0, 90.0, 0.0]) * np.pi / 180.0
    print(f"Moved to Target Joint: {target_joint}")
    try:
        while True:
            controller.servo_to(target_joint=target_joint)
            current_pose = controller.get_current_pose()
            print(f"Current TCP Pose: {current_pose}")
    except KeyboardInterrupt:
        print("Stopping UR5e controller...")
        controller.stop()