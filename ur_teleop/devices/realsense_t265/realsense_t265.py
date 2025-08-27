#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
from typing import Tuple
from scipy.spatial.transform import Rotation as R

class T265Tracker:
    """
    A interface for the RealSense T265 camera to track pose data.
    This class initializes the camera, retrieves pose data, and converts it to a z-up coordinate system.
    
    Attributes:
        pipeline (rs.pipeline): The RealSense pipeline for streaming data.
        config (rs.config): The configuration for the RealSense pipeline.
        initial_pose (Tuple[np.ndarray, R]): The initial pose of the camera as a translation vector and rotation.
    """
    def __init__(self):
        """
        Initializes the T265 tracker, setting up the RealSense pipeline and configuration.
        
        It waits for the first pose frame to initialize the initial pose.
        """
        
        # Initialize RealSense T265 camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.pose)
        self.pipeline.start(self.config)

        # Wait for the first pose frame to initialize
        for _ in range(10):
            frames = self.pipeline.wait_for_frames()
        self.init_pose = self._get_pose(frames)

    def _get_pose(self, frames) -> Tuple[np.ndarray, R]:
        """
        Extracts the pose data from the given frames.
        
        Args:
            frames: The frames from the RealSense pipeline containing pose data.
            
        Returns:
            trans: Translation vector as a numpy array.
            rot: Rotation as a scipy Rotation object.
        """
        
        # Get the pose frame and extract translation and rotation
        pose = frames.get_pose_frame().get_pose_data()
        trans = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
        rot = R.from_quat([pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w])
        
        # Convert to z-up coordinate
        trans, rot = self._convert_to_z_up(trans, rot)
        
        return trans, rot

    def _convert_to_z_up(self, trans: np.ndarray, rot: R) -> Tuple[np.ndarray, R]:
        """
        Converts the pose from the T265 coordinate system to a z-up coordinate system.
        
        The T265 uses a x-right, y-down, z-forward coordinate system,
        while the z-up coordinate system uses x-right, y-forward, z-up.
        T265:                   z-up:
                z                   z   y
               /                    |  /
              /                     | /
             /                      |/
             ------- x              ------- x
            |
            |
            y
        This function rotates the translation and rotation accordingly.
        Specifically, it rotates the pose by -90 degrees around the x-axis.
        
        Args:
            trans (np.ndarray): Translation vector in T265 coordinate system.
            rot (R): Rotation in T265 coordinate system.
        Returns:
            ur_pos (np.ndarray): Translation vector in z-up coordinate system.
            ur_rot (R): Rotation in z-up coordinate system.
        """
        
        # Rotation matrix to convert T265 to z-up coordinate system
        rot_mat = np.array([[1, 0, 0],
                            [0, 0, -1],
                            [0, 1, 0]])
        
        # Convert T265 translation to z-up translation
        convert_trans = rot_mat @ trans
        
        # Convert T265 rotation to z-up rotation
        convert_rot = rot_mat @ rot.as_matrix()
        convert_rot = R.from_matrix(convert_rot)
        
        return convert_trans, convert_rot

    def get_current_pose(self) -> Tuple[np.ndarray, R]:
        """
        Gets the current pose of the camera.
        
        Returns:
            trans (np.ndarray): Translation vector in z-up coordinate system.
            rot (R): Rotation in z-up coordinate system.
        """
        
        frames = self.pipeline.wait_for_frames()
        
        return self._get_pose(frames)
    
    def get_delta_pose(self) -> Tuple[np.ndarray, R]:
        """
        Gets the delta pose since the initial pose.
        
        Returns:
            delta_trans (np.ndarray): Translation vector representing the change in position.
            delta_rot (R): Rotation representing the change in orientation.
        """
        
        # Get the current pose
        trans, rot = self.get_current_pose()

        # Delta pose calculation
        delta_trans = trans - self.init_pose[0]
        delta_rot = rot * self.init_pose[1].inv()
        
        return delta_trans, delta_rot
    
    def stop(self):
        self.pipeline.stop()
        
if __name__ == "__main__":
    
    t265 = T265Tracker()
    try:
        while True:
            trans, rot = t265.get_delta_pose()
            print(f"Translation: {trans}, Rotation: {rot.as_rotvec()}")
    except KeyboardInterrupt:
        print("Stopping T265 tracker...")
        t265.stop()