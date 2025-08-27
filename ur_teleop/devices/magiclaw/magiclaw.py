#!/usr/bin/env python3

import zmq
import numpy as np
from typing import Tuple
from .protobuf import magiclaw_msg_pb2
from scipy.spatial.transform import Rotation as R

class MagiClawSubscriber:
    def __init__(self, host: str, port: int, hwm: int = 1, conflate: bool = True, timeout: int = 100) -> None:
        """Subscriber initialization.

        Args:
            host (str): The host address of the subscriber.
            port (int): The port number of the subscriber.
            hwm (int): High water mark for the subscriber. Default is 1.
            conflate (bool): Whether to conflate messages. Default is True.
        """

        print("{:-^80}".format(" MagiClaw Subscriber Initialization "))
        print(f"Address: tcp://{host}:{port}")

        # Create a ZMQ context
        self.context = zmq.Context()
        # Create a ZMQ subscriber
        self.subscriber = self.context.socket(zmq.SUB)
        # Set high water mark
        self.subscriber.set_hwm(hwm)
        # Set conflate
        self.subscriber.setsockopt(zmq.CONFLATE, conflate)
        # Connect the address
        self.subscriber.connect(f"tcp://{host}:{port}")
        # Subscribe all messages
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        # Set poller
        self.poller = zmq.Poller()
        self.poller.register(self.subscriber, zmq.POLLIN)
        self.timeout = timeout

        print("Package Claw")
        print("Message Motor")
        print("{\n\tfloat angle = 1;\n\tfloat speed = 2;\n\tfloat iq = 3;\n\tint32 temperature = 4;\n}")
        print("Message Claw")
        print("{\n\tfloat angle = 1;\n\tMotor motor = 2;\n}")
        print("Message Finger")
        print(
            "{\n\tbytes img = 1;\n\trepeated float pose = 2;\n\trepeated float force = 3;\n\trepeated float node = 4;\n}"
        )
        print("Message Phone")
        print(
            "{\n\tbytes color_img = 1;\n\trepeated int32 depth_img = 2\n\trepeated int32 depth_width = 3\n\trepeated int32 depth_height = 4\n}"
        )
        print("Message MagiClaw")
        print(
            "{\n\tfloat timestamp = 1;\n\tClaw claw = 2;\n\tFinger finger_0 = 3;\n\tFinger finger_1 = 4;\n\tPhone phone = 5;\n}"
        )

        print("Claw Subscriber Initialization Done.")
        print("{:-^80}".format(""))

    def subscribeMessage(self):
        """Subscribe the message.

        Returns:
            The message.
        """

        # Receive the message
        msg = self.subscriber.recv()
        # Parse the message
        magiclaw = magiclaw_msg_pb2.MagiClaw()
        magiclaw.ParseFromString(msg)

        # Unpack the message
        claw_angle = magiclaw.claw.angle
        motor_angle = magiclaw.claw.motor.angle
        motor_speed = magiclaw.claw.motor.speed
        motor_iq = magiclaw.claw.motor.iq
        magiclaw_pose = np.array(magiclaw.pose)

        return (
            claw_angle,
            motor_angle,
            motor_speed,
            motor_iq,
            magiclaw_pose,
        )

    def close(self):
        """Close ZMQ socket and context to prevent memory leaks."""
        if hasattr(self, "subscriber") and self.subscriber:
            self.subscriber.close()
        if hasattr(self, "context") and self.context:
            self.context.term()
            

class MagiClaw:
    def __init__(self, host: str) -> None:
        """Initialize the MagiClaw subscriber.

        Args:
            host (str): The host address of the MagiClaw subscriber.
        """
        
        self.subscriber = MagiClawSubscriber(host, 6300)
        
        self.init_pose = self.get_current_pose()
    
    def get_current_pose(self) -> Tuple[np.ndarray, R]:
        """Get the current pose from the MagiClaw subscriber.

        Returns:
            trans (np.ndarray): Translation vector in z-up coordinate system.
            rot (R): Rotation in z-up coordinate system.
        """
        
        # Subscribe to the message
        _, _, _, _, magiclaw_pose = self.subscriber.subscribeMessage()
        
        trans = magiclaw_pose[:3]
        rot = R.from_quat(magiclaw_pose[3:])
        
        return trans, rot
    
    def get_delta_pose(self) -> Tuple[np.ndarray, R]:
        """Get the delta pose from the MagiClaw subscriber.

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
        """Stop the MagiClaw subscriber."""
        
        self.subscriber.close()
        print("MagiClaw subscriber stopped.")
        
if __name__ == "__main__":
    # Example usage
    host = "192.168.31.39"
    
    magiclaw = MagiClaw(host)
    try:
        while True:
            trnas, rot = magiclaw.get_delta_pose()
            print(f"Translation: {trnas}, Rotation: {rot.as_rotvec()}")
    except KeyboardInterrupt:
        print("Stopping MagiClaw subscriber...")
        magiclaw.stop()