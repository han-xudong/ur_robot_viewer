# UR Robot Viewer

## Introduction

This repository is a viewer for UR robot, visualizing the robot's 3D state and plotting the joint and TCP data.

## Features

The viewer is based on Rerun and RTDE, and provides the following features:

- 3D visualization of UR robot
- Plots of joint angles, joint velocities, joint torques
- Plots of TCP pose, TCP velocity, TCP force

## Installation

To install the UR Robot Viewer, follow these steps:

```bash
git clone https://github.com/han-xudong/ur_robot_viewer.git
cd ur_robot_viewer
pip install -e .
```

## Usage

First, check the `config/address.yaml` file to set `robot_ip` to the IP address of your UR robot.

Next, run the data streamer:

```bash
python scripts/stream_robot.py --fps <fps>
```

where `<fps>` is the frames per second for streaming the data (e.g., `100`).

Then, run the viewer:

```bash
python ur_robot_viewer/viewer.py --robot <your_robot>
```

where `<your_robot>` is the type of your UR robot, e.g., `ur3`, `ur5`, or `ur10`.

Now you will see the 3D visualization of the robot and the plots of joint and TCP data.

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgements

- [rerun](https://rerun.io)
- [RTDE_Python_Client_Library](https://github.com/UniversalRobots/RTDE_Python_Client_Library)
- [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde)
