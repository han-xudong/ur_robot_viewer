import pyrealsense2 as rs

ctx = rs.context()
print(f"RealSense devices connected: {ctx.query_devices().size()}")

for dev in ctx.query_devices():
    print(f"Device Name: {dev.get_info(rs.camera_info.name)}")
