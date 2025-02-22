## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

print("""
    ____      __       __   ____             _______                    
   /  _/___  / /____  / /  / __ \___  ____ _/ / ___/___  ____  ________ 
   / // __ \/ __/ _ \/ /  / /_/ / _ \/ __ `/ /\__ \/ _ \/ __ \/ ___/ _ |
 _/ // / / / /_/  __/ /  / _, _/  __/ /_/ / /___/ /  __/ / / (__  )  __/
/___/_/ /_/\__/\___/_/  /_/ |_|\___/\__,_/_//____/\___/_/ /_/____/\___/  
                                                                        D435-i
""")

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:

    clock=0 #clock for limit the ROS publishing rate
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Extract data from frames
        depth_data = np.asanyarray(depth_frame.get_data())
        color_data = np.asanyarray(color_frame.get_data())
        # Extract intrinsic parameters
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        # Extract timestamps
        depth_timestamp = depth_frame.timestamp
        color_timestamp = color_frame.timestamp
        # Extract frame sizes and data types
        depth_width = depth_frame.width
        depth_height = depth_frame.height
        depth_format = depth_frame.profile.format()
        color_width = color_frame.width
        color_height = color_frame.height
        color_format = color_frame.profile.format()

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_data, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_data.shape
        # Create a dictionary to store the data
        camData = {
            'depth_data': np.clip(depth_data, 0, 255).flatten().astype(int).tolist(),
            'color_data': np.shape(color_data),
            'depth_intrinsics': depth_intrinsics,
            'color_intrinsics': color_intrinsics,
            'depth_timestamp': depth_timestamp,
            'color_timestamp': color_timestamp,
            'depth_width': depth_width,
            'depth_height': depth_height,
            'depth_format': depth_format,
            'color_width': color_width,
            'color_height': color_height,
            'color_format': color_format,
        }

        clock+=1
        if clock==20: #limits the publishing rate (needs to be slower than the camera frequency)
            print(camData)
            clock=0

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_data = cv2.resize(color_data, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_data, depth_colormap))
        else:
            images = np.hstack((color_data, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

finally:
    # Stop streaming
    pipeline.stop()