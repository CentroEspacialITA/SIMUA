# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# mypy: ignore-errors
# pylint: disable-all
#!/usr/bin/python
from custom_msgs.msg import Custom as CustomMsg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import warnings
import rclpy
import json
import cv2
import os

# ignore warnings
warnings.filterwarnings("ignore")

# initialize the Camera object
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
pipeline.start(config)
timer_period=0.05  # Adjust this based on your desired frame rate

# initialize ROS2 methods
rclpy.init()
node=Node('camera_node')
publisher=node.create_publisher(Image,'camera_image',10)
bridge=CvBridge()

print("""
    ____      __       __   ____             _______                    
   /  _/___  / /____  / /  / __ \___  ____ _/ / ___/___  ____  ________ 
   / // __ \/ __/ _ \/ /  / /_/ / _ \/ __ `/ /\__ \/ _ \/ __ \/ ___/ _ |
 _/ // / / / /_/  __/ /  / _, _/  __/ /_/ / /___/ /  __/ / / (__  )  __/
/___/_/ /_/\__/\___/_/  /_/ |_|\___/\__,_/_//____/\___/_/ /_/____/\___/  
                                                                        D435-i
""")

print("\n")
userParams={
    "saveInDisk":input("Save camera data in disk (Y/N)? -> ")
}
print("\n")

try:
    while rclpy.ok():
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

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # Convert the color frame to a NumPy array
        color_image = np.asanyarray(color_frame.get_data())
        
        # Set the timestamp of the ROS message (optional)
        simple_image_msg=bridge.cv2_to_imgmsg(color_image,encoding='bgr8')
        simple_image_msg.header.stamp = node.get_clock().now().to_msg()

        # Create a dictionary to store the data
        camData = {
            'color_image':np.clip(color_image, 0, 255).flatten().astype(int).tolist(),
            'color_image_shape':str(np.shape(color_image)),
            'depth_data': np.clip(depth_data, 0, 255).flatten().astype(int).tolist(),
            'depth_shape':str(np.shape(depth_data)),
            'color_data': np.clip(color_data, 0, 255).flatten().astype(int).tolist(),
            'color_data_shape':str(np.shape(color_data)),
            'depth_intrinsics': str(depth_intrinsics),
            'color_intrinsics': str(color_intrinsics),
            'depth_timestamp': str(depth_timestamp),
            'color_timestamp': str(color_timestamp),
            'depth_width': int(depth_width),
            'depth_height': int(depth_height),
            'depth_format': str(depth_format),
            'color_width': int(color_width),
            'color_height': int(color_height),
            'color_format': str(color_format),
        }

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

        # REFAZER PARA CUSTOM MSGS!!
        # Publish the ROS Image message
        # msg = CustomMsg()
        # msg.colorimage=camData['color_image']
        # msg.colorimageshape=camData['color_image_shape']
        # msg.depthdata=camData['depth_data']
        # msg.depthshape=camData['depth_shape']
        # msg.colordata=camData['color_data']
        # msg.colordatashape=camData['color_data_shape']
        # msg.depthintrinsics=camData['depth_intrinsics']
        # msg.colorintrinsics=camData['color_intrinsics']
        # msg.depthtimestamp=camData['depth_timestamp']
        # msg.colortimestamp=camData['color_timestamp']
        # msg.depthwidth=camData['depth_width']
        # msg.depthheight=camData['depth_height']
        # msg.depthformat=camData['depth_format']
        # msg.colorwidth=camData['color_width']
        # msg.colorheight=camData['color_height']
        # msg.colorformat=camData['color_format']

        msg=simple_image_msg
        publisher.publish(msg)
        node.get_logger().info(f'Published camera frame: {node.get_clock().now().to_msg()}')
        rclpy.spin_once(node,timeout_sec=timer_period)

except KeyboardInterrupt:
    if userParams['saveInDisk'].lower()=='y':
        print(userParams)
        # converts data object to JSON format and saves into a file
        path='./output'
        os.makedirs(path,exist_ok=True)
        jsonFilePath=os.path.join(path, "camera-data.json")
        with open(jsonFilePath,"w") as json_file:
            json.dump(camData, json_file, indent=4)
        print(f"\nJSON file saved at: {jsonFilePath}\n")

    pipeline.stop()
    node.destroy_node()
    rclpy.shutdown()
