source /opt/ros/humble/setup.bash
rosdep update
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros/ros_tutorials.git -b humble-devel
cd ..
rosdep install -i --from-path src --rosdistro humble -y
colcon build
cd src
ros2 pkg create --build-type ament_python py_pubsub
cd py_pubsub/py_pubsub
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
