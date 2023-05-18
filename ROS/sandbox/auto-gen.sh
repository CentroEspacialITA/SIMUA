source /opt/ros/humble/setup.bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/ros/ros_tutorials.git -b humble-devel
cd ..
rosdep install -i --from-path src --rosdistro humble -y
colcon build
