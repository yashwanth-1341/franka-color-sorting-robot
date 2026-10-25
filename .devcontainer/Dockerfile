FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y \
build-essential git \
python3-colcon-common-extensions python3-rosdep \
ros-dev-tools \
ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
&& rm -rf /var/lib/apt/lists/*
# Simulation, ros2_control, MoveIt 2 and vision dependencies of the Panda workspace
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
ros-jazzy-moveit \
ros-jazzy-ros-gz \
ros-jazzy-gz-ros2-control \
ros-jazzy-ros2-control \
ros-jazzy-ros2-controllers \
ros-jazzy-xacro \
ros-jazzy-joint-state-publisher-gui \
ros-jazzy-robot-state-publisher \
ros-jazzy-rviz2 \
ros-jazzy-cv-bridge \
ros-jazzy-tf2-ros-py \
python3-opencv python3-numpy \
&& rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
CMD ["bash"]
