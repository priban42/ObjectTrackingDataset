commonly used commands

mamba activate pribavoj
cd ~/pribavoj_ws_moveit/
source devel/setup.bash

realsense-viewer

roslaunch realsense2_camera rs_camera.launch
#path to settings: /home/ros/miniconda3/envs/pribavoj/share/realsense2_camera/launch/rs_camera.launch
rosrun image_view image_view image:=/camera/realsense2_camera


roslaunch panda_moveit_config franka_control.launch robot_ip:=192.168.102.11


installation:

#ROS install on conda:

mamba create -n ros_env
mamba activate ros_env

# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channel
conda config --env --add channels robostack-staging
# remove the defaults channel just in case, this might return an error if it is not in the list which is ok
conda config --env --remove channels defaults

# Install ros-noetic into the environment (ROS1)
mamba install ros-noetic-desktop

mamba deactivate
mamba activate ros_env

mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools

#moveit install on conda

mamba install wstools

mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src

wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove moveit_tutorials
wstool update -t .

cd ~/ws_moveit/src

###git clone https://github.com/ros-planning/moveit_tutorials.git -b master
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel

cd ~/ws_moveit
mamba env update --file environment_pribavoj.yaml

conda config --env --add channels conda-forge
touch moveit_tutorials/CATKIN_IGNORE
