installation:

# ROS install on conda ([source](https://robostack.github.io/GettingStarted.html)):
```
mamba create -n ros_env
mamba activate ros_env
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
mamba install ros-noetic-desktop
mamba deactivate
mamba activate ros_env

mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools
```
* test with:
```
mamba activate ros_env
roscore
```
# moveit install on conda ([source](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html))(modified):

```
mamba install wstools
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src

wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove moveit_tutorials
wstool update -t .

cd ~/ws_moveit/src

# git clone https://github.com/ros-planning/moveit_tutorials.git -b master
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel

cd ~/ws_moveit
mamba env update --file environment_pribavoj.yaml

conda config --env --add channels conda-forge

cd ~/ws_moveit/src
touch moveit_tutorials/CATKIN_IGNORE

cd ~/ws_moveit
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```
```
source ~/ws_moveit/devel/setup.bash
```
