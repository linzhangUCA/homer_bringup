# homer_bringup
ROS package to start up HomeR hardware interface

## Usage
```console
# create workspace
mkdir -p ~/homer_ws/src
cd ~/homer_ws/src

# check out package
git clone https://github.com/linzhanguca/homer_bringup.git

# resolve binary dependencies and build workspace
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/homer_ws/
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
```
