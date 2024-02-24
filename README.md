<<<<<<< HEAD
## bfmc_dashboard package

### 1.a Create the ros workspace if you don t have it already, and build
mkdir ~/catkin_ws
catkin_make
### 1.b So you don't have to source setuo.bash every time you open a new terminal
gedit ~/.bashrc
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

### 2 Git clone into src folder and build
cd ~/catkin_ws/src
git clone https://github.com/euhenn/bfmc_dashboard.git
catkin_make

### 3 Run the launch file
cd ~/catkin_ws
roscore
roslaunch bfmc_dashboard camera_with_gui.launch
=======
# bfmc_dashboard
>>>>>>> 5e7fb4dfaaff385022806a3cfde494ab7347c562
