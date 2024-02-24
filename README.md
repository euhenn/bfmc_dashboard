# bfmc_dashboard

## Setting up ROS workspace
### 1.a Create the ros workspace if you don t have it already, and build
```bash
mkdir ~/catkin_ws
catkin_make
```
### 1.b So you don't have to source setuo.bash every time you open a new terminal

```bash
gedit ~/.bashrc
```
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## 2. Install [ROS Qt plug-in](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html). Go with the Bionic online installer (works with noetic on Ubuntu 20.04)

## 3. Git clone into src folder and build
cd ~/catkin_ws/src
git clone https://github.com/euhenn/bfmc_dashboard.git
catkin_make

## 4. Run the launch file
cd ~/catkin_ws
roscore
roslaunch bfmc_dashboard camera_with_gui.launch

Check the files and project in the Qt IDE. "Open project" and select the folder of your workspace (catkin_ws in our case) the header file .h with the .ui file are separated from the implementation file .cpp into 2 folders to maintain the ros logic
