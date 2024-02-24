# bfmc_dashboard

## 1. Setting up ROS workspace
### 1.a Create the ros workspace if you don t have it already, and build
```bash
mkdir ~/catkin_ws
catkin_make
```
### 1.b So you don't have to source setuo.bash every time you open a new terminal

```bash
gedit ~/.bashrc
```
Add the following lines at the end of the file
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## 2. Install [ROS Qt plug-in](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html)
Go with the Bionic online installer, as this one works with noetic on Ubuntu 20.04 also.

## 3. Git clone into src folder and build
```bash
cd ~/catkin_ws/src
git clone https://github.com/euhenn/bfmc_dashboard.git
```
Always catkin_make into the workspace folder!
```bash
cd ~/catkin_ws
catkin_make
```
## 4. Run the launch file
Open a master node
```bash
roscore
```
Launch the example in another terminal
```bash
roslaunch bfmc_dashboard camera_with_gui.launch
```


## Additional info
Check the files and project in the Qt IDE. "Open project" and select the folder of your workspace (catkin_ws in our case) the header file .h with the .ui file are separated from the implementation file .cpp into 2 folders to maintain the ros logic.


### Cheat sheet :)
granting read and write permissions
```bash
sudo chmod 666 /dev/ttyUSB0
```
node that listens on the serial
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
ros stuff
```bash
rosrun rqt_topic rqt_topic
rosrun rqt_graph rqt_graph
rostopic pub toggle_led std_msgs/Empty --once
```

