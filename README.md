# Python Pkg in ROS 2
## Prerequisite
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) and Setup
- Initialize a [Catkin Workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
## Colcon Setup
1. Install [Colcon](https://colcon.readthedocs.io/en/released/) Command Extensions
```
sudo apt install python3-colcon-command-extensions
```
2. Setup Auto Completion Feature. Verify the colocon-argcomplete.bash file is there
```
cd /usr/share/colcon_argcomplete/hook
ls
```
3. Add the Auto Completion to bashrc. Start by opening it in your preferred editor. I use Vim.
```
vim ~/.bashrc  
```
4. Add the following line towards the end of your file. I added it after the ROS 2 source
```
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
## Build a Catkin Workspace in ROS 2
1. Begin by creating the structure directories. By convention the ROS 2 Workspace is the following
```
mkdir -p ros2_ws/src
```
2. Build your workspace
```
cd ros2_ws
colcon build
```
If you get an error. It is likely you have not installed (Colcon)[https://colcon.readthedocs.io/en/released/]
3. Add your workspace to your bashrc. Start by opening it in your preferred editor. I use Vim.
```
vim ~/.bashrc  
```
4. Add the following line towards the end of your file. I added it after the Colcon Argument Complete source.
```
source ~/ros2_ws/install/setup.bash
```
This source command assumes that you created your workspace from your home directory ~. You will have to modifiy it depending on where your catkin workspace was initialized
## Create a Python Package
1. Begin by going into your src folder in your catkin ws
```
cd ros2_ws/src
```
2. Create your Package
```
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```
3. Now build your workspace
```
colcon build
4. If there is an issue with your colocon build command do the following. 

4.1 Look at the version of setuptools using pip3. You require pip3 list to do this. 
```
pip3 list | grep setuptools
```
4.2 Download the setuptools 58.2.0 version
```
pip3 install setuptools==58.2.0
```
5. Alternatively to build a package alone instead of the entire workspace
```
colcon build --packages-select name_of_your_pkg
```

## Resources
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- Udemy Course [ROS 2 for Beginners](https://www.udemy.com/course/ros2-for-beginners/?couponCode=2021PM20)
