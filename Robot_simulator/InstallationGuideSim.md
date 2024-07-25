# How to set up your Simulator Environment
This readme provides a guide on using the simulator environment.

**If you don't plan to use the simulator, then you may skip ahead to [Running on the Physical Robot](../Week00-01/InstallationGuidePhysical.md)**

---
# Installing Linux 
We provide three options here for you. Option 1 is the easiest solution but it may not give you the best performance computation-wise. The other two options would likely give you boost in computational performance, but may require a bit more work to get things set up. We strongly recommend you to stick with Option 1 if you are not comfortable with setting up your own ROS environment. **You only need to install the Linux environment if you plan to use the simulator**

Note that for people with Mac (especially with Apple M1/M2 chips), please work with the demonstrators to see what works best, as some of these instructions may not work for you.

## Option 1: Using the pre-built virtual machine image 

Install [Oracle VM Virtualbox](https://www.virtualbox.org/) (make sure your [BIOS setting allows virtualization](https://forums.virtualbox.org/viewtopic.php?f=1&t=62339))

Download [the ready-to-use VM image](https://drive.google.com/file/d/1_MRbKHOA1Lo_qFBaASKvvaQ36HuJRIPI/view?usp=sharing) and [import](https://docs.oracle.com/en/virtualization/virtualbox/6.0/user/ovf.html#ovf-import-appliance) it to your VirtualBox (username: ```ece4078```  password: ```2021Lab```). 

You can change the amount of resources (e.g., RAM, processing cores) assigned to your VM in VirtualBox settings after importing the VM image depending on the hardware available to you.

## Option 2: Installing WSL2 on Windows
The new Windows Subsystem for Linux (WSL2) has nice supports for GUI app (eg. Gazebo), and it may run faster on your laptop/PC compared to running in a virtual machine. However, the nice GUI support for WSL2 only works well on Windows 11 (and certain Windows 10 build), so this option may not be viable for you if you don't have Windows 11. 

Please follow these [instructions](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#2-install-wsl) to set up your WSL2. Note that you should install **Ubuntu 18.04**, and you can skip the step to install octave.

To access the files in your Win11 from the Ubuntu subsystem, in your ubuntu terminal you can use ```cd /mnt/```. For example, if you have a folder named ```test_folder``` in the C:\ drive of Win11, you can navigate to that directory using ```cd /mnt/c/test_folder```. To transfer files between Win11 and Ubuntu, you can use the ```cp``` command (more info [here](https://manpages.ubuntu.com/manpages/trusty/man1/cp.1.html)). For example, if you have a file at C:/test.txt in Win11, you can copy it to your Ubuntu using ```cp /mnt/c/test.txt your_destination_dir```.

Otherwise, you may try [this](https://devblogs.microsoft.com/commandline/access-linux-filesystems-in-windows-and-wsl-2/) if you prefer using explorer to drag and drop. You may place your files/folders in ```/home/your_user_name/```.

## Option 3: Dual boot your computer 
CAUTION! Please only proceed with this option if you know what you are doing, and do this at your own risk.

Here is a [tutorial](https://www.youtube.com/watch?v=CWQMYN12QD0&ab_channel=TechnoTim) you can follow to set up dual boot on your computer. 

# Install the environment from scratch in an empty Ubuntu 18 (Only for Options 2 & 3)

***Note: ROS-Melodic requires Ubuntu 18, and you only need to do this if you have chosen either Option 2 or 3***

This instruction allows you to install the environment that supports development with both the physical and simulated robot. If you are only working with the physical robot, you only need to install the python packages. We provide the simulation environment as an additional resource since it can be useful for development and debugging when you don't have access to a physical robot or an arena, or when you want to have precise control over an object's coordinates and behaviours for evaluating a function.

## Python packages
install python packages
```
sudo apt update
sudo apt install python3-pip curl python-flask python-gevent
python3 -m pip install --upgrade pip 
python3 -m pip install flask gevent pyyaml numpy requests opencv-python pynput pygame
```

## ROS + Gazebo for simulation
install Gazebo 11 (after the installation is done you should be able to open Gazebo by typing ```gazebo``` in your terminal)
```
curl -sSL http://get.gazebosim.org | sh
```

install ROS Melodic
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop
```

install ROS Melodic packages for Gazebo 11
```
sudo apt install ros-melodic-gazebo11-plugins
sudo apt install ros-melodic-gazebo11-ros-pkgs 
sudo apt install ros-melodic-gazebo11-ros-control
sudo apt install python-catkin-tools python3-dev python3-catkin-pkg-modules python3-rospkg-modules python3-numpy python3-empy
```

setup local catkin workspace
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

download the PenguinPi modules
```
cd ~/catkin_ws/src
sudo apt install git
sudo apt update
git clone https://bitbucket.org/cirrusrobotics/penguinpi_description.git
git clone https://bitbucket.org/cirrusrobotics/penguinpi_gazebo.git
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
cd ~/catkin_ws
catkin_make
```

open ```~/catkin_ws/penguinpi_gazebo/scripts/server``` in a text editor, find [line 399-404](https://bitbucket.org/cirrusrobotics/penguinpi_gazebo/src/5760f245291dc639e146fd4adc3f5328333c2ef7/scripts/server#lines-399) and replace the codes with the following:
```
serverport = 40000
http_server = WSGIServer(('', serverport), app)
http_server.serve_forever()
```

**[EDIT]** Please replace the ```src``` folder in your local ```catkin_ws``` directory with [this src folder provided](https://drive.google.com/file/d/1VSmSmg7iuF-tQzWhlnmtwjsNhzxlPqU8/view?usp=sharing), which contains the required models, launch / world files, and the ```scene_manager.py``` script, then run ```catkin_make``` again under ```catkin_ws``` with the downloaded ```src``` folder).

# Launch the Simulator Environment

From within the Ubuntu environment (VM/native/WSL2, whichever one you use), open a terminal and type
```
source ~/catkin_ws/devel/setup.bash
roslaunch penguinpi_gazebo ECE4078.launch
```
You should see a base Gazebo world open with PenguinPi inside an empty map
![Empty World](EmptyWorld.png?raw=true "Empty World")

This terminal window will need to stay open while you are working with the simulated robot. **Keep an eye on the messages in this terminal window**, if there are only normal messages (text color in white) and warnings (text color in yellow) then all is good. If you see error messages (text color in red) then you will need to double check if all required packages are installed (see [set up the environment](../Week00-01/InstallationGuidePhysical.md#install-the-environment-from-scratch-in-an-empty-ubuntu-18)). For example, without the ```flask``` python package you won't be able to establish a connection to control the virtual robot spawn inside Gazebo, and you will see an error message about the penguinpi server failing to start.

**Open a new terminal** and spawn objects in the map by typing the following commands (right click on scene_manager.py and go to "Properties -> Permissions" to double check that the "Execute" box is ticked. This file is under the ~/catkin_ws/src/penguinpi_gazebo/ folder)
```
source ~/catkin_ws/devel/setup.bash
rosrun penguinpi_gazebo scene_manager.py -l map1.txt
```
You should see the empty map now populated with targets and markers.
Play around with the ```rosrun penguinpi_gazebo scene_manager.py``` command using the following variations:
- Run without ```-l map1.txt``` tag to spawn objects at random locations
- Run with ```-d``` tag to remove the spawned objects
- Run with ```-s NAME.txt``` to save a new map

You can change the number of targets and markers in the map by changing obj_class_dict at Line 23 of scene_manager.py

You can include more virtual objects under the ~/catkin_ws/src/penguinpi_gazebo/models/ folder, see [this tutorial](https://classic.gazebosim.org/tutorials?tut=import_mesh&cat=build_robot) for more information. 

![Simulator Map](SimulatorMap.png?raw=true "Simulator Map")

