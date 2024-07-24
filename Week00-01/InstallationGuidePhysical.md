# How to set up your Robot
This readme provides a step-by-step guide on setting up your environment and connecting to the PenguinPi robot.

**For setting up the environment, we recommend Ubuntu 18 in this guide, as it works for both simulator and physical robot. However, as all the markings are done on the physical robot, the simulator environment is NOT required. You only need to [install the required python packages](#Installing-Python-env-to-work-with-a-physical-robot) if you don't plan to use the simulator environment. The python env can be installed on your home system directly, whether it's Ubuntu 18 or not.** 

For information on using the simulator environment please see [the guide on the simulator environment](../Robot_simulator/InstallationGuideSim.md#Launch-the-Simulator-Environment).

- [Installing Python env to work with a physical robot](#Installing-Python-env-to-work-with-a-physical-robot)
- [Running on the Physical Robot](#Running-on-the-Physical-Robot)
    - [Inside the robot kit](#inside-the-kit)
    - [Powering on and connecting to the robot](#powering-on-and-connecting-to-the-robot)
    - [Running codes on the robot](#running-codes-on-the-robot)
    - [Disconnecting and shutting down the robot](#disconnecting-and-shutting-down-the-robot)
- [Troubleshooting](#Troubleshooting)

---
# Installing Python env to work with a physical robot
If you only plan to work with the physical robot, then you only need to install required python packages on your home system and there is no need to install the Linux environment.

1. Install a [Python](https://www.python.org/downloads/) (use version ≥ 3.6)

2. Navigate to a directory where you want to install the robot-related stuff and open it in a terminal

3. Create a Python virtual environment named "PenguinPi" (this is optional, setting up a venv helps to ensure the robot-related packages won't interfere with your other Python stuff): ```py -3 -m venv PenguinPi```; or if this doesn't work for you, try ```python -m venv PenguinPi```

4. Activate your venv: On Windows: ```PenguinPi\Scripts\activate```; On Mac/Linux: ```source PenguinPi/bin/activate```

5. Install pip in the venv: ```python -m pip install -U pip```

6. Install wheel in the venv: ```python -m pip install wheel```

7. Install python packages needed for M1 in the venv: ```python -m pip install flask gevent pyyaml numpy requests opencv-python pynput pygame```

Note: if you follow these steps to set up a python venv, every time before running robot-related codes you'll need to activate the venv. Also when running a script, instead of ```python3``` you will use ```python```, e.g., ```python operate.py --ip 192.168.50.1 --port 8080```.

8. To deactivate the env after you are done working inside it type ```deactivate```

---
# Running on the Physical Robot

## Inside the kit

The PenguinPi robot kit includes:
* one PenguinPi robot: details see fig below
* one Wifi Dongle: to be plugged in the robot's usb port for generating wifi hotspot for a computer to connect to the robot
* one USB cable: an extender of any usb ports on the robot for dev and debugging
* one charging cable: when fully charged, the light on the charger will change from red to green

The robot has an ID number printed on the carrying case and on the robot for logging ownership and repairs. Please don't mismatch the robot and its case.

![PenguinPi Robot Kit](PenguinPiKitAnno.png?raw=true "PenguinPi Robot kit")

Here is an overview of the robot's parts. **The camera is the front side of the robot** (the triangle part of the frame is the tail).

![PenguinPi Robot](PenguinPi.png?raw=true "PenguinPi Robot")

## Powering on and connecting to the robot
First, switch the robot on with the side switch

Wait for it to boot - this takes about 1 min. When IP addresses appears on the OLED screen, e.g., ```192.168.50.1```,  the booting is finished.

Connect your PC's wifi to the PenguinPi's hotspot penguinpi:xx:xx:xx (look for it in your wifi list). The hotspot password is ```PenguinPi```. You can identify which wifi belongs to your robot by the WMAC shown on the robot's screen (the last 6 digits/letters shown would match the hotspot's last 6 digits/letters).

You can test your connection by opening a web browser and entering the address ```192.168.50.1:8080``` ("192.168.50.1" is your robot's IP, "8080" is the port). On this webpage you will be able to test the motors and camera, as well as see a range of diagnostic information relating to your robot. **Please let us know if your robot's motors and/or camera aren't working so we can arrange for replace and repair.**

![Web browser view of the physical robot](WebRobot.png?raw=true "Web browser view of the physical robot")

You can also connect to the robot using ssh (the ssh password is ```PenguinPi```), replace the IP address if needed.

```
ssh -X pi@192.168.50.1
```

You can upload files to the robot using the ```scp``` command:
```
scp -r ./LOCALDIR pi@192.168.50.1:~/REMOTEDIR
```

## Running codes on the robot
To run your codes on the robot, you need to find the IP address of your robot. This is shown on its OLED screen. When you run ```operate.py``` you will need to include the "--ip" and "--port" flags to your python running command, for example ```python3 operate.py --ip 192.168.50.1 --port 8080```. The python script will then be executed on the physical robot, instead of on the simulated robot. 

After your environment is set up, in addition to working with the physical robot, you can also [launch the simulator environment](../Robot_simulator/InstallationGuideSim.md#Launch-the-Simulator-Environment) to work with a simulated robot.

You can also connect an external screen, keyboard, and mouse to the robot, then switch it on and use it as a Raspberry Pi. Inside the Raspberry Pi interface, you can install python packages onboard of the robot by running pip in the terminal, e.g., ```python3 -m pip install pynput```. You can also install packages inside the ssh session if your PenguinPi has internet connection (you can set the internet connection up in the Raspberry Pi interface).

## Disconnecting and shutting down the robot
When you are done with the robot, inside the ssh session run ```sudo halt & logout``` to shut down the robot safely. Once the screen says 'TURNRaspberryPi OFF SAFELY' you can toggle the power switch. Avoid toggling the switch directly to "hard" shut down.

---
# Troubleshooting

- Please be gentle with the robot as they can be quite fragile
- Some of the robots may come with one or both of the wheels soldered differently than the simulator (you can check this using the 'Test motors' function on the web browser view, the Left/Right buttons should make the left/right wheel each spin **forward** briefly to be consistent with the sim). Modify the ```set_velocity``` function in [pibot.py](util/pibot.py#L21) and the ```control``` function in [operate.py](operate.py#L58) to fit how your robot is assembled. Most commonly seen type is the **right wheel rotates backwards** when running motor test, which has already been addressed in the current codes. Please let us know if one of both of your wheels have directions different from the sim so we can keep a log on this.
- Some of the robots may have a faulty camera (you can check this via the camera stream window on the web browser view). Please let us know if you have a robot with a faulty camera so we can arrange for replacement and repair.
- Some of the robots may have a faulty battery (if the charger LED blinks red when you are charging the robot and plug-unplug several times doesn't fix this). Please let us know if you have a robot with a faulty battery so we can arrange for replacement and repair.
- Virtual box not importing the image properly (not able to import and open the image at all): if the error is related to E_INVALIDARG, check if your virtualbox has been installed in C:\ (instead of D:\ or any other drive).
- Don't tick the 3D acceleration option in Virtualbox as it might cause the VM to black-screen or crash with an out-of-memory error.
- Razer graphic cards might have issues with the VM/gazebo/RViz (crashes without apparent error messages).
- If running the VM on a Mac, and encounter a "Kernel Driver Not Installed (rc=-1908 error)" follow [this guide](https://www.howtogeek.com/658047/how-to-fix-virtualboxs-%E2%80%9Ckernel-driver-not-installed-rc-1908-error/) to fix the issue.
- If installing from scratch and the command to install Gazebo: "curl -sSL http://get.gazebosim.org | sh" does nothing, follow the instructions under "Alternative installation: step-by-step" in [this guide](http://gazebosim.org/tutorials?tut=install_ubuntu)
- Slow VM on Mac: this might be a resolution issue, see [here](https://www.reddit.com/r/virtualbox/comments/houi9k/how_to_fix_virtualbox_61_running_slow_on_mac/) for fixes. You can also try [enabling 3D acceleration](https://superuser.com/questions/172989/virtualbox-running-ubuntu-is-slow-mac-os-x-host) in VistualBox
- [NEW] Some Windows 11 users may run into an "Execution Policy Settings" error, with an error message similar to "cannot be loaded because the execution of scripts is disabled on this system", when activating their Python venv. This can be fixed by typing ```Set-ExecutionPolicy Unrestricted -Scope Process``` in a terminal to change your Execution Policy Settings
- [NEW] Some robots don't show the web browser preview due to an internal server error. You can ignore this error and check if you are connected to the robot by either ssh into it or by running operate.py

---
# Acknowledgement
- Part of the lab sessions are inspired by the [Robotic Vision Summer School](https://www.rvss.org.au/)
- Robot dev in collaboration with [QUT Centre for Robotics](https://github.com/qcr/PenguinPi-robot)
