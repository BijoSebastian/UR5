# Instructions to get started with the UR5 

Host machine 
- OS version: Ubuntu 20.04
- Ros noetic already installed
- Create a Catkin workspace called ur_ws under a Projects folder at home  

## Follow instructions given on [Universal Robots ROS Driver]( https://github.com/UniversalRobots/Universal_Robots_ROS_Driver): 
```
# change into the workspace

cd ~/Projects/ur_ws

# clone the driver

git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description. This is currently necessary, until the changes are merged upstream.

git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies, following commands will take time be patient

sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace

catkin_make

# activate the workspace (ie: source it)

source devel/setup.bash
```

## Follow instructions given [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/usage_example.md) to setup the simulation 

- Download and install VirtualBox
```
sudo apt install virtualbox
```
- Create a virtual machine with Ubuntu 16.04: Recommended settings 4096 bytes of RAM and 50 GB fixed size hard disk. 

- After creation change resolution to 1024x768 for ease of use

- Install Java Development Kit 8 within the virtual machine
```
sudo apt-get update 
apt install openjdk-8-jdk
```
### Install UR-SIM: Offline Polyscope simulator within the virtual machine

- Recommended version is [Offline Simulator - CB3 - Linux - URSim-3.15.7](https://www.universal-robots.com/download/software-cb-series/simulator-linux/offline-simulator-cb3-linux-ursim-3157/). 

- This will require you to create an account with Universal Robots if you do not have one already

- Extract and install (provide password when needed):
```
cd ursim-3.15.7.xxxxx
./install.sh
```

- You may receive an error stating 
```
dpkg:error processing package runit (--configure):
```
- This seems to not affect the simulator working, so ignoring for now.

- Within the ursim folder, now you can simply launch the software from the terminal for UR-5 robot by:

```
./start-ursim.sh UR5
```

### Ensure that the simulator works as expected:

- Click on Run Program
- Go to Move tab
- Change slider values to see robot moving in simulation

### Setup Simulation to work with ur_robot_driver. 

- For using the ur_robot_driver with a real robot or a simulated robot you need to install the externalcontrol-1.0.5.urcap in the polyscope (teaching pendent on the real robot or simulator itself for simulated robot) which can be found [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/resources/externalcontrol-1.0.5.urcap). 

- Copy the downloaded file (do not extract it)
 into the ~/ursim-3.15.7.xxx/programs.UR5 folder 

- Launch simulator again and follow instructions below:

    - Since we have set up a CB3 simulator follow instructions [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)

    - If you have setup the e-Series simulator follow instructions [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/README.md#setting-up-a-ur-robot-for-ur_robot_driver) 

    - Note that the setup will by default have the remote host IP set to 192.168.56.1 and the port set to 50002. Keep these unchanged we will see where these addresses come from in the next step.

- Save the new external control program for ease of use later

### Setup communication between the host PC and the virtual machine for sending commands to the simulator

- Shutdown the virtual machine 

- Within VirtualBox main window, click File > Host Network Manager. A dialog will open in which you can click the create button which will add an entry to the list (vboxnet0). It should automatically get the static IP address 192.168.56.1. If not, set it up manually in the dialog. Once done, you can close the dialog again to go back to your VirtualBox main window. 

- Select the virtual machine you created and click on Settings. Select the Network point in the list on the left and activate at least one network adapter, setting the Attached to dropdown menu to Host-only Adapter. Select the network you just created (vboxnet0). Click on the ok button to close the dialog and go back to your main window.

- As a side note: A host-only network is not the only option to get this running, but it is the easiest. If you consider using another networking mode, make sure that the virtual machine is reachable via its own IP address from the host machine and that it can reach the host machine as well.

- With this setup the host machine should have a static IP of 192.168.56.1 and the virtual machine a static IP of 192.168.56.101. We will assume this setup for the rest of the instructions, make necessary changes if this is not the case for you.

# Starting the driver and visualizing the simulated robot in RViz

- Within the virtual machine launch the simulator. Make sure that it is running, as otherwise no joint data will be sent.

-On the host machine, in a new terminal:

```
# activate the workspace (it would be useful to create an alias for this )

source ~/Projects/ur_ws/devel/setup.bash

# launch the driver 

roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.56.101
```

- In another terminal:

```
# activate the workspace 

source ~/Projects/ur_ws/devel/setup.bash
roslaunch ur_robot_driver example_rviz.launch
```

This should open up an RViz window showing the robot. Try moving around the robot's joints within the simulator using the sliders and watch how it also moves inside ROS.

# Control the simulated robot using the test_move script

- Before any control of the robot can happen, a program containing the **External Control** program node must be running on the robot. You should see the output *Robot ready to receive control commands* in the terminal where you launched the driver. 

- With the above setup, in a third terminal run:

```
rosrun ur_robot_driver test_move
```

- Follow instructions on screen

# Control the simulated robot using MoveIt!

- For controlling the robot using MoveIt! 

Start the following three launch files (Each in a separate terminal window, remember to source the ur_ws workspace in each new terminal):

```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.56.101

roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch

roslaunch ur5e_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5e_moveit_config)/launch/moveit.rviz
```

- You should get an RViz window showing the robot overlayed with an orange version of the robot

- You can change the planning target by dragging around the blue ball. The orange robot will show the configuration used to reach the target pose.

- By clicking on the Plan button in the left panel a path from the current pose to the target pose is calculated. On success, it gets animated using a semi-transparent version of the robot

- Remeber to select the Collision-aware IK and Use cartesian paths option for smooth movements 

- By clicking on the Execute button in the left panel the robot executes the planned motion

# Setting up above modes of control to work with the real robot

- Power up the robot
- Navigate to Setup Robot / Setup Network
- Keep the robot in static address mode and provide an IP address (e.g. 192.168.1.25)
- Input Netmask as 255.255.255.0, Gateway and DNS server as 0.0.0.0
- Connect your desktop PC to the robot via a direct LAN cable connection
- On the PC setup the ethernet connection to be static IP and provide an IP address (e.g. 192.168.1.50)
- Input Netmask as 255.255.255.0, Gateway and DNS server as 0.0.0.0
- Wait a few seconds for the network to setup and try pinging the robot from your PC
- You should now be good to use the above mentioned direct control or Move-it based planning with the following minor modifications:
    - Provide the remote PC IP in the external control mode on the Polyscope (teaching pendent)
    -  When you launch the ur_robot_driver on the host PC provide the correct robot_ip

# Result

![First run](https://github.com/BijoSebastian/UR5/blob/main/Results/Project1.gif)