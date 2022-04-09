# Instructions on adding 3 finger Robotiq gripper to the UR5 setup for MoveIt

The following set of instructions assume you have already completed the [Getting Started](https://github.com/BijoSebastian/UR5/blob/main/Docs/Getting%20started.md) instructions and have the setup working

Notice that the previous setup did not include any gripper for the UR5 neither in the simulator nor in the MoveIt based planning considerations. The following set of instructions will guide you in getting the [3 finger Robotiq gripper](https://robotiq.com/products/3-finger-adaptive-robot-gripper) added to your setup.

## Getting the Robotiq 3F model

We will get this from the Robotiq repo located [here](https://github.com/ros-industrial/robotiq)

```
# change into the ur_ws workspace

cd ~/Projects/ur_ws

# clone the Robotiq repo 

git clone https://github.com/ros-industrial/robotiq src/robotiq
```
This repository is no longer maintained as of April, 2022 and can lead to a broken workspace if you try to build it as such. For now, I suggest removing anything that is not needed from the cloned repo. We only need the folders named _robotiq_3f_gripper_visualization_ and _robotiq_ from the repository.

In addition, it would be helpful to disable all the joints on the gripper other than the base joints on each finger. To do this edit the following files:

```
#In file
robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated_macro.xacro
#change line 44 to
<joint name="${prefix}palm_finger_1_joint" type="fixed">
#and line 51 to 
<joint name="${prefix}palm_finger_2_joint" type="fixed">

#In file
robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_finger_articulated_macro.xacro
#change line 115 to 
<joint name="${prefix}joint_2" type="fixed">
#change line 122 to
<joint name="${prefix}joint_3" type="fixed">

#In file 
robotiq/package.xml
#Remove the following lines
<exec_depend>robotiq_modbus_tcp</exec_depend>
<exec_depend>robotiq_3f_gripper_control</exec_depend>
<exec_depend>robotiq_2f_gripper_control</exec_depend>
<exec_depend>robotiq_ft_sensor</exec_depend>
#since we only need the visualization to be built
```

Once you have removed the other folders, proceed with installing dependencies and building the workspace

```
# install dependencies, following commands will take time be patient

sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace

catkin_make

# activate the workspace (source it)

source devel/setup.bash
```
The workspace should build without any errors

## Configuring the robot with gripper and a table using MoveIt setup assistant 

We will be using the MoveIt setup assistant for generating the SRDF file for our workcell consisting of the robot, gripper and the table. Follow the steps below:

- The Setup assistant will need a top level xacro. The top level xacro should define a macro that instantiates a UR5 model and a 3F gripper model and then defines the connection between the two in the form of a macro. The macro will also define the table and attach the robot to the table. This new macro should then be instantiated. 

- Use the workcell.xacro file provided [here](https://github.com/BijoSebastian/UR5/blob/main/description/workcell.xacro). 

- Launch the setup assistant (Ensure that you have sourced the workspace before doing this)
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
- Follow the steps mentioned [here](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html) to configure the robot. Following are some things to note:

    - Add a virtual joint between the table and world
    - Add a planning group for the arm, named manipulator, with kdl_kinematics_plugin/KDLKinematicsPlugin as Kinematic Solver as the planner. Add kinematic chain to the planning group with *base_link* as Base link and tool0 as Tip link
    - Add a planning group for the gripper with no solver attached. Add only the base revolute joints of each finger on the gripper to this group
    - Define a pose called "up" with the robot upright
    - Define open and close pose for the gripper
    - Define the robotiq gripper as an end effector, with tool0 as the parent link and the manipulator as the parent group
    - Do Auto Add FollowJointsTrajctory Controllers For Each Planning Group, we will change this later
    - Give the name *ur_workcell_moveit_config* for the config to be generated

- If the planner takes too long to plan that is primarily due to the fact that the new gripper introduces a lot more links which needs to be taken into account during the planning phase.

    - A solution here would be to simplify the gripper by using the unarticulated finger model 
    
- With this you should be good to launch the panning group along with RViz to test out the planning capabilities

```
roslaunch ur_workcell_moveit_config demo.launch
```

## <mark> Steps to make the above setup work with the URSim and the real robot </mark>

- With the default controllers in the MoveIt config generation somehow the planner fails repeatedly. Solution was to manually edit the *ros_controllers.yaml* file in the config folder (*.../ur_workcell_moveit_config/config/ros_controllers.yaml*) to match the file under *fmauch_universal_robot/ur5e_moveit_config/config*. 

- The generated MoveIt config will be missing *ur_workcell_moveit_config/launch/ur_workcell_moveit_planning_execution.launch*. Copy this file from the *fmauch_universal_robot/ur5e_moveit_config/launch/ur5e_moveit_planning_execution.launch*, rename accordingly and make the following changes
```
#on line 9
<include file="$(find ur_workcell_moveit_config)/launch/move_group.launch">
```
 We need MoveIt to load the new robot description we generated, so make the following changes:
 ```
 #In file
 ur_workcell_moveit_config/launch/move_group.launch
 #Change line 40 to 
 <arg name="load_robot_description" default="true" />
 #In file 
 ur_workcell_moveit_config/launch/planning_context.launch
 #Change line 3 to 
 <arg name="load_robot_description" default="true"/> 
 ```
 
- Follow instructions from the getting started to launch the URSim simulator and run the external control program

- With the simulator running launch the following commands on the host system (Each in a separate terminal window, with the workspace sourced)
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.56.101

roslaunch ur_workcell_moveit_config ur_workcell_moveit_planning_execution.launch 

roslaunch ur_workcell_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur_workcell_moveit_config)/launch/moveit.rviz
```

You should be able to use the RViz setup to plan and send commands to the simulator.
 
### The same setup as above could be used for using the new planning group with the actual robot. 

- Follow instructions from the getting started and remember to use the new ur_workcell_moveit_config commands used above instead of the ones provided in  getting started

### For later

- It is possible that the manipulator may not reach the planned pose when executing motion plans. This could be caused due to the fact that manipulator plan group uses the base_link as the origin where as in reality the base link is 0.025 above the origin of the world along z axis (Assuming the world origin coincides with the table/box origin)
