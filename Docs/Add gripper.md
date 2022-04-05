# Instructions on adding 3 finger Robotiq gripper to the UR5 setup for Moveit

The following set of instructions assume you have already completed the [Getting Started](https://github.com/BijoSebastian/UR5/blob/main/Docs/Getting%20started.md) instructions and have the setup working

Notice that the previous setup did not include any gripper for the UR5 neither in the simulator nor in the Moveit based plannning considerations. The following ste of instrucitons will guide you in getting the [3 finger Robotiq gripper](https://robotiq.com/products/3-finger-adaptive-robot-gripper) added to your setup.

## Getting the Robotiq 3F model

We will get this from the Robotiq repo located [here](https://github.com/ros-industrial/robotiq)

```
# change into the ur_ws workspace

cd ~/Projects/ur_ws

# clone the Robotiq repo 

git clone https://github.com/ros-industrial/robotiq src/robotiq
```
This repository is no longer maintained as of April, 2022 and can lead to a broken workspace if you try to build it as such. For now I usggets removing anything that is not needed from the cloned repo. We only need the folders named _robotiq_3f_gripper_visualization_ and _robotiq_ from the repository.

In addition, it would be helpfull to disable all the joints on the gripper other than the base joints on each finger. To do this edit the following files:

```
robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_articulated_macro.xacro
robotiq_3f_gripper_visualization/cfg/robotiq-3f-gripper_finger_articulated_macro.xacro
```

Once you have removed the other folders, proceed with installing dependencies and bulding the workspace

```
# install dependencies, following commands will take time be patient

sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace

catkin_make

# activate the workspace (ie: source it)

source devel/setup.bash
```

## Configuring the robot using MoveIt setup assistant

We will be using the MoveIt setup assitant for generating the SRDF file for our robot. Follow the steps below:

- The Setup assitant will need a top level xacro that instantiates a UR5 model and a 3F gripper model and then defines the connection between the two. Use the arm.xacro file provided [here](https://github.com/BijoSebastian/UR5/blob/main/description/arm.xacro) 

- Launch the setup assistant (Ensure that you have sourced the workspace before doing this)
```
roslaunch moveit_setup_assistant setup_assistant.launch
```
- Follow the steps mentioned [here](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html) to configure the robot. Following are some things to note:

    - Add a virtual joint between the ur5e_base_link and world
    - Add a planning group for the arm with kdl_kinematics_plugin/KDLKinematicsPlugin as Kinematic Solver as the planner. Add kinematic chain to the planning group with *base_link* as Base link and tool0 as Tip link
    - Add a planning group for the gripper with no solver attached. Add only the base joints of each finger onn the gripper to this group
    - Define a home pose with the robot upright
    - Define open and close pose for the gripper
    - Do Auto Add FollowJointsTrajctory Controllers For Each Planning Group, we will change this later

- With the default controllers in the Moveit config generation somehow the planner fails continously. Solution was to edit the *ros_controllers.yaml* file in the config folder manually to match the file under *fmauch_universal_robot/ur5e_moveit_config/config*, with this change the planner seems to work fine

- If the planner takes too long to plan that is primarily due to the fact that the new gripper introduces a lot more links which needs to be taken into account during the plannig phase.

    - A solution here would be to simplify the gripper by using the unarticulaed finger modles 

# note to self for later the table and cylinder in the wolrd coudl be added directly by editing the xacro file and adding in the corresponding urdf for the same. 
