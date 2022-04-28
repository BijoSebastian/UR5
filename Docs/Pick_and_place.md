# Instructions to impliment automated pick and place capabilities on UR5 using April tags 

Once the April tag based pose estimation is working as expected, we can use it to perform automated pick and place operation.

Note that we will be operating under the assumption that the "table" is our fixed or world coordinte system.

## Obtain the transformation between the table and the camera

- Place the April tag marker on the table and  record the estimated transfrom between the fixed camera and the table.

- We already know the transform between the table and the robot base, same as the one used between the box and the robot base in the [workcell.xacro file](https://github.com/BijoSebastian/UR5/blob/main/description/workcell.xacro). 

- The box in the workcell.xacro file will be used for collission checking by the planning group.

- Create a [TF broadcastor node](https://github.com/BijoSebastian/UR5/blob/main/pick_and_place/src/tf_broadcaster.cpp) to publish both of the above transfroms continously. 

- With this we now have the transfrom from detected tag -to- camera -to- table (fixed frame). We also have the entire UR kinemtaic chain -to- table as well. This will allow the MoveIt framework to estimate motion plans to move the robot's Tool Center Point (TCP) to the detected tag position. 

## Using the C++ move group interface 
- With the transforms available the next step is to use the move group interface to plan the plath and move the mainpulator to the detected pose. 
 
- Folow [this](https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html) tutorial to the write the pick and place tutorial that move the robot to predefined poses. The completed code is available [here](https://github.com/BijoSebastian/UR5/blob/main/pick_and_place/src/pick_place_tutorial.cpp). 
- Make nececssary changes to the cmake file and package.xml 
  - Note that the gripper capabilities are commented out since we do not have the gripper interfaced on the real robot or simulated in the URSim. This will be explored at a later point.

## Putting it all togther
   
- Adding a TF listener to the above code to obtain the updated pose of the april tag is the last remaining piece. See completed code [here](https://github.com/BijoSebastian/UR5/blob/main/pick_and_place/src/pick_place_tf.cpp)

- The code will first move the robot to "UP" pose then move to be right behind the detected April tag postions while keeping the same wrist orinetataion as before, pointing straight forward.

- Once the motion is complete the robot will go back to UP pose during which you can move the April tag to a new position and the whole cycle repeats 5 times.

  - Note that the Planning time has been set to a maximum of 20 seconds to allow for path optimisation. This has made noticable difference in the plans executed by the robot 

## Result

![First run](https://github.com/BijoSebastian/UR5/blob/main/Results/pose_track.gif)
