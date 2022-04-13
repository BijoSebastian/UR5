# Instructions on adding IDS camera for April tag based pose estimation 

## Setup the camera

- Camera model used: UI-3060CP-C-HQ Rev.2

- Download the IDS Software Suite from [here](https://en.ids-imaging.com/downloads.html) for the corresponding camera hardware. Remember to choose Linux 64 bit under the operating system option. You will be required to create a free account with IDS for accessing the software. I used the archive file installation option

- Follow the instructions [here](https://en.ids-imaging.com/files/downloads/ids-software-suite/readme/readme-ids-software-suite-linux-4.95.2_EN.html#distributions) for extracting and installing the downloaded package. Remember to scroll down to the Archive set of instructions under the Installation section. I used the "run" script and no additional software packages were needed on my Ubunutu-20.04 machine.

- Finally connect the camera to your machine through USB/ethernet and the camera should have a solid green light, if it is recognized and working well. 


## Setting up the ueye_cam ROS package

- ueye_cam is a ROS interface for the uEye digital cameras. You can find the instructions for the same [here](http://wiki.ros.org/ueye_cam)

- To avoid messing up the already existing ur_ws, I recommend creating a new sperate workspace for the ueye_cam. Follow the Build instructions given [here](http://wiki.ros.org/ueye_cam) to get the ROS node installed

- With the camera connected execute the following commands in a terminal:

#Source the ueye_cam ws
source ~/Projects/ueye_ws/devel/setup.bash
roslaunch ueye_cam rgb8.launch

- To see all the topics being published, in a new terminal execute:
```
rostopic list
```
- In a new terminal launch Rviz 
```
rosrun rviz rviz 
```
Add a display for  image type and subscribe to the topic */camera/image_raw* to view the image being published
