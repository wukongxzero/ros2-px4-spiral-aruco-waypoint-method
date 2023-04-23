# spiral-aruco-drone-control

Here is the way to run it on your machine, thankfully ros2 has deleted the entirety of sdf written launch files .. which means consequent pcs have better support but the issue with using ros2 humble was that it needs gazebo garden no gazebo classic and most of the libraries have dependency conflicts, i tried and tested the thing it sucks a lot.

so here are the requirements to run the code:

system: ubuntu 22.04
ros: ros2; foxy.

install ros2 foxy along with px4 by going throught the docs.

install mavsdk from https://mavsdk.mavlink.io/main/en/

create 
mkdir ros2_ws
cd ros2_ws 
and then, mkdir src
run colcon build 

after running colcon build get into the directory src

clone this repository
git clone https://github.com/wukongxzero/spiral-aruco-drone-control

change directory into ros2_ws

click on colcon build --symlink-install 

Here is the tricky part and what i could not build due to time constraints, and my system has been pretty bad to utilize xrce-fast-dds protocol:

for each problem , you will have to use three terminals for this:

first things first xrce-fast-dds, rtps should be properly built into your system. i occurred dependency issues,which will take some time to figure out.
regardless, 

here is what you need to launch:

for waypoint navigation:
run :
terminal 1: 
cd PX4-Autopilot 
make px4_sitl gazebo 

terminal 2:
run:
micro-ros-agent udp4 --port 8888

what this will do is , it will run micro-ros-agent which will perform mapping between XRCE-DDS and RTPS. such that PX4 micro-rtps client is able to communicate with ROS2 NODES,

terminal 3:
ros2 launch drone_control waypoint.launch.py


for spiral path planning :
terminal 1 and 2 is the same but for terminal 3 :
ros2 launch drone_control spiral.launch.py


for aruco , we use opencv to code the node:

terminal 1:
cd PX4-Autopilot 
make px4_sitl make px4_sitl gazebo_typhoon_h480

terminal 2:
micro-ros-agent udp4 --port 8888

terminal 3:
ros2 launch drone_control aruco.launch.py


i am sure this is the right way to write the run files after installing all dependencies.
