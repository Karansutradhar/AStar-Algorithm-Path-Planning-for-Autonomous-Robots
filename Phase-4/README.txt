READ ME for phase 4

to run video 1 or video 2 simulation follow the below steps

download my catkin_ws and cd into it

cd_make the workspace than then open a second terminal

source both of the terminals with $~/<path to ws>/devel/setup.bash

next in one terminal $roslaunch part4 turtlebot_world.launch

in the other terminal after gazebo finishes initializing run $rosrun part4 TurtleBot.py

depending on the simulation you'd like to run change the name of the txt file loaded in in catkin_ws/src/part4/src/scripts/TurtleBot.py line 49

if running simulations back to back, control c out of both terminals and then follow the above directions after making the change in "TurtleBot.py"
