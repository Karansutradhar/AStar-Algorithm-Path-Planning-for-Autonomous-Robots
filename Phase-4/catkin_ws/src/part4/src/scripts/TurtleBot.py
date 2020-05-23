#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
from geometry_msgs.msg import Twist
import math

class move():
    def __init__(self):
        # initiliaze
        rospy.init_node('GoForward', anonymous=False)

	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     


	cmds = []
	wheel_radius = 0.076/2
	wheel_base =(0.354/2)
	# change file to either video1-rpms.txt or video2-rpms.txt
	file = open("video1-rpms.txt", "r") 
	for line in file:
	    rpmR,rpmL = line.split(",")
	    rpmR = float(rpmR)
	    rpmL = float(rpmL)
	    velR = 2.0*math.pi*wheel_radius*rpmR/60.0
	    velL = 2.0*math.pi*wheel_radius*rpmL/60.0
	    v = (wheel_radius/2) * (velL + velR)
	    z = (velR-velL)/wheel_base
	    rospy.loginfo("linear x: " + str(v)+" angular z: " + str(z))
	    cmds.append([v,z])
	    # needed to fix shortened path
	    cmds.append([v,z])
	cmds.reverse()
	#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 2 HZ
        r = rospy.Rate(2);

        # Twist is a datatype for velocity
        move_cmd = Twist()
	
	
	
	count = 0
	# as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
	    # publish the velocity
            move_cmd.linear.x = cmds[count][0]
	    move_cmd.angular.z = cmds[count][1]
	    self.cmd_vel.publish(move_cmd)
	    rospy.loginfo(str(cmds[count][0])+str(cmds[count][1]))
    	    count +=1
	    # wait for .5 seconds (2 HZ) and publish again
            r.sleep()
                        
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        move()
    except:
        rospy.loginfo("move node terminated.")

