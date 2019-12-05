#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
import curiosity_mars_rover_description.msg
from std_msgs.msg import String
from std_msgs.msg import Int64
from std_msgs.msg import Float64

def wheels_client(directionreq, distancereq, speedreq):
	client = actionlib.SimpleActionClient('curiosity_rover_wheels_node', curiosity_mars_rover_description.msg.CuriosityWheelsAction)
	client.wait_for_server()
	goal = curiosity_mars_rover_description.msg.CuriosityWheelsGoal(mode=directionreq, distance=distancereq, speed=speedreq)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()
		
def goal_wheels(msg):
	#rospy.logwarn("Mode %s " % (modewheels))
	try:
		result = wheels_client(msg.direction, msg.distance, msg.speed)
		ag_pub = rospy.Publisher('curiosity_to_agent', String, queue_size=4, latch=True)
		ag_pub.publish("done")
		rospy.loginfo("Wheels Action Result: %s" % (result.result))
	except rospy.ROSInterruptException:
		rospy.loginfo("program interrupted before completion", file=sys.stderr)	

if __name__ == '__main__':
        rospy.init_node('wheels_client_py')
        rospy.Subscriber("gwendolen_curiosity_wheels", curiosity_mars_rover_description.msg.Move3, goal_wheels)
        rospy.spin()
