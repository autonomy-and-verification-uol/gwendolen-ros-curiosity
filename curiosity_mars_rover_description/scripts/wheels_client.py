#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
import curiosity_mars_rover_description.msg
from std_msgs.msg import String
from std_msgs.msg import Int64

modewheels = ""

def wheels_client(modereq, distancereq):
	client = actionlib.SimpleActionClient('curiosity_rover_wheels_node', curiosity_mars_rover_description.msg.CuriosityWheelsAction)
	client.wait_for_server()
	goal = curiosity_mars_rover_description.msg.CuriosityWheelsGoal(mode=modereq, distance=distancereq)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()
		
def goal_wheels(modereq):
	global modewheels 
	modewheels = modereq.data
#	rospy.logwarn("Mode %s " % (modewheels))
		
def distance_wheels(distancereq):
	global modewheels
	try:
#		result = wheels_client(modereq)
#		rospy.logwarn("Mode requested %s Distance requested %d" % (modewheels, distancereq.data))
		result = wheels_client(modewheels, distancereq.data)
		ag_pub = rospy.Publisher('curiosity_to_agent', String, queue_size=4, latch=True)
		ag_pub.publish("done")
		rospy.loginfo("Wheels Action Result: %s" % (result.result))
	except rospy.ROSInterruptException:
		rospy.loginfo("program interrupted before completion", file=sys.stderr)

if __name__ == '__main__':
        rospy.init_node('wheels_client_py')
#        goal_wheels("forward")
        rospy.Subscriber("gwendolen_curiosity_wheels", String, goal_wheels)
        rospy.Subscriber("gwendolen_curiosity_wheels_distance", Int64, distance_wheels)
        rospy.spin()
