#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
import curiosity_mars_rover_description.msg
from std_msgs.msg import String

def arm_client(modereq):
	client = actionlib.SimpleActionClient('curiosity_rover_arm_node', curiosity_mars_rover_description.msg.CuriosityArmAction)
	client.wait_for_server()
	goal = curiosity_mars_rover_description.msg.CuriosityArmGoal(mode=modereq)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()

def goal_arm(modereq):
	try:
		result = arm_client(modereq.data)
		rospy.loginfo("Arm Action Result: %s" % (result.result))
		ag_pub = rospy.Publisher('curiosity_to_agent', String, queue_size=4, latch=True)
		ag_pub.publish('arm(' + modereq.data + ')')
	except rospy.ROSInterruptException:
		rospy.loginfo("program interrupted before completion", file=sys.stderr)

if __name__ == '__main__':
	rospy.init_node('arm_client_py')
	rospy.Subscriber("gwendolen_curiosity_arm", String, goal_arm)
	rospy.spin()
