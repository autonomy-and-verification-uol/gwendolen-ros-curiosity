#!/usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
import curiosity_mars_rover_description.msg
from std_msgs.msg import String

def mast_client(modereq):
	client = actionlib.SimpleActionClient('curiosity_rover_mast_node', curiosity_mars_rover_description.msg.CuriosityMastAction)
	client.wait_for_server()
	goal = curiosity_mars_rover_description.msg.CuriosityMastGoal(mode=modereq)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()
		
def goal_mast(modereq):
	try:
		result = mast_client(modereq.data)
		rospy.loginfo("Mast Action Result: %s" % (result.result))
	except rospy.ROSInterruptException:
		rospy.loginfo("program interrupted before completion", file=sys.stderr)

if __name__ == '__main__':
        rospy.init_node('mast_client_py')
        rospy.Subscriber("gwendolen_curiosity_mast", String, goal_mast)
        rospy.spin()
