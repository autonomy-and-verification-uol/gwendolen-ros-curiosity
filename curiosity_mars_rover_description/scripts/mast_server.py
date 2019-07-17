#!/usr/bin/env python
import time
import rospy
import actionlib
import curiosity_mars_rover_description.msg
from std_msgs.msg import Float64
from std_msgs.msg import String



class CuriosityMarsRoverMast(object):
	#~ _feedback = curiosity_mars_rover_description.msg.CuriosityMastFeedback()
	_result = curiosity_mars_rover_description.msg.CuriosityMastResult()
	
	def __init__(self, name):
		rospy.loginfo("CuriosityRover Mast Initialising...")
        
		self.publishers_curiosity_d = {}
		self.controller_ns = "curiosity_mars_rover"
		self.controller_command = "command"
		self.controllers_list = [   "mast_p_joint_position_controller",
                                    "mast_02_joint_position_controller",
                                    "mast_cameras_joint_position_controller"
								]

		for controller_name in self.controllers_list:
			topic_name = "/"+self.controller_ns+"/"+controller_name+"/"+self.controller_command
			self.publishers_curiosity_d[controller_name] = rospy.Publisher(
				topic_name,
				Float64,
				queue_size=1)

		self.wait_publishers_to_be_ready()
		self.init_publisher_variables()
#		self.init_state()
		
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, curiosity_mars_rover_description.msg.CuriosityMastAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

		rospy.logwarn("CuriosityRover Mast ... READY")
		ag_pub = rospy.Publisher('curiosity_to_agent', String, queue_size=4, latch=True)
		ag_pub.publish("mast")
		
	def execute_cb(self, goal):
		mast_mode_requested = goal.mode
		self._result.result = self.set_mast_pose(mast_mode_requested)
		self._as.set_succeeded(self._result)

	def wait_publishers_to_be_ready(self):
		rate_wait = rospy.Rate(10)
		for controller_name, publisher_obj in self.publishers_curiosity_d.iteritems():
			publisher_ready = False
			while not publisher_ready:
				rospy.logwarn("Checking Publisher for ==>"+str(controller_name))
				pub_num = publisher_obj.get_num_connections()
				publisher_ready = (pub_num > 0)
				rate_wait.sleep()
			rospy.loginfo("Publisher ==>" + str(controller_name) + "...READY")
            
	def init_publisher_variables(self):
		"""
		We create variables for more pythonic access access to publishers
		and not need to access any more
		:return:
		"""
		# Get the publishers for mast
		self.mast_p = self.publishers_curiosity_d[self.controllers_list[0]]
		self.mast_02 = self.publishers_curiosity_d[self.controllers_list[1]]
		self.mast_cameras = self.publishers_curiosity_d[self.controllers_list[2]]

		# Init Messages
		self.mast_p_pos_msg = Float64()
		self.mast_02_pos_msg = Float64()
		self.mast_cameras_pos_msg = Float64()

	#~ def init_state(self):
		#~ self.set_mast_pose("close")

	def set_mast_pose(self, mode_name):
		if mode_name == "close" or mode_name == "open":
			if mode_name == "close":
				self.mast_p_pos_msg.data = 1.57
				self.mast_02_pos_msg.data = -1.57
				self.mast_cameras_pos_msg.data = 0.0
				rospy.logwarn("Closing CuriosityRover Mast")
			if mode_name == "open":
				self.mast_p_pos_msg.data = 0.0
				self.mast_02_pos_msg.data = -0.5
				self.mast_cameras_pos_msg.data = 0.0
				rospy.logwarn("Opening CuriosityRover Mast")
			self.mast_p.publish(self.mast_p_pos_msg)
			self.mast_02.publish(self.mast_02_pos_msg)
			self.mast_cameras.publish(self.mast_cameras_pos_msg)
			return True
		else:
			rospy.logerr("Requested a Not supported MAST POSE")
			return False


if __name__ == '__main__':
    rospy.init_node('curiosity_rover_mast_node')
    server = CuriosityMarsRoverMast(rospy.get_name())
    rospy.spin()


