#!/usr/bin/env python
import time
import rospy
import actionlib
import curiosity_mars_rover_description.msg
from std_msgs.msg import Float64
from std_msgs.msg import String



class CuriosityMarsRoverArm(object):
	#~ _feedback = curiosity_mars_rover_description.msg.CuriosityArmFeedback()
	_result = curiosity_mars_rover_description.msg.CuriosityArmResult()
	
	def __init__(self, name):
		rospy.loginfo("CuriosityRover Arm Initialising...")

		self.publishers_curiosity_d = {}
		self.controller_ns = "curiosity_mars_rover"
		self.controller_command = "command"
		self.controllers_list = [   "arm_01_joint_position_controller",
									"arm_02_joint_position_controller",
									"arm_03_joint_position_controller",
									"arm_04_joint_position_controller",
									"arm_tools_joint_position_controller"
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
		self._as = actionlib.SimpleActionServer(self._action_name, curiosity_mars_rover_description.msg.CuriosityArmAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

		rospy.logwarn("CuriosityRover Arm ... READY")
		self.ag_pub = rospy.Publisher('curiosity_to_agent', String, queue_size=4, latch=True)
		self.ag_pub.publish("arm")
		
	def execute_cb(self, goal):
		arm_mode_requested = goal.mode
		self._result.result = self.set_arm_pose(arm_mode_requested)
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
		# Get the publishers for arm
		self.arm_01 = self.publishers_curiosity_d[self.controllers_list[0]]
		self.arm_02 = self.publishers_curiosity_d[self.controllers_list[1]]
		self.arm_03 = self.publishers_curiosity_d[self.controllers_list[2]]
		self.arm_04 = self.publishers_curiosity_d[self.controllers_list[3]]
		self.arm_tools = self.publishers_curiosity_d[self.controllers_list[4]]

		# Init Messages
		self.arm_01_pos_msg = Float64()
		self.arm_02_pos_msg = Float64()
		self.arm_03_pos_msg = Float64()
		self.arm_04_pos_msg = Float64()
		self.arm_tools_pos_msg = Float64()

#	def init_state(self):
#		self.set_arm_pose("close")

	def set_arm_pose(self, mode_name):
		if mode_name == "close" or mode_name == "open":
			if mode_name == "close":
				self.arm_01_pos_msg.data = -1.57
				self.arm_02_pos_msg.data = -0.4
				self.arm_03_pos_msg.data = -1.1
				self.arm_04_pos_msg.data = -1.57
				self.arm_tools_pos_msg.data = -1.57
				rospy.logwarn("Closing CuriosityRover Arm")
			if mode_name == "open":
				self.arm_01_pos_msg.data = 0.0
				self.arm_02_pos_msg.data = 0.0
				self.arm_03_pos_msg.data = 0.0
				self.arm_04_pos_msg.data = 0.0
				self.arm_tools_pos_msg.data = 0.0
				rospy.logwarn("Opening CuriosityRover Arm")
			self.arm_01.publish(self.arm_01_pos_msg)
			self.arm_02.publish(self.arm_02_pos_msg)
			self.arm_03.publish(self.arm_03_pos_msg)
			self.arm_04.publish(self.arm_04_pos_msg)
			self.arm_tools.publish(self.arm_tools_pos_msg)
			return True
		else:
			rospy.logerr("Requested a Not supported ARM POSE")
			return False


if __name__ == '__main__':
    rospy.init_node('curiosity_rover_arm_node')
    server = CuriosityMarsRoverArm(rospy.get_name())
    rospy.spin()


