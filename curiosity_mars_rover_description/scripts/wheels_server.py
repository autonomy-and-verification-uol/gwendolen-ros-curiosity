#!/usr/bin/env python
import time
import rospy
import actionlib
import curiosity_mars_rover_description.msg
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CuriosityMarsRoverWheels(object):
	#~ _feedback = curiosity_mars_rover_description.msg.CuriosityWheelsFeedback()
	_result = curiosity_mars_rover_description.msg.CuriosityWheelsResult()

	def __init__(self, name):
		rospy.loginfo("CuriosityRover Wheels Initialising...")

		self.publishers_curiosity_d = {}
		self.controller_ns = "curiosity_mars_rover"
		self.controller_command = "command"
		self.controllers_list = [   "back_wheel_L_joint_velocity_controller",
                                    "back_wheel_R_joint_velocity_controller",
                                    "front_wheel_L_joint_velocity_controller",
                                    "front_wheel_R_joint_velocity_controller",
                                    "middle_wheel_L_joint_velocity_controller",
                                    "middle_wheel_R_joint_velocity_controller",
                                    "suspension_arm_B2_L_joint_position_controller",
                                    "suspension_arm_B2_R_joint_position_controller",
                                    "suspension_arm_B_L_joint_position_controller",
                                    "suspension_arm_B_R_joint_position_controller",
                                    "suspension_arm_F_L_joint_position_controller",
                                    "suspension_arm_F_R_joint_position_controller",
                                    "suspension_steer_B_L_joint_position_controller",
                                    "suspension_steer_B_R_joint_position_controller",
                                    "suspension_steer_F_L_joint_position_controller",
                                    "suspension_steer_F_R_joint_position_controller"
								]

		for controller_name in self.controllers_list:
			topic_name = "/"+self.controller_ns+"/"+controller_name+"/"+self.controller_command
			self.publishers_curiosity_d[controller_name] = rospy.Publisher(
				topic_name,
				Float64,
				queue_size=1)

		self.wait_publishers_to_be_ready()
		self.init_publisher_variables()
		self.init_state()

		#~ self.cmd_vel_msg = Twist()
		#~ cmd_vel_topic = "/cmd_vel"
		#~ rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)

		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, curiosity_mars_rover_description.msg.CuriosityWheelsAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

		rospy.logwarn("CuriosityRover Wheels ... READY")
		ag_pub = rospy.Publisher('curiosity_to_agent', String, queue_size=4, latch=True)
		ag_pub.publish("wheels")

	#~ def cmd_vel_callback(self, msg):
		#~ self.cmd_vel_msg = msg

	def execute_cb(self, goal):
		wheels_action_requested = goal.mode
		wheels_distance_requested = goal.distance
		wheels_speed_requested = goal.speed
		if wheels_action_requested == "forward":
			rospy.logwarn("CuriosityRover moving forwards")
			self._result.result = self.move_forwards(wheels_distance_requested, wheels_speed_requested)
			#~ self._result.result = self.move_with_cmd_vel()
			self._as.set_succeeded(self._result)
		elif wheels_action_requested == "backward":
			rospy.logwarn("CuriosityRover moving backwards")
			self._result.result = self.move_backwards(wheels_distance_requested, wheels_speed_requested)
			#~ self._result.result = self.move_with_cmd_vel()
			self._as.set_succeeded(self._result)
		elif wheels_action_requested == "left":
			rospy.logwarn("CuriosityRover moving left")
			self._result.result = self.move_turn_left(wheels_distance_requested, wheels_speed_requested)
			#~ self._result.result = self.move_with_cmd_vel()
			self._as.set_succeeded(self._result)
		elif wheels_action_requested == "right":
			rospy.logwarn("CuriosityRover moving right")
			self._result.result = self.move_turn_right(wheels_distance_requested, wheels_speed_requested)
			#~ self._result.result = self.move_with_cmd_vel()
			self._as.set_succeeded(self._result)
		elif wheels_action_requested == "stop":
			rospy.logwarn("CuriosityRover stopping")
			self._result.result = self.move_turn_stop()
			#~ self._result.result = self.move_with_cmd_vel()
			self._as.set_succeeded(self._result)
		else:
			rospy.logerr("Requested a not supported wheels action")

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
		# Get the publishers for wheel speed
		self.back_wheel_L = self.publishers_curiosity_d[self.controllers_list[0]]
		self.back_wheel_R = self.publishers_curiosity_d[self.controllers_list[1]]
		self.front_wheel_L = self.publishers_curiosity_d[self.controllers_list[2]]
		self.front_wheel_R = self.publishers_curiosity_d[self.controllers_list[3]]
		self.middle_wheel_L = self.publishers_curiosity_d[self.controllers_list[4]]
		self.middle_wheel_R = self.publishers_curiosity_d[self.controllers_list[5]]
		# Get the publishers for suspension
		self.suspension_arm_B2_L = self.publishers_curiosity_d[self.controllers_list[6]]
		self.suspension_arm_B2_R = self.publishers_curiosity_d[self.controllers_list[7]]
		self.suspension_arm_B_L = self.publishers_curiosity_d[self.controllers_list[8]]
		self.suspension_arm_B_R = self.publishers_curiosity_d[self.controllers_list[9]]
		self.suspension_arm_F_L = self.publishers_curiosity_d[self.controllers_list[10]]
		self.suspension_arm_F_R = self.publishers_curiosity_d[self.controllers_list[11]]
		# Get the publishers for steering
		self.suspension_steer_B_L = self.publishers_curiosity_d[self.controllers_list[12]]
		self.suspension_steer_B_R = self.publishers_curiosity_d[self.controllers_list[13]]
		self.suspension_steer_F_L = self.publishers_curiosity_d[self.controllers_list[14]]
		self.suspension_steer_F_R = self.publishers_curiosity_d[self.controllers_list[15]]

		# Init Messages
		self.back_wheel_L_velocity_msg = Float64()
		self.back_wheel_R_velocity_msg = Float64()
		self.front_wheel_L_velocity_msg = Float64()
		self.front_wheel_R_velocity_msg = Float64()
		self.middle_wheel_L_velocity_msg = Float64()
		self.middle_wheel_R_velocity_msg = Float64()
		self.suspension_arm_B2_L_pos_msg = Float64()
		self.suspension_arm_B2_R_pos_msg = Float64()
		self.suspension_arm_B_L_pos_msg = Float64()
		self.suspension_arm_B_R_pos_msg = Float64()
		self.suspension_arm_F_L_pos_msg = Float64()
		self.suspension_arm_F_R_pos_msg = Float64()
		self.suspension_steer_B_L_pos_msg = Float64()
		self.suspension_steer_B_R_pos_msg = Float64()
		self.suspension_steer_F_L_pos_msg = Float64()
		self.suspension_steer_F_R_pos_msg = Float64()

	def init_state(self):
		self.set_suspension_mode("standard")
		self.set_turning_radius(None)
		self.set_wheels_speed(0.0)

	def set_suspension_mode(self, mode_name):
		if mode_name == "standard":
			self.suspension_arm_B2_L_pos_msg.data = -0.2
			self.suspension_arm_B2_R_pos_msg.data = -0.2
			self.suspension_arm_B_L_pos_msg.data = -0.2
			self.suspension_arm_B_R_pos_msg.data = -0.2
			self.suspension_arm_F_L_pos_msg.data = 0.2
			self.suspension_arm_F_R_pos_msg.data = 0.2
			self.suspension_arm_B2_L.publish(self.suspension_arm_B2_L_pos_msg)
			self.suspension_arm_B2_R.publish(self.suspension_arm_B2_R_pos_msg)
			self.suspension_arm_B_L.publish(self.suspension_arm_B_L_pos_msg)
			self.suspension_arm_B_R.publish(self.suspension_arm_B_R_pos_msg)
			self.suspension_arm_F_L.publish(self.suspension_arm_F_L_pos_msg)
			self.suspension_arm_F_R.publish(self.suspension_arm_F_R_pos_msg)

	def set_turning_radius(self, turn_radius):
		if not turn_radius:
			# We dont need Ackerman calculations, its not turn.
			self.suspension_steer_B_L_pos_msg.data = 0.0
			self.suspension_steer_B_R_pos_msg.data = 0.0
			self.suspension_steer_F_L_pos_msg.data = 0.0
			self.suspension_steer_F_R_pos_msg.data = 0.0
		else:
			# TODO: Ackerman needed here
			if turn_radius > 0.0:
				self.suspension_steer_B_L_pos_msg.data = -0.3
				self.suspension_steer_B_R_pos_msg.data = -0.3
				self.suspension_steer_F_L_pos_msg.data = 0.3
				self.suspension_steer_F_R_pos_msg.data = 0.3
			else:
				self.suspension_steer_B_L_pos_msg.data = 0.3
				self.suspension_steer_B_R_pos_msg.data = 0.3
				self.suspension_steer_F_L_pos_msg.data = -0.3
				self.suspension_steer_F_R_pos_msg.data = -0.3
		self.suspension_steer_B_L.publish(self.suspension_steer_B_L_pos_msg)
		self.suspension_steer_B_R.publish(self.suspension_steer_B_R_pos_msg)
		self.suspension_steer_F_L.publish(self.suspension_steer_F_L_pos_msg)
		self.suspension_steer_F_R.publish(self.suspension_steer_F_R_pos_msg)

	def set_wheels_speed(self, turning_speed):
		"""
		Sets the turning speed in radians per second
		:param turning_speed: In radians per second
		:return:
		"""
		# TODO: turning_speed for each wheel should change based on ackerman.
		self.back_wheel_L_velocity_msg.data = turning_speed
		self.back_wheel_R_velocity_msg.data = -1*turning_speed
		self.front_wheel_L_velocity_msg.data = turning_speed
		self.front_wheel_R_velocity_msg.data = -1*turning_speed
		self.middle_wheel_L_velocity_msg.data = turning_speed
		self.middle_wheel_R_velocity_msg.data = -1*turning_speed
		self.back_wheel_L.publish(self.back_wheel_L_velocity_msg)
		self.back_wheel_R.publish(self.back_wheel_R_velocity_msg)
		self.front_wheel_L.publish(self.front_wheel_L_velocity_msg)
		self.front_wheel_R.publish(self.front_wheel_R_velocity_msg)
		self.middle_wheel_L.publish(self.middle_wheel_L_velocity_msg)
		self.middle_wheel_R.publish(self.middle_wheel_R_velocity_msg)

	def move_forwards(self, distance, speed):
		self.set_turning_radius(None)
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		while(current_distance < distance):
			self.set_wheels_speed(speed)
			t1=rospy.Time.now().to_sec()
			current_distance= speed*(t1-t0)
		self.move_turn_stop()
		return True

	def move_backwards(self, distance, speed):
		self.set_turning_radius(None)
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		while(current_distance < distance):
			self.set_wheels_speed(speed)
			t1=rospy.Time.now().to_sec()
			current_distance= speed*(t1-t0)
		self.move_turn_stop()
		return True

	def move_turn_left(self, distance, speed):
		self.set_turning_radius(1.0)
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		while(current_distance < distance):
			self.set_wheels_speed(speed)
			t1=rospy.Time.now().to_sec()
			current_distance= speed*(t1-t0)
		self.move_turn_stop()
		return True

	def move_turn_right(self, distance, speed):
		self.set_turning_radius(-1.0)
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		while(current_distance < distance):
			self.set_wheels_speed(speed)
			t1=rospy.Time.now().to_sec()
			current_distance= speed*(t1-t0)
		self.move_turn_stop()
		return True

	def move_turn_stop(self):
		self.set_turning_radius(None)
		self.set_wheels_speed(0.0)
		time.sleep(10)
		return True

	#~ def move_with_cmd_vel(self):
		#~ wheel_speed = self.cmd_vel_msg.linear.x
		#~ turning_radius = self.cmd_vel_msg.angular.z
		#~ if turning_radius == 0.0:
			#~ turning_radius = None
		#~ rospy.logdebug("turning_radius="+str(turning_radius)+",wheel_speed="+str(wheel_speed))
		#~ self.set_turning_radius(turning_radius)
		#~ self.set_wheels_speed(wheel_speed)
		#~ return True

if __name__ == '__main__':
    rospy.init_node('curiosity_rover_wheels_node')
    server = CuriosityMarsRoverWheels(rospy.get_name())
    rospy.spin()
    #~ rate = rospy.Rate(10.0)
    #~ while not rospy.is_shutdown():
        #~ server.move_with_cmd_vel()
        #~ rate.sleep()
