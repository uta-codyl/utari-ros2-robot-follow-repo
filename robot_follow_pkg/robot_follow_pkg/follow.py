#! /usr/bin/env python3
import time
# ~ import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf_transformations import euler_from_quaternion

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# ~ from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
import tf2_geometry_msgs

class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('robot_follow', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
		
		
		self.prefix0 = self.get_parameter_or('prefix0',  Parameter('str', Parameter.Type.STRING, 'a0_') )
		self.prefix1 = self.get_parameter_or('prefix1',  Parameter('str', Parameter.Type.STRING, 'a1_') )
		
		OutputTopicCmdVel = self.get_parameter_or('output_vel_topic',  Parameter('str', Parameter.Type.STRING, '/a200_0001/cmd_vel') )
		OutputTopicGoal = self.get_parameter_or('output_goal_topic',  Parameter('str', Parameter.Type.STRING, '/a200_0001/goal_pose') )
		StopWithNoMsg = self.get_parameter_or('no_update_stop',  Parameter('bool', Parameter.Type.BOOL, True) )
		self.UseNavGoal = self.get_parameter_or('use_nav_goal',  Parameter('bool', Parameter.Type.BOOL, True) )
		StartEnabled = self.get_parameter_or('start_enabled',  Parameter('bool', Parameter.Type.BOOL, True) )
		# todo add offset
		
		# Vars 
		self.Enable = StartEnabled.value
		self.Goal = PointStamped
		self.Timeout = 0.5 # seconds
		
		self.FromFrame = self.prefix0.value + "base_link"
		if self.UseNavGoal.value:
			self.ToFrame = "map"
		else:
			self.ToFrame = self.prefix1.value + "base_link"
		self.AcceptRadius = 2.0
		self.P_rot = 0.6
		self.P_lin = 0.15
		
		if self.UseNavGoal.value:
			self._action_client = ActionClient(self, NavigateToPose, '/a200_0001/navigate_to_pose')
			
			
			# ~ self.publisher_goals = self.create_publisher(PoseStamped, OutputTopicGoal.value, 1)
			# ~ self.timer_period = 5.0   # seconds
			self.timer_period = 12.0   # seconds
		else: # p controller 
			self.publisher_cmd_vel = self.create_publisher(Twist, OutputTopicCmdVel.value, 1)
			self.timer_period = 1.0 / 20.0  # seconds
		
		self.timer = self.create_timer(self.timer_period, self.timer_callback)
		self.FailCount = 0
		
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		
		# Creates a subscriber
		self.subscriber_ = self.create_subscription(Bool, "follow/enable", self.listener_callback, 1)
		
		# ~ self.get_logger().info('InputTopic: %s' % InputTopic.value )
		# ~ self.get_logger().info('TargetFrame: %s' % self.TargetFrame.value )
		self.get_logger().info('FromFrame: %s' % self.FromFrame )
		self.get_logger().info('ToFrame: %s' % self.ToFrame )
		self.get_logger().info('OutputTopicCmdVel: %s' % OutputTopicCmdVel.value )
		self.get_logger().info('OutputTopicGoal: %s' % OutputTopicGoal.value )
		self.get_logger().info('StopWithNoMsg: %d' % StopWithNoMsg.value )
		self.get_logger().info('UseNavGoal: %d' % self.UseNavGoal.value )
		self.get_logger().info('Robot Follow ready' )
		
		self.timer_callback()
		
	def listener_callback(self, msg):
		self.get_logger().info('Enable msg: "%s"' % msg.data)
		self.Enable = msg.data
		
	def send_nav_goal(self, GoalPose):
		goal_msg = NavigateToPose.Goal()
		goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
		# ~ goal_msg.pose.header.frame_id = self.FromFrame
		# ~ goal_msg.pose.header.frame_id = self.ToFrame
		# ~ goal_msg.pose.pose.position.x = -2.0
		# ~ goal_msg.pose.pose.position.y =  0.0
		# ~ goal_msg.pose.pose.position.x = X
		# ~ goal_msg.pose.pose.position.y = Y
		goal_msg.pose = GoalPose
		
		self._action_client.wait_for_server()
		return self._action_client.send_goal_async(goal_msg)
		

	def timer_callback(self):
		self.get_logger().info('Timer callback ... ' )
		if self.Enable:
			t = self.tf_buffer
			try:
				Pt = PoseStamped()
				Pt.header.frame_id = self.FromFrame
				# ~ Pt.header.stamp = self.get_clock().now().to_msg()
				if self.UseNavGoal.value:
					# transform point behind lead robot to the map from
					Pt.pose.position.x = -2.0
				# ~ else:
					# lookup transform of lead robot from current robot frame
					# ~ t = self.tf_buffer.lookup_transform
					# ~ (
						# ~ self.ToFrame,
						# ~ self.FromFrame,
						# ~ rclpy.time.Time()
					# ~ )
				t = self.tf_buffer.transform(
						Pt,
						self.ToFrame
					)
					
				self.get_logger().info(
					# ~ f'Transform {self.FromFrame} to {self.ToFrame}: \n{t.transform}. Time {t.header.stamp}')
					f'Transform {self.FromFrame} to {self.ToFrame}: \n{t}. ')#Time {t.header.stamp}')
				self.FailCount = 0
			except TransformException as ex:
				self.get_logger().info(
					f'Could not transform {self.FromFrame} to {self.ToFrame}: {ex}')
				self.FailCount = self.FailCount + 1
				# ~ time.sleep(2.0)
				return
			
			if self.FailCount < 10:
				# there seems to be a major discrepancy in time, possibly due to sim time ?
				# ~ self.get_logger().info(f'Age of message: {(self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds*(10.0**-9.0)} ')
				# ~ self.get_logger().info(f'Now: {(self.get_clock().now().nanoseconds)*(10.0**-9.0)} ')
				# ~ self.get_logger().info(f'stamp: {Time.from_msg(t.header.stamp).nanoseconds*(10.0**-9.0)} ')
				# ~ if (StopWithNoMsg and ((self.get_clock().now() - t.header.stamp) < 2.0)) :
				
				if self.UseNavGoal.value:
					self.get_logger().info( f'Calling goal action. Goal: {(t)}')
					
					future =self.send_nav_goal(t)
					# ~ rclpy.spin_until_future_complete(self, future)
					
					
				else: # P controller
					# transform is already in robot frame is already error
					# ~ yaw = np.arctan2(t.transform.translation.y, t.transform.translation.x)
					# check if error is below acceptance radius
					# ~ if np.linalg.norm([t.transform.translation.y, t.transform.translation.x]) > self.AcceptRadius:
						# ~ yaw = np.arctan2(t.transform.translation.y, t.transform.translation.x)
					LinearError = np.linalg.norm([t.pose.position.y, t.pose.position.x])
					if LinearError > self.AcceptRadius:
						yaw = np.arctan2(t.pose.position.y, t.pose.position.x)
						
						Vel = Twist()
						Vel.angular.z = self.P_rot * yaw
						Vel.linear.x = self.P_lin * t.pose.position.x
						self.publisher_cmd_vel.publish(Vel)
					else:
						self.get_logger().info(f'Linear Error is low. Stopping robot. Linear Error {LinearError}.')
						Vel = Twist() # pub zeros
						self.publisher_cmd_vel.publish(Vel)
						yaw = np.arctan2(t.pose.position.y, t.pose.position.x)
						time.sleep(2.0)
					self.get_logger().info(f'Error X: {t.pose.position.x} meters. Y: {t.pose.position.y} meters. Yaw: {yaw} radians. ')
				
			else:
				self.get_logger().info("Failed to find location transform too many times. Disabling follow.")
				self.Enable = False
				return
		
		
		# ~ msg = Twist()
		# ~ msg.data = 'Hello World: %d' % self.i
		# ~ self.publisher_.publish(msg)
		# ~ self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	rclpy.spin(minimal_publisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
