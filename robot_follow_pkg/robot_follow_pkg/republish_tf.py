#! /usr/bin/env python3
import time
# ~ import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from tf2_msgs.msg import TFMessage


class MinimalPublisher(Node):

	def __init__(self):	
		super().__init__('republish_tf', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
		
		self.ChildFrame = self.get_parameter_or('republish_tf_child_frame_id',  Parameter('str', Parameter.Type.STRING, 'base_link') )
		self.TargetFrame = self.get_parameter_or('republish_tf_frame_id',  Parameter('str', Parameter.Type.STRING, 'odom') )
		self.PreFixChild = self.get_parameter_or('republish_tf_child_prefix',  Parameter('str', Parameter.Type.STRING, 'a0') )
		self.PreFixTarget = self.get_parameter_or('republish_tf_prefix',  Parameter('str', Parameter.Type.STRING, '') )
		
		InputTopic = self.get_parameter_or('republish_tf_input_topic',  Parameter('str', Parameter.Type.STRING, '/a200_0000/tf') )
		OutputTopic = self.get_parameter_or('republish_tf_output_topic',  Parameter('str', Parameter.Type.STRING, '/tf') )
		# todo add offset
		
		# Vars 
		self.subscription = self.create_subscription(
			TFMessage,
			InputTopic.value,
			self.callback,
			100)
		self.subscription  # prevent unused variable warning
		self.publisher_ = self.create_publisher(TFMessage, OutputTopic.value, 100)
		# ~ self.publisher_ = self.create_publisher(TFMessage, '/tf', 100)
		self.timer_period = 1.0  # seconds
		# ~ self.timer = self.create_timer(self.timer_period, self.timer_callback)
		
		# ~ self.tf_buffer = Buffer()
		# ~ self.tf_listener = TransformListener(self.tf_buffer, self)
		
		self.get_logger().info('InputTopic: %s' % InputTopic.value )
		self.get_logger().info('OutputTopic: %s' % OutputTopic.value )
		self.get_logger().info('TargetFrame: %s' % self.TargetFrame.value )
		self.get_logger().info('ChildFrame: %s' % self.ChildFrame.value )
		self.get_logger().info('PreFixChild: %s' % self.PreFixChild.value )
		self.get_logger().info('PreFixTarget: %s' % self.PreFixTarget.value )
		self.get_logger().info('TF Republish ready' )

	def callback(self, msg):
		# ~ self.get_logger().info('I heard: "%s"' % msg.transforms)
		
		OutputMsg = TFMessage()
		for x in msg.transforms:
			if ((x.header.frame_id == self.TargetFrame.value) and (x.child_frame_id == self.ChildFrame.value)):
				if self.PreFixTarget.value != "": 
					x.header.frame_id = self.PreFixTarget.value + "_"  + x.header.frame_id
				else:
					x.header.frame_id = x.header.frame_id
					
				if self.PreFixChild.value != "": 
					x.child_frame_id =  self.PreFixChild.value + "_"  + x.child_frame_id
				else:
					x.child_frame_id =  x.child_frame_id
				OutputMsg.transforms.append(x)
			
		print(OutputMsg.transforms)
		
		self.publisher_.publish(OutputMsg)
		# ~ self.get_logger().info('Publishing: "%s"' % msg.transforms)
		
		# ~ time.sleep(2.)

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
