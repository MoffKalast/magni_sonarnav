#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
import actionlib
import math
import tf2_ros
import numpy as np
import random

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Vector3Stamped, Vector3
from sensor_msgs.msg import Range

from copy import deepcopy
from tf2_geometry_msgs import do_transform_vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def clamp(val, minval, maxval):
	if val < minval:
		return minval
	if val > maxval:
		return maxval
	return val

class RandomWalk:

	def __init__(self):
		rospy.init_node('magni_randomwalk', anonymous=False)
		rospy.on_shutdown(self.cleanup)
		self.running = True

		self.buffer = tf2_ros.Buffer(rospy.Time(30))

		self.listener = tf2_ros.TransformListener(self.buffer)
		self.broadcaster = tf2_ros.TransformBroadcaster()

		self.ranges = []
		for i in range(0,5):
			self.ranges.append(4)

		self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.sonardir = []
		for i in range(0,5):
			self.sonardir.append(self.get_forward("sonar_"+str(i)))

		self.sonar_sub = rospy.Subscriber('/sonars', Range, self.sonars)

		self.rand_angle = 0
		self.prominent_turn = 0
		self.avoid = rospy.Time.now()
		self.avoid_time = rospy.Duration(secs=3)

	def get_forward(self, frame):
		try:

			time = rospy.Time.now()
			while not self.buffer.can_transform(frame, "base_link", time, rospy.Duration(0.5)):
				time = rospy.Time.now()
				rospy.logwarn("Waiting for "+frame+"...")

				if not self.running:
					sys.exit(0)

			tf = self.buffer.lookup_transform(frame, "base_link", time, rospy.Duration(1.0))

			v = Vector3Stamped()
			v.vector.x = 0
			v.vector.y = 1
			v.vector.z = 0

			t = TransformStamped()
			t.transform.rotation.x = tf.transform.rotation.x
			t.transform.rotation.y = tf.transform.rotation.y
			t.transform.rotation.z = tf.transform.rotation.z
			t.transform.rotation.w = tf.transform.rotation.w

			return do_transform_vector3(v, t).vector

		except:
			rospy.logfatal("Sonar transforms not found.")
			sys.exit(0)

	def sonars(self, msg):
		frameid = int(msg.header.frame_id[-1])
		self.sonarmsg(msg.range, self.sonardir[frameid])

	#4  90 4
	#3  45 1 left
	#2   0 3
	#1 -45 2 right
	#0 -90 0

	def sonarmsg(self, rrange, ddir):
		deg = int(round((math.atan2(ddir.x,ddir.y)*180)/math.pi))
		self.ranges[int(deg/45)+2] = rrange

		mindist = 4.0
		for i in [1,2,3]:
			if self.ranges[i] < mindist:
				mindist = self.ranges[i]

		minside = self.ranges[0]
		if minside > self.ranges[4]:
			minside = self.ranges[4]

		turn = clamp(((self.ranges[3] - self.ranges[1])/4.0) + self.rand_angle, -0.5, 0.5)

		if mindist < 0.4:
			self.twist(-0.2,0)
			self.avoid = rospy.Time.now()
		elif mindist < 0.5 or rospy.Time.now() - self.avoid < self.avoid_time:
			if minside > 0.4:
				self.twist(0, math.copysign(0.5,self.prominent_turn))
			else:
				self.twist(-0.2, 0)
		else:
			self.prominent_turn = self.prominent_turn * 0.95 + turn * 0.05
			self.twist(0.3, turn)

		print(self.prominent_turn)

		#self.rand_turn = random.choice([-1,1])

		self.rand_angle += (random.random() - 0.5)*0.05
		self.rand_angle = clamp(self.rand_angle,-0.3,0.3)

	def twist(self, linear, angular):
		cmd = Twist()
		cmd.linear.x = linear
		cmd.angular.z = angular
		self.velocity_publisher.publish(cmd)

	def cleanup(self):
		self.running = False


try:
	t = RandomWalk()
	rospy.spin()

except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
