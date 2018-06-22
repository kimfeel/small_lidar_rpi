#!/usr/bin/python
import roslib
import rospy
import tf
import time
import math

import os, struct, array
from fcntl import ioctl
import sys
from threading import Thread
from sensor_msgs.msg import LaserScan

step = 0
theta = 0 
step_for_pi = 2048
rotation_rate = 5.625 / 64 * (math.pi/180)

br = tf.TransformBroadcaster()
x = 0#pose.pose.position.x
y = 0#pose.pose.position.y
z = 0
delay = 0.1#time_for_quarter#0.1#5 #seconds
elapsed_time = 0

def steps_fun(data):
	global step
	step = data.data
	#print step

rospy.init_node('test')
from std_msgs.msg import Int16
tfListener = tf.TransformListener()
rospy.Subscriber("steps", Int16, steps_fun)

try:
	while not rospy.is_shutdown():
		theta = rotation_rate * step + math.pi/2	
		#print rotation_rate		
		#if step > step_for_pi :
		#	theta = theta + 0			
		if theta > 2*math.pi:
			theta = theta - 2*math.pi	
		start_time = time.time()
		br.sendTransform((x,y,z),tf.transformations.quaternion_from_euler(0,math.pi/2,theta),rospy.Time.now(),'laser','base_link')
		elapsed_time = time.time() - start_time

except KeyboardInterrupt:
	print "Stopped!\n"

