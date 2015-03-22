#!/usr/bin/env python
import roslib; 
import rospy

from create_node.msg import TurtlebotSensorState

from geometry_msgs.msg import Point32
from sensor_msgs.msg import   LaserScan
import os, time
import thread
NUM_SONAR_POINTS = 40
class LaserPing(object):
	def __init__(self, name, rate, frame_id="/base_link",  **kwargs):
		self.wall_signal = 0
		self.cliff_left_signal = 0
		self.cliff_front_left_signal = 0
		self.cliff_front_right_signal = 0
		self.cliff_right_signal = 0
		self.name = name
		self.rate = rate
		self.frame_id = frame_id
		self.t_delta = rospy.Duration(1.0 / self.rate)
		self.t_next = rospy.Time.now() + self.t_delta
		self.msg = LaserScan()
		self.msg.angle_min = -0.3
		self.msg.angle_max = 0.3
		self.msg.angle_increment = (self.msg.angle_max - self.msg.angle_min)/NUM_SONAR_POINTS #width / number of samples
		self.msg.time_increment = 0.0
		self.msg.range_min = 0.0
		self.msg.range_max = 3.0 
		self.msg.header.frame_id = self.frame_id
		self.pub = rospy.Publisher(self.name, LaserScan)
		rospy.Subscriber("/mobile_base/sensors/core", TurtlebotSensorState, self.callback)
	def poll(self):
		now = rospy.Time.now()
		if now > self.t_next:
			try:
				#self.pub_value("wall_sensor_link",self.wall_signal)
				#self.pub_value("left_cliff_sensor_link",self.cliff_left_signal)
				self.pub_value("laser", self.wall_signal)
			except:
				return
			self.t_next = now + self.t_delta
	def pub_value(self, frame_id, signal):
		self.msg.header.frame_id = frame_id
		self.msg.scan_time = 1.0/30.0
		# Convert signal cm to meters for ROS
		distance = signal / 100.0
		self.msg.ranges = [distance] * NUM_SONAR_POINTS
		# Add a timestamp and publish the message
		self.msg.header.stamp = rospy.Time.now()
		self.pub.publish(self.msg)

	def callback(self, data):
	    	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.wall_signal)
		self.wall_signal = data.wall_signal
		self.cliff_left_signal = data.cliff_left_signal 
		self.cliff_front_left_signal = data.cliff_front_left_signal 
		self.cliff_front_right_signal = data.cliff_front_right_signal 
		self.cliff_right_signal = data.cliff_right_signal 

if __name__ == '__main__':
    try:
        rospy.init_node('sonic_scan', log_level=rospy.DEBUG)
	sensor = LaserPing('sonic_scan', 12)
        while not rospy.is_shutdown():
		sensor.poll()

    except rospy.ROSInterruptException:
        pass




