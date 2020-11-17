#!/usr/bin/env python
import rospy
import numpy as np
import tf
import math
import geometry_msgs.msg
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class waypoint(object):
	def __init__(self):
		self.path = Marker	()
		self.marker_id = 1
		rospy.init_node('echoer')
		rospy.Subscriber("/clicked_point", geometry_msgs.msg.PointStamped, self.get_way_point)
		self.publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 10)

	# fetch clicked way points
	def get_way_point(self, msg):
		# display way points and path on the map
		self.display_way_point(msg.point.x,msg.point.y, msg.point.z)
		self.display_path(msg.point.x,msg.point.y, msg.point.z)

	# display way points on the map
	def display_way_point(self,x,y,z):
		points = Marker()		
		points.header.frame_id = "world"	# publish path in map frame		
		points.type = points.POINTS
		points.action = points.ADD
		points.lifetime = rospy.Duration(0)
		points.id = self.marker_id
		self.marker_id += 1
		points.scale.x = 0.1
		points.scale.y = 0.1
		points.scale.z = 0.1	
		points.color.a = 1.0
		points.color.r = 0.0
		points.color.g = 0.0
		points.color.b = 1.0
		points.pose.orientation.w = 1.0

		point = Point()
		point.x = x
		point.y = y
		point.z = z

		points.points.append(point);
		# Publish the MarkerArray
		self.publisher.publish(points)

	# display path between way points on the map
	def display_path(self,x,y,z):
		self.path.header.frame_id = "world"	# publish path in map frame
		self.path.type = self.path.LINE_STRIP
		self.path.action = self.path.ADD
		self.path.lifetime = rospy.Duration(0)
		self.path.id = 0
		self.path.scale.x = 0.05
		self.path.color.a = 1.0
		self.path.color.r = 0.0
		self.path.color.g = 1.0
		self.path.color.b = 0.0
		self.path.pose.orientation.w = 1.0

		point = Point()
		point.x = x
		point.y = y
		point.z = z

		self.path.points.append(point);
		# Publish the path
		self.publisher.publish(self.path)

	def run(self):
		rospy.spin()

if __name__ == '__main__':
	print "*********** waypoint.py: read and display way point on the map ***********"
	waypoint().run()
