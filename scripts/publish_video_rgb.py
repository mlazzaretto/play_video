#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy, rospkg
import sys, os
import cv2
import glob
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from play_video.msg import Action

### Publish RGB 1920x1080 frames ###

class image_publisher:
	def __init__(self):

		# Params
		self.action_code = rospy.get_param('action_code', 'S001C003P005R001A013')
		self.package_dir = rospkg.RosPack().get_path('play_video')
		self.input_dir = rospy.get_param('input_dir', self.package_dir + '/data/rgb_frames/orig_rgb/' + self.action_code + '_rgb/')
		
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher('image_color_rect', Image, queue_size=10)
		self.action_pub = rospy.Publisher('action_code', Action , queue_size=10)

		self.rgb_frames = []

		for frame in sorted(glob.glob(self.input_dir+'*.png')):
			cv_image = cv2.imread(frame)
			self.rgb_frames.append(cv_image)

	def run(self):

		rate = rospy.Rate(2)
		for i in range(len(self.rgb_frames)):

			if not rospy.is_shutdown():
				frame_rgb = self.rgb_frames[i]

				
				now = rospy.get_rostime()
				rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
				ros_msg_rgb = self.bridge.cv2_to_imgmsg(frame_rgb, 'bgr8')
				ros_msg_rgb.header.stamp.secs = now.secs
				ros_msg_rgb.header.stamp.nsecs = now.nsecs
				ros_msg_rgb.height = 1080
				ros_msg_rgb.width = 1920
				ros_msg_rgb.encoding = "bgr8"
				action_msg = Action()
				action_msg.action = self.action_code
				action_msg.header.stamp.secs = now.secs
				action_msg.header.stamp.nsecs = now.nsecs				
				
				self.action_pub.publish(action_msg)
				self.image_pub.publish(ros_msg_rgb)

				rate.sleep()
			else:
				return


def main(args):

	rospy.init_node('image_publisher', anonymous=True)
	img_pub = image_publisher()
	img_pub.run()
	rospy.spin()


if __name__=='__main__':

	main(sys.argv)


