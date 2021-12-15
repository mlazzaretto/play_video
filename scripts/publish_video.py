#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy, rospkg
import sys, os
import cv2
import glob
import numpy as np
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CameraInfo
from play_video.msg import Action
from cv_bridge import CvBridge, CvBridgeError

#RGB_PATH = rospkg.RosPack().get_path('play_video')+ '/data/nturgb+d_rgb_frames/'
#DEPTH_PATH = rospkg.RosPack().get_path('play_video') + '/data/nturgb+d_depth_masked_s1s2/'
RGB_PATH = rospkg.RosPack().get_path('play_video')+ '/data/rgb_frames/aff_rgb/'
DEPTH_PATH = rospkg.RosPack().get_path('play_video') + '/data/depth_frames/'

### Publish rgb frames and depth frames from folder, along with action code ###

class image_publisher:
	def __init__(self, action_code):

		# Params
		self.action_code = rospy.get_param('action_code', action_code)
		self.package_dir     = rospkg.RosPack().get_path('play_video')
		self.input_dir      = rospy.get_param('input_dir', RGB_PATH + self.action_code + '_rgb/')
		self.input_depth_dir = rospy.get_param('input_depth_dir', DEPTH_PATH + self.action_code + '/')
		
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher('image_color_rect', Image, queue_size=10)
		self.depth_pub = rospy.Publisher('image_depth_rect', Image, queue_size=10)
		self.cameraInfo_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
		self.action_pub = rospy.Publisher('action_code', Action , queue_size=10)

		self.rgb_frames = []
		self.depth_frames = []

		for frame in sorted(glob.glob(self.input_dir+'*.png')):
			print(frame)
			cv_image = cv2.imread(frame)
			self.rgb_frames.append(cv_image)

		for frame in sorted(glob.glob(self.input_depth_dir+'*.png')):
			cv_image = cv2.imread(frame, cv2.IMREAD_GRAYSCALE)
			self.depth_frames.append(cv_image)


		#scaling_factor = 1080/424	

		self.cam_info_msg = CameraInfo()
		self.cam_info_msg.width = 512
		self.cam_info_msg.height = 424	
		
				
		fx = 365.481
		fy = 365.481
		cx = 257.346
		cy = 210.347
		self.cam_info_msg.K = np.array([fx, 0, cx,
										0, fy, cy,
										0, 0, 1])	
		

	def run(self):

		rate = rospy.Rate(2)
		for i in range(len(self.rgb_frames)):

			if not rospy.is_shutdown():
				frame_rgb = self.rgb_frames[i]
				frame_depth = self.depth_frames[i]

				#if frame_rgb is not None:
				now = rospy.get_rostime()
				rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
				ros_msg_rgb = self.bridge.cv2_to_imgmsg(frame_rgb, 'bgr8')
				ros_msg_depth = self.bridge.cv2_to_imgmsg(frame_depth, 'mono8')
				ros_msg_rgb.header.stamp.secs = now.secs
				ros_msg_rgb.header.stamp.nsecs = now.nsecs
				ros_msg_rgb.height = 424
				ros_msg_rgb.width = 512
				ros_msg_rgb.encoding = "bgr8"
				ros_msg_depth.header.stamp.secs = now.secs
				ros_msg_depth.header.stamp.nsecs = now.nsecs
				ros_msg_depth.height = 424
				ros_msg_depth.width = 512
				ros_msg_depth.encoding = "mono8"

				action_msg = Action()
				action_msg.action = self.action_code
				action_msg.header.stamp.secs = now.secs
				action_msg.header.stamp.nsecs = now.nsecs

				
				self.action_pub.publish(action_msg)
				self.image_pub.publish(ros_msg_rgb)
				self.depth_pub.publish(ros_msg_depth)
				self.cameraInfo_pub.publish(self.cam_info_msg)

				rate.sleep()
			else:
				return


def main(args):

	rospy.init_node('image_publisher', anonymous=True)

	for folder in sorted(glob.glob(RGB_PATH+'*C003P005*/')):
	
		action = folder.split('/')[-2]
		action_code = action.split('_')[0]
		print('***********ACTION CODE:',action_code,'****************')
		img_pub = image_publisher(action_code)
		img_pub.run()
	
	rospy.spin()

if __name__=='__main__':

	main(sys.argv)

