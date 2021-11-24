#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy, rospkg
import sys, os
import cv2
import glob
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_publisher:
	def __init__(self):

		# Params
		self.action_code = rospy.get_param('action_code', 'S001C003P005R001A013_rgb')
		self.package_dir     = rospkg.RosPack().get_path('play_video')
		self.input_dir      = rospy.get_param('input_dir', self.package_dir + '/rgb_frames/' + self.action_code + '/')
		
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher('image_color_rect', Image)

	def run(self):

		rate = rospy.Rate(10)
		
		for frame in sorted(glob.glob(self.input_dir+'*.png')):

			if not rospy.is_shutdown():
				cv_image = cv2.imread(frame)
				#cv2.imshow('img_win', cv_image)
				#cv2.waitKey(25)

				if cv_image is not None:
					ros_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
					self.image_pub.publish(ros_msg)

				rate.sleep()
			else:
				return


def main(args):
	rospy.init_node('image_publisher', anonymous=True)
	img_pub = image_publisher()
	img_pub.run()

if __name__=='__main__':

	main(sys.argv)


