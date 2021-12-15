#!/usr/bin/env python
from __future__ import division
import sys, os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import glob, re
import rospy
import message_filters
from std_msgs.msg import Header
from hiros_skeleton_msgs.msg import SkeletonGroup, Skeleton, Marker, Link, KinematicState
from geometry_msgs.msg import Pose, Point
from play_video.msg import Action


#convert from rgb to depth frame
rgb_to_depth_affine_transforms = dict(
    C001=np.array([[3.45638311e-01, 2.79844266e-03, -8.22281898e+01],
                   [-1.37185375e-03, 3.46949734e-01, 1.30882644e+01]]),

    C002=np.array([[3.42938209e-01, 8.72629655e-04, -7.28786114e+01],
                   [3.43287830e-04, 3.43578203e-01, 1.75767495e+01]]),

    C003=np.array([[3.45121348e-01, 8.53232038e-04, -7.33328852e+01],
                   [1.51167845e-03, 3.45115132e-01, 2.22178592e+01]]),
)

depth_to_rgb_affine_transforms = dict(
    C001=np.array([[2.89310518e+00, -2.33353370e-02, 2.38200221e+02],
                   [1.14394588e-02, 2.88216964e+00, -3.67819523e+01]]),

    C002=np.array([[2.90778446e+00, -1.04633946e-02, 2.15505801e+02],
                   [-3.43830682e-03, 2.91094100e+00, -5.13416831e+01]]),

    C003=np.array([[2.89756295e+00, -7.16367761e-03, 2.12645813e+02],
                   [-1.26919485e-02, 2.89761514e+00, -6.53095423e+01]]),
)

#,
#                   [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]),

fx = 365.481
fy = 365.481
cx = 257.346
cy = 210.347

### Take message of skeleton detected on RGB images and send message of affine transformed skeleton to adapt it to depth frames ###

class skeleton_to_depth():

	def __init__(self):

		rospy.init_node('skeleton_to_depth', anonymous=True)
		self.action_sub = message_filters.Subscriber('/action_code', Action)
		self.skeleton_sub = message_filters.Subscriber('/hiros/opw/node_01/skeleton_group', SkeletonGroup)
		self.pub = rospy.Publisher('/hiros/opw/node_01/res_skeleton_group', SkeletonGroup, queue_size=10)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.action_sub, self.skeleton_sub], 10, 0.5, allow_headerless=False)
		self.ts.registerCallback(self.callback) 


	def callback(self, action_msg, in_skel_msg):

		print('***************Resizing 2D skeleton***************')

		action_code = action_msg.action
		cam_code= action_code[4:8]
		A = rgb_to_depth_affine_transforms[cam_code]

		out_skel_msg = in_skel_msg

		for skeleton in out_skel_msg.skeletons:

			for marker in skeleton.markers:
				x = marker.center.pose.position.x
				y = marker.center.pose.position.y
				point_aff = np.matmul(A, np.array([[x],[y],[1]]))
				marker.center.pose.position.x = point_aff[0]
				marker.center.pose.position.y = point_aff[1]

			for link in skeleton.links:
				x = link.center.pose.position.x
				y = link.center.pose.position.y
				point_aff = np.matmul(A, np.array([[x],[y],[1]]))
				link.center.pose.position.x = point_aff[0]
				link.center.pose.position.y = point_aff[1]

		self.pub.publish(out_skel_msg)

	def spin(self):
		try:
			rospy.spin()
		except KeyboardInterrupt:
			print("Shutting down")

def main():

	affine_transform_node = skeleton_to_depth()
	affine_transform_node.spin()

if __name__ == '__main__':

	main()
