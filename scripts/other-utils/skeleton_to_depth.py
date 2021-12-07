#!/usr/bin/env python
from __future__ import division
import sys, os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import glob, re
import rospy
from std_msgs.msg import Header
from hiros_skeleton_msgs.msg import SkeletonGroup, Skeleton, Marker, Link, KinematicState
from geometry_msgs.msg import Pose, Point


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

#A_CODE = 'S001C003P005R001A013'
SKEL_PATH = '../../data/openpose_skeletons/'
DEPTH_PATH = '../depth_frames/'
OUT_PATH = '../../data/openpose_skeletons/affine/'

### Read skeleton file (detected on 1920x1080 rgb images) and create messages with transformed skeleton (adapted to depth frames) ###


def overlay_rgbd(filename):


	with open(filename, 'r') as in_file:
		name = filename.split('/')[-1]
		out_file_path = OUT_PATH + name

		with open(out_file_path, 'a') as out_file:

			action_code = name.split('.')[0]
			cam_code= action_code[4:8]
			A = rgb_to_depth_affine_transforms[cam_code]

			num_frames = int(in_file.readline())

			for f in range(num_frames):
				print('FRAME: {}\n'.format(f))
				sg_msg = SkeletonGroup()

				#HEADER
				in_file.readline()
				sg_msg.header.seq = int(in_file.readline().split(':')[-1])
				sg_msg.header.stamp.secs = int(in_file.readline().split(':')[-1])
				sg_msg.header.stamp.nsecs = int(in_file.readline().split(':')[-1])

				#SKELETONS
				skeleton_msgs = []
				num_skeletons = int(in_file.readline())
				for s in range(num_skeletons):
					skeleton_msg = Skeleton()
					skeleton_msg.id = int(in_file.readline().split(':')[-1])
					skeleton_msg.max_markers = int(in_file.readline().split(':')[-1])
					skeleton_msg.max_links = int(in_file.readline().split(':')[-1])
					skeleton_msg.confidence = float(in_file.readline().split(':')[-1])

					num_markers = int(in_file.readline().split(':')[-1])
					markers = []
					markers_img = np.zeros((424,512))

					for m in range(num_markers):
						marker = Marker()
						marker.id = int(in_file.readline().split(':')[-1])
						print('MARKER ID: {}\n'.format(marker.id))
						marker.confidence = float(in_file.readline().split(':')[-1])
						x = float(in_file.readline().split(':')[-1])
						y = float(in_file.readline().split(':')[-1])
						point_aff = np.matmul(A, np.array([[x],[y],[1]]))
						markers_img[int(point_aff[1]), int(point_aff[0])] = 255
						marker.center.pose.position.x = point_aff[0]
						marker.center.pose.position.y = point_aff[1]
						markers.append(marker)

					cv2.imshow('markers', markers_img)
					cv2.waitKey(0)

					skeleton_msg.markers = markers

					num_links = int(in_file.readline().split(':')[-1])
					links = []
					for l in range(num_links):
						link = Link()
						link.id = int(in_file.readline().split(':')[-1])
						link.parent_marker = int(in_file.readline().split(':')[-1])
						link.child_marker = int(in_file.readline().split(':')[-1])
						link.confidence = float(in_file.readline().split(':')[-1])
						x = float(in_file.readline().split(':')[-1])
						y = float(in_file.readline().split(':')[-1])
						point_aff = np.matmul(A, np.array([[x],[y],[1]]))
						link.center.pose.position.x = point_aff[0]
						link.center.pose.position.y = point_aff[1]
						links.append(link)
					skeleton_msg.links = links

				sg_msg.skeletons = skeleton_msgs



def main(args):

	for filename in glob.glob(SKEL_PATH+'*.skeleton'):

		trasf_skel = overlay_rgbd(filename)

if __name__=='__main__':

	main(sys.argv)
