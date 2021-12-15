#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy
import sys, os
import cv2
import glob
import numpy as np
import PIL
import openpifpaf
import matplotlib.pyplot as plt
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Image, CameraInfo
from play_video.msg import Action
from hiros_skeleton_msgs.msg import SkeletonGroup, Skeleton, Marker, Link, KinematicState
from cv_bridge import CvBridge, CvBridgeError

#convert from rgb to depth frame
rgb_to_depth_affine_transforms = dict(
    C001=np.array([[3.45638311e-01, 2.79844266e-03, -8.22281898e+01],
                   [-1.37185375e-03, 3.46949734e-01, 1.30882644e+01]]),

    C002=np.array([[3.42938209e-01, 8.72629655e-04, -7.28786114e+01],
                   [3.43287830e-04, 3.43578203e-01, 1.75767495e+01]]),

    C003=np.array([[3.45121348e-01, 8.53232038e-04, -7.33328852e+01],
                   [1.51167845e-03, 3.45115132e-01, 2.22178592e+01]]),
)

'''
body_graph = [(0, 1), (2, 1), (3, 4), (4, 5), (5, 1), (6, 5),
             (7, 8), (8, 1), (9, 8), (10, 9), (11, 10), (12, 8),
             (13, 12), (14, 15), (15, 0), (16, 0), (17, 15), (18, 16),
             (19, 14), (20, 19), (21, 14), (22, 11), (23, 22), (24, 11)]
'''

body_foot_skeleton = [
    (16, 14), (14, 12), (17, 15), (15, 13), (12, 13), (6, 12), (7, 13),
    (6, 7), (6, 8), (7, 9), (8, 10), (9, 11), (2, 3), (1, 2), (1, 3),
    (2, 4), (3, 5), (4, 6), (5, 7),
    (16, 20), (16, 19), (16, 18),    # left foot
    (17, 23), (17, 21), (17, 22)     # right foot
]

face_skeleton = [
    (25, 5), (39, 4),  # ear to ear body
    (54, 1),  # nose to nose body
    (60, 3), (3, 63), (66, 2), (2, 69), ] + [   # eyes to eyes body
    (x, x + 1) for x in range(24, 40)] + [   # face outline
    (24, 41), (41, 42), (42, 43), (43, 44), (44, 45), (45, 51),  # right eyebrow
    (40, 50), (50, 49), (49, 48), (48, 47), (47, 46), (46, 51),  # left eyebrow
    (24, 60), (60, 61), (61, 62), (62, 63), (63, 51), (63, 64), (64, 65), (65, 60),  # right eye
    (40, 69), (69, 68), (68, 67), (67, 66), (66, 51), (66, 71), (71, 70),   # left eye
    (70, 69), ] + [(x, x + 1) for x in range(51, 59)] + [  # nose
    (59, 54), (57, 75), (78, 36), (72, 28), (72, 83)] + [(x, x + 1) for x in range(72, 83)] + [
    (72, 84), (84, 85), (85, 86), (86, 87), (87, 88), (88, 78),  # upper lip
    (72, 91), (91, 90), (90, 89), (89, 78)]  # lower lip

lefthand_skeleton = ([
    (92, 10),  # connect to wrist
    (92, 93), (92, 97), (92, 101), (92, 105),  # connect to finger starts
    (92, 109)] + [(x, x + 1) for s in [93, 97, 101, 105, 109] for x in range(s, s + 3)]  # four f.
    + [(94, 97), (97, 101), (101, 105), (105, 109)])

righthand_skeleton = ([
    (113, 11),  # connect to wrist
    (113, 114), (113, 118), (113, 122), (113, 126),   # connect to finger starts
    (113, 130)] + [(x, x + 1) for s in [114, 118, 122, 126, 130] for x in range(s, s + 3)]
    + [(115, 118), (118, 122), (122, 126), (126, 130)])

WHOLEBODY_SKELETON = body_foot_skeleton + face_skeleton + lefthand_skeleton + righthand_skeleton

VIDEO_PATH = '/home/lazzaretto/catkin_ws/src/play_video/data/ntu_dataset/60/rgb/nturgb+d_rgb/'
DEPTH_PATH = '/home/lazzaretto/catkin_ws/src/play_video/data/ntu_dataset/60/depth/nturgb+d_depth_masked/'
TEMP_PATH = '/home/lazzaretto/catkin_ws/src/play_video/data/temp/'
#VIDEO_PATH = rospkg.RosPack().get_path('play_video')+ '/data/nturgb+d_rgb_videos/'
#DEPTH_PATH = rospkg.RosPack().get_path('play_video') + '/data/nturgb+d_depth_masked_s1s2/'


class opifpaf_publisher:

	def __init__(self, action_code):

		# Params
		self.action_code = rospy.get_param('action_code', action_code)
		#self.package_dir = rospkg.RosPack().get_path('play_video')
		self.input_video = rospy.get_param('input_dir', VIDEO_PATH + self.action_code + '_rgb.avi')
		self.input_depth_dir = rospy.get_param('input_depth_dir', DEPTH_PATH + self.action_code + '/')

		rospy.init_node('openpifpaf_publisher', anonymous=True)
		self.bridge = CvBridge()
		self.skeleton_pub = rospy.Publisher('skeleton_group', SkeletonGroup, queue_size = 15)
		self.depth_pub = rospy.Publisher('image_depth_rect', Image, queue_size=15)
		self.opifpaf_pub = rospy.Publisher('image_openpifpaf', Image, queue_size=15)
		self.cameraInfo_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=15)
		self.action_pub = rospy.Publisher('action_code', Action, queue_size = 15)
		self.count = 1

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

		cap = cv2.VideoCapture(self.input_video)
		annotation_painter = openpifpaf.show.AnnotationPainter()
		predictor = openpifpaf.Predictor(checkpoint='shufflenetv2k30-wholebody', visualize_image=True)

		cam_code= self.action_code[4:8]
		A = rgb_to_depth_affine_transforms[cam_code]

		rate = rospy.Rate(2)
		
		for frame in sorted(glob.glob(self.input_depth_dir+'*.png')):
			if not rospy.is_shutdown():

				now = rospy.get_rostime()

				_, frame_rgb = cap.read()
				frame_depth = cv2.imread(frame, cv2.IMREAD_GRAYSCALE)
				frame_rgb_pil = PIL.Image.fromarray(cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2RGB))

				skeletons, _, _ = predictor.pil_image(frame_rgb_pil)
				
				image = openpifpaf.visualizer.Base.image()
				out_image = TEMP_PATH +'temp.png'
				with openpifpaf.show.image_canvas(image, out_image) as ax:
					annotation_painter.annotations(ax, skeletons)
	
				img = cv2.imread(out_image)
				ros_msg_opifpaf = Image()
				ros_msg_opifpaf.header.stamp.secs = now.secs
				ros_msg_opifpaf.header.stamp.nsecs = now.nsecs
				ros_msg_opifpaf.encoding = "bgr8"				
				ros_msg_opifpaf.height = 2160
				ros_msg_opifpaf.width = 3840
				if img.dtype.byteorder == '>':
					ros_msg_opifpaf.is_bigendian = True
				ros_msg_opifpaf.data = img.tobytes()
				ros_msg_opifpaf.step = len(ros_msg_opifpaf.data)//ros_msg_opifpaf.height
			
				
				skeletons_msg = SkeletonGroup()
				skeletons_msg.header.seq = self.count
				skeletons_msg.header.stamp.secs = now.secs
				skeletons_msg.header.stamp.nsecs = now.nsecs

				skeleton_msgs = []
				rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

				for s,skeleton in enumerate(skeletons):
					skeleton_msg = Skeleton()
					skeleton_msg.id = s
					skeleton_msg.src_time.secs = now.secs
					skeleton_msg.src_time.nsecs = now.nsecs
					skeleton_msg.max_markers = 133
					skeleton_msg.max_links = 25
					skeleton_msg.confidence = skeleton.score
					### do bounding box from skeleton.bbox()--> 4-array with bbox coords ###
					markers = []
					for j,joint in enumerate(skeleton.data):
						marker = Marker()
						marker.id = j+1
						if joint[2]>0.0:
							marker.confidence = joint[2]
							point = np.ones((3,1))
							point[0,0] = joint[0]
							point[1,0] = joint[1]
							point_aff = np.matmul(A, point)
							marker.center.pose.position.x = point_aff[0]
							marker.center.pose.position.y = point_aff[1]
							#marker.center.pose.position.x = joint[0]
							#marker.center.pose.position.y = joint[1]
							markers.append(marker)
					skeleton_msg.markers = markers
					
					links = []
					for l,joints in enumerate(body_foot_skeleton):
						link = Link()
						link.id = l+1
						link.parent_marker = joints[0]
						link.child_marker = joints[1]
						parent_confidence = markers[joints[0]-1].confidence
						child_confidence = markers[joints[1]-1].confidence
						parent_pos = markers[joints[0]-1].center.pose.position
						child_pos = markers[joints[1]-1].center.pose.position
						if parent_confidence>0.0 and child_confidence>0.0:
							link.confidence = min(parent_confidence,child_confidence)
							point = np.ones((3,1))
							x = (parent_pos.x+child_pos.x)*0.5
							y = (parent_pos.y+child_pos.y)*0.5
							point[0,0] = x
							point[1,0] = y
							point_aff = np.matmul(A, point)
							link.center.pose.position.x = point_aff[0]
							link.center.pose.position.y = point_aff[1]
							links.append(link)
					skeleton_msg.links = links



					skeleton_msgs.append(skeleton_msg)
				skeletons_msg.skeletons = skeleton_msgs

				#ros_msg_depth = self.bridge.cv2_to_imgmsg(frame_depth, 'mono8')

				ros_msg_depth = Image()
				ros_msg_depth.header.stamp.secs = now.secs
				ros_msg_depth.header.stamp.nsecs = now.nsecs
				ros_msg_depth.encoding = "mono8"				
				ros_msg_depth.height = 424
				ros_msg_depth.width = 512
				if frame_depth.dtype.byteorder == '>':
					ros_msg_depth.is_bigendian = True
				ros_msg_depth.data = frame_depth.tobytes()
				ros_msg_depth.step = len(ros_msg_depth.data)//ros_msg_depth.height
				
				action_msg = Action()
				action_msg.action = self.action_code
				action_msg.header.stamp.secs = now.secs
				action_msg.header.stamp.nsecs = now.nsecs

				self.cam_info_msg.header.stamp.secs = now.secs
				self.cam_info_msg.header.stamp.nsecs = now.nsecs


				self.action_pub.publish(action_msg)
				print('*************ACTION*************')
				self.depth_pub.publish(ros_msg_depth)
				print('*************DEPTH*************')
				self.cameraInfo_pub.publish(self.cam_info_msg)
				print('*************CAMINFO*************')
				self.skeleton_pub.publish(skeletons_msg)
				print('*************SKELETON*************')
				self.opifpaf_pub.publish(ros_msg_opifpaf)				
				



				rate.sleep()
			else:
				return



def main():
	
	for filename in sorted(glob.glob(VIDEO_PATH+'S001C003P005R001A01*.avi')):
		action = filename.split('/')[-1]
		action_code = action.split('_')[0]
		opifpaf_pub = opifpaf_publisher(action_code)
		opifpaf_pub.run()
	
	rospy.spin()

if __name__=='__main__':

	main()
