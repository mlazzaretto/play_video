#!/usr/bin/env python

import sys, os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import glob, re

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

A_CODE = 'S001C003P005R001A013'
RGB_PATH = '../rgb_frames/'+A_CODE+'_rgb/'
DEPTH_PATH = '../depth_frames/'+A_CODE+'/'
OUT_PATH = '../depth_frames/aff_depth/'+A_CODE+'/'

def show_depth_mask(img_rgb, img_d):
	
	img_rgb_cropped = img_rgb[:, 0:1304]

	cam = A_CODE[4:8]
	A = depth_to_rgb_affine_transforms[cam]
	depth = cv2.warpAffine(img_d, A, (1304, 1080), flags=cv2.INTER_CUBIC)

	temp = cv2.findNonZero(depth)
	for pix in temp:
		depth[pix[0][1],pix[0][0]]= 255
	mask = np.asarray(depth, dtype="uint8")
	res = cv2.bitwise_and(img_rgb_cropped,img_rgb_cropped,mask = mask)

	cv2.imshow('depth mask', res)
	cv2.waitKey(0)


def overlay_rgbd(img_d):

	cam = A_CODE[4:8]
	A = depth_to_rgb_affine_transforms[cam]
	depth = cv2.warpAffine(img_d, A, (1304, 1080))	
	depth_full = np.zeros((1080, 1920), dtype=np.uint8)
	depth_full[0:1080,0:1304]=depth
	return depth_full

	

def show_depth_mask1(img_rgb, img_d):

	cam = A_CODE[4:8]
	A = depth_to_rgb_affine_transforms[cam]
	(rows, cols) = img_d.shape[:2]
	print(rows, cols)

	depth = cv2.warpAffine(img_d, A, (1920, 1080), flags=cv2.INTER_CUBIC)
	(rows, cols) = depth.shape[:2]

	for i in range(cols):
		print(i, ": ", depth[0,i])
		




	temp = cv2.findNonZero(depth)
	for pix in temp:
		depth[pix[0][1],pix[0][0]]= 255
	mask = np.asarray(depth, dtype="uint8")
	res = cv2.bitwise_and(img_rgb,img_rgb,mask = mask)

	cv2.imshow('affine depth',mask)
	cv2.waitKey(0)



def main(args):

	for filename in glob.glob(DEPTH_PATH+'*.png'):

		frame = filename.split('/')[3][12:15]
		img_rgb = cv2.imread(RGB_PATH + A_CODE + '_rgb_'+ frame + '.png')
		img_depth = cv2.imread(DEPTH_PATH + 'MDepth-00000' + frame + '.png',cv2.IMREAD_GRAYSCALE)
		#show_depth_mask1(img_rgb, img_depth)
		#depth = overlay_rgbd(img_depth)
		#cv2.imwrite(OUT_PATH+ 'MDepth-00000' + frame + '.png',depth)

		#cv2.imshow('transformed depth image', depth)
		#cv2.waitKey(0)

	img_depth = np.ones((424,512), dtype=np.uint8)*255
	show_depth_mask1(img_rgb, img_depth)


if __name__=='__main__':

	main(sys.argv)
