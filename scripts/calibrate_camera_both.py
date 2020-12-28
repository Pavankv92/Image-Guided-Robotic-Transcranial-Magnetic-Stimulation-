#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2
import glob


pattern = (8,5) 
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-16)
np.set_printoptions(precision=16)


def intrinsic(all_image_names, mode):
    objp = np.zeros((np.prod(pattern), 3), np.float32)
    objp[:,:2] = np.indices(pattern).T.reshape(-1,2)

    objectpoints = []
    imagepoints = []

    for name in all_image_names:
        #print(name)
        gray_image = cv2.imread(name, 0)
        if mode == 'ir':
            gray_image = cv2.resize(gray_image, None, 2.0, 2.0, cv2.INTER_CUBIC)

        found, corners = cv2.findChessboardCorners(gray_image, pattern)

        if found:
            cv2.cornerSubPix(gray_image, corners, (11,11), (-1, -1), criteria)
            objectpoints.append(objp)
            imagepoints.append(corners)

            #debug
            #img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
            #cv2.drawChessboardCorners(img, pattern, corners, found)
            #cv2.imshow('image', img)
            #cv2.waitKey(250)

        else:
            print("not found " + name)

    cv2.destroyAllWindows()
    return (objectpoints, imagepoints, gray_image.shape[::-1])

print('loading files')
(objectpoints_rgb, imagepoints_rgb,shape_rgb)= intrinsic(glob.glob('../data/calibration_images/image_rgb*.png'), 'rgb')
(objectpoints_ir, imagepoints_ir,shape_ir)= intrinsic(glob.glob('../data/calibration_images/image_ir*.png'), 'ir')

print('starting intrinsic calibration')
rms_rgb, camera_matrix_rgb, dist_coeffs_rgb, rot_vecs_rgb, trans_vecs_rgb = cv2.calibrateCamera(objectpoints_rgb, imagepoints_rgb, shape_rgb, None, None)
rms_ir, camera_matrix_ir, dist_coeffs_ir, rot_vecs_ir, trans_vecs_ir = cv2.calibrateCamera(objectpoints_ir, imagepoints_ir, shape_ir, None, None)

print("###############")
print("RMS RGB: ", rms_rgb)
print("camera matrix RGB: ", camera_matrix_rgb)
print("distortion coefficients RGB: ", dist_coeffs_rgb)
print("###############")
print("RMS IR: ", rms_ir)
print("camera matrix IR: ", camera_matrix_ir)
print("distortion coefficients IR: ", dist_coeffs_ir)
print("###############")

print('starting extrinsic calibration')
retval, camera_matrix_ir, dist_coeffs_ir, camera_matrix_rgb, dist_coeffs_rgb, rot, trans, essential, fundamental = cv2.stereoCalibrate(objectpoints_rgb, imagepoints_ir, imagepoints_rgb, camera_matrix_ir, dist_coeffs_ir, camera_matrix_rgb, dist_coeffs_rgb, shape_rgb, (1920, 1080), flags=cv2.CALIB_FIX_INTRINSIC, criteria=criteria)

print('rotation',rot)
print('translation', trans)
print("essential", essential)
print("fundamental", fundamental)