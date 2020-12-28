#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import cv2
import glob



criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-16)
np.set_printoptions(precision=16)

def createObjectpoints(height,width,size):
    objp = np.zeros((height*width, 3), np.float32)
    objp[:,:2] = np.indices((width, height)).T.reshape(-1,2)
    objp *= size
    return np.around(objp, 3)

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img


def transform_from_image(gray_image, show=False):
    objp = createObjectpoints(5,8,0.04)
   
    cam_mtx = np.array([[1.0535387124286060e+03, 0, 9.5122786283926860e+02],
                    [0, 1.0538348361017167e+03, 5.3702175248086860e+02],
                    [0, 0, 1]], np.float32)

    cam_dist = np.array([0.0055971983516318, 0.116410774791633, -0.0002495590397735, -0.0005281194057462, -0.192996279885084], np.float32)
    
    found, corners = cv2.findChessboardCorners(gray_image, (8,5))

    if found:
        cv2.cornerSubPix(gray_image, corners, (11,11), (-1, -1), criteria)

        #debug
        #img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
        #cv2.drawChessboardCorners(img, pattern, corners, found)
        #cv2.imshow('image', img)
        #cv2.waitKey(250)


        #solvePnP is used to get the position and orientation of the object
        found, rvecs, tvec = cv2.solvePnP(objp, corners, cam_mtx, cam_dist)
        rot3X3 = cv2.Rodrigues(rvecs)[0]
        
        if show:
            axis = np.float32([[0.08,0,0], [0,0.08,0], [0,0,-0.08]]).reshape(-1,3)
            imgpts, __ = cv2.projectPoints(axis, rvecs, tvec, cam_mtx, cam_dist)
            gray_image = draw(gray_image,corners,imgpts)
        
        #print(rotationMatrix_3X3)
        transformation = np.array([[rot3X3[0,0], rot3X3[0,1], rot3X3[0,2], tvec[0]],
                                    [rot3X3[1,0], rot3X3[1,1], rot3X3[1,2], tvec[1]],
                                    [rot3X3[2,0], rot3X3[2,1], rot3X3[2,2], tvec[2]],
                                    [0, 0, 0, 1]], np.float32)

    else:
        print("pattern not found")
        raise ValueError("Could not find the pattern in the image")

    return transformation
