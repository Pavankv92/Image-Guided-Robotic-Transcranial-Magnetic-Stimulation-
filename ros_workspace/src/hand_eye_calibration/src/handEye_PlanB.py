#!/usr/bin/env python

from __future__ import print_function
import numpy as np
from numpy.linalg import inv
from numpy.linalg import multi_dot

X = np.array([[-1,0,0,0.12],
             [0,1,0,0.13],
             [0,0,1,0.17],
             [0,0,0,1]])

def handEye (M,N,number):
    for i in range(number):
        Y = multi_dot([M[i],X,inv(N[i])])
        print (Y)