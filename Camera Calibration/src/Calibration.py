import sys
import cv2
import numpy as np
import scipy.stats as st
import os
import math
import glob
import random


def parameter_calculation(points_3D, points_2D):

    A = np.zeros((len(points_3D)*2, 12))

    j = 0
    for i in range(0, len(points_3D)):
        x = np.array([[points_3D[i][0], points_3D[i][1], points_3D[i][2], points_3D[i][3], 0, 0, 0, 0, -points_2D[i][0]*points_3D[i][0], -points_2D[i][0]*points_3D[i][1], -points_2D[i][0]*points_3D[i][2], -points_2D[i][0]*points_3D[i][3]]])
        y = np.array([[0, 0, 0, 0, points_3D[i][0], points_3D[i][1], points_3D[i][2], points_3D[i][3], -points_2D[i][1]*points_3D[i][0], -points_2D[i][1]*points_3D[i][1], -points_2D[i][1]*points_3D[i][2], -points_2D[i][1]*points_3D[i][3]]])
        A[j] = x
        A[j+1] = y
        j+= 2

    ATA = np.dot(A.T, A)
    U, S, V = np.linalg.svd(ATA, full_matrices=True)
    solution = V[:,11]

    array1 = np.array([solution[0], solution[1], solution[2]])
    array2 = np.array([solution[4], solution[5], solution[6]])
    array3 = np.array([solution[8], solution[9], solution[10]])
    b = np.matrix([[solution[3]], [solution[7]], [solution[11]]])

    ro = 1/np.linalg.norm(array3)
    u0 = ro**2*(np.dot(array1, array3))
    v0 = ro**2*(np.dot(array2, array3))
    alfa_v = math.sqrt(ro**2*np.dot(array2,array2)-v0**2)
    s = (ro**4/alfa_v)*(np.dot(np.cross(array1, array3), np.cross(array2, array3)))
    alfa_u = math.sqrt(ro**2*np.dot(array1, array1)-s**2-u0**2)
	
	
	#prints u0 and v0
    print("(u0,v0): ",u0,v0)
    print("\n \n")
	
	#prints alfa_u and alfa_v
    print("(alfa_u,alfa_v): ",alfa_u,alfa_v)
    print("\n \n")
	
    print("s: ",s)
    print("\n \n")
	
	#calulate intrinsic parameter
    intrinsic_parameter = np.matrix([[alfa_u, s, u0],[0.0, alfa_v, v0],[0.0, 0.0, 1]])

    K_inverse = np.linalg.inv(intrinsic_parameter)
    r1 = np.cross(array2,array3)/np.linalg.norm(np.cross(array2,array3))
    r3 = array3
    r2 = np.cross(r3, r1)
    ep = np.sign(b[2][0])
    T = (np.dot(K_inverse,b))
    T *= ep*ro
	
	#calculate extrinsic parameter
    extrinsic_paramameter = np.matrix([[r1[0], r1[1], r1[2], T[0][0]],[r2[0], r2[1], r2[2], T[1][0]],[r3[0], r3[1], r3[2], T[2][0]]])

	#prints extrinsic parameters
    print("extrinsic parameters are:", extrinsic_paramameter)
    print("\n\n")
	
	#prints intrinsic parameters
    print("intrinsic parameters are:", intrinsic_parameter)
    print("\n\n")
	
    
    M = np.dot(intrinsic_parameter, extrinsic_paramameter)

    return M

def calculate_MeanSquareError(points_3D, points_2D, M):

    points_2DH = []
    points_2D = []
    points_2D_ref = []

    MeanSquareErrorx = 0
    MeanSquareErrory = 0

    f = open("3D_objectpoints.txt", "r")
    for ln in f:
        points_DH3 = np.array([[float(ln.split()[0]), float(ln.split()[1]), float(ln.split()[2]), 1.0]])
        a = (M*points_DH3.T)
        points_2DH.append(a)
    f.close()

    g = open("2D_imagepoints.txt", "r")
    for ln in g:
        points_2D_ref.append([float(ln.split()[0]), float(ln.split()[1])])
    g.close()

    np.seterr(divide='ignore', invalid='ignore')
    for pt in points_2DH:
        if(pt[2]!=0):
            pt[0] = pt[0]/pt[2]
            pt[1] = pt[1]/pt[2]
            pt[2] = pt[2]/pt[2]
            pt = np.delete(pt, (2), axis = 0)
            points_2D.append(pt)

    for i in range(0, len(points_2D)):
        
        MeanSquareErrorx += (points_2D_ref[i][0]-points_2D[i][0])**2
        MeanSquareErrory += (points_2D_ref[i][1]-points_2D[i][1])**2
    
    MeanSquareErrorx = MeanSquareErrorx/len(points_2D_ref)
    MeanSquareErrory = MeanSquareErrory/len(points_2D_ref)
    MeanSquareError = MeanSquareErrorx+MeanSquareErrory

    return MeanSquareError





def main():
    objpt = open("3D_objectpoints.txt", "r")	
    imgpt = open("2D_imagepoints.txt", "r")
    points_3D = []
    points_2D = []

    for ln in objpt:
        points_3D.append([float(ln.split()[0]), float(ln.split()[1]), float(ln.split()[2]), 1.0])
    objpt.close()

    for ln in imgpt:
        points_2D.append([float(ln.split()[0]), float(ln.split()[1])])
    imgpt.close()
	
    M = parameter_calculation(points_3D, points_2D)
    MeanSquareError = calculate_MeanSquareError(points_3D, points_2D, M)
    

    print("M:", M)
    print("\n")
	
	#print Mean Square Error
    print("Mean Square Error:", MeanSquareError)
    
if __name__ == '__main__':
    main()
