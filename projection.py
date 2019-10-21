import cv2
import numpy as np 
import scipy as sp
data_dir = "/home/nizq/Downloads/Datas/beethoven_data/calib/"

def projectionMatrix(file):
    projection_file_name = data_dir + file.split('.')[0] + ".txt"
    proj_file = open(projection_file_name, "r+")
    matrix_line = proj_file.readline()
    P = np.empty((3,4), dtype = float)
    for i in range(3):
        matrix_line = proj_file.readline()
        for j in range(4):
            P[i, j] = float(matrix_line.split()[j])
    return P


def camera_position(file):
    P = projectionMatrix(file)

    Ps = np.split(P, [3,4], axis = 1)
    P = Ps[0]
    b = -Ps[1]

    X = np.linalg.solve(P,b)

    return X
    