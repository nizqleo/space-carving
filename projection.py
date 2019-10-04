import cv2
import numpy as np 

data_dir = "/home/nizq/Downloads/Datas/bunny_data/calib/"

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

        

    

