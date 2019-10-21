import cv2
import numpy as np
import math

def consistency(col_set):
    
    col_center = np.mean(col_set, axis = 0)
    #print(col_center)

    normed_col_center = col_set - col_center
    #print(normed_col_center)

    distance = np.sum(np.multiply(normed_col_center, normed_col_center), axis=1)
    #print(distance)

    max_distance = np.amax(distance)
    #print(max_distance)

    if math.sqrt(max_distance) > 200:
        return [False, col_center]
    else:
        return [True, col_center]