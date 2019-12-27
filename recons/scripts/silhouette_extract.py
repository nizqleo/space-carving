import cv2
import numpy as np

data_dir = "/home/nizq/Downloads/Datas/bunny_data/silhouettes/"

def get_mask(file):
    
    silhouette_file = data_dir + file.split('.')[0] + ".pgm"
    img = cv2.imread(silhouette_file)
    return img

def threshold(inten):

	
	inten = np.array(inten, dtype = np.uint8)

	reval_T, dst = cv2.threshold(inten, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_TRIANGLE)

	return dst