import cv2

data_dir = "/home/nizq/Downloads/Datas/beethoven_data/silhouettes/"

def get_mask(file):
    
    silhouette_file = data_dir + file.split('.')[0] + ".pgm"
    img = cv2.imread(silhouette_file)
    return img

