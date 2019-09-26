import os
import cv2
import math
import numpy as np 

import silhouette_extract
import projection

data_dir = "/home/nizq/Downloads/Datas/bunny_data/images"
           
graity = 0.25
xmin = -7.5
xmax = 7.5#12.5
ymin = -10#-28
ymax = 10#-3
zmin = -7.5#-20
zmax = 15#5

# x: [-7.5, 7.5]  15
# y: [-10, 10] 20
# z: [-7.5, 15] 22.5
height = 768
width = 1024

obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = bool)
carve_cnt = 0
out_cnt = 0
if __name__ == "__main__":

    for file in os.listdir(data_dir):
        print("processing "+ file)
        img = cv2.imread(file)
        img_mask = silhouette_extract.get_mask(file)
        P = projection.projectionMatrix(file)
        
        for i in range(obj.shape[0]):
            for j in range(obj.shape[1]):
                for k in range(obj.shape[2]):
                    if obj[i,j,k] == 0:
                        continue
                    voxel_cord = np.array([[xmin + i*graity], [ymin + j*graity],[zmin + k*graity], [1]])
                    pix_cord = P.dot(voxel_cord)

                    pix_cord = pix_cord / pix_cord[2,0]
                    if pix_cord[1,0] >= height or pix_cord[0,0] < 0 or pix_cord[0,0] >= width or pix_cord[1,0] < 0:
                        out_cnt = out_cnt+1
                        continue


                    # Pixels with intensity values 0 (black) denote object occupancy
                    if img_mask[int(pix_cord[1,0]), width-int(pix_cord[0,0]), 0] > 0 or img_mask[int(pix_cord[1,0]), width-int(pix_cord[0,0]), 1] > 0 or img_mask[int(pix_cord[1,0]), width-int(pix_cord[0,0]), 2] > 0:
                        obj[i,j,k] = 0
                        carve_cnt= carve_cnt+1


    print(obj.size, obj.size-carve_cnt, out_cnt)
    final_file = open("./result.txt", "w")
    head_line = "{} {} {} {} {} {} {}\n"
    mid_element = "{} "
    final_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax))
            
    for i in range(obj.shape[0]):
        for j in range(obj.shape[1]):
            for k in range(obj.shape[2]):
                final_file.write(mid_element.format(int(obj[i,j,k])))
    pass