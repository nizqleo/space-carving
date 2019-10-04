import os
import cv2
import math
import numpy as np 

import silhouette_extract
import projection
import radiance


data_dir = "/home/nizq/Downloads/Datas/beethoven_data/images1"
           
graity = 0.25
xmin = -10
xmax = 5
ymin = -10
ymax = 8
zmin = -5
zmax = 17.5

# x: [-7.5, 7.5]  15
# y: [-10, 10] 20
# z: [-7.5, 15] 22.5
height = 768
width = 1024

obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = bool)
out_cnt = 0

camera_position = {}
camera_P = {}
image_pixel_mark_situ = {}

if __name__ == "__main__":

    for file in os.listdir(data_dir):
        print("processing "+ file)
        img_mask = silhouette_extract.get_mask(file)
        P = projection.projectionMatrix(file)
        
        # register camera position
        camera_position[file] = projection.camera_position(file) 
        camera_P[file] = P
        pixel_map = np.zeros((height, width), dtype = bool)
        image_pixel_mark_situ[file] = pixel_map

        carve_cnt = 0
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

                    pixel_map[int(pix_cord[1,0]), int(pix_cord[0,0])] = 0

                    # Pixels with intensity values 0 (black) denote object occupancy
                    if img_mask[int(pix_cord[1,0]), int(pix_cord[0,0]), 0] > 0 or img_mask[int(pix_cord[1,0]), int(pix_cord[0,0]), 1] > 0 or img_mask[int(pix_cord[1,0]), int(pix_cord[0,0]), 2] > 0:
                        obj[i,j,k] = 0
                        carve_cnt= carve_cnt+1

        print(carve_cnt, "voxels carved this round")
        

    print(obj.size, obj.size-carve_cnt, out_cnt)
    final_file = open("./visual_hull_result.txt", "w")
    head_line = "{} {} {} {} {} {} {} {}\n" # graity, xmin, xmax, ymin, ymax, zmin, zmax, is_colored version
    voxel_elem = "{} "
    voxel_elem_color = "{} {} {} {}"
    final_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax, 0))
            
    for i in range(obj.shape[0]):
        for j in range(obj.shape[1]):
            for k in range(obj.shape[2]):
                final_file.write(voxel_elem.format(int(obj[i,j,k])))
    
    # end of the visual hull build
    
    # begin to sweep
    carve_cnt = 0
    for i in range(obj.shape[0]):

        # form the camera set
        threshold = xmin + i*graity
        camera_set = [] 
        for key,values in  camera_position.items():
            if(values[0] < threshold):
                camera_set.append(key)

        for j in range(obj.shape[1]):
            for k in range(obj.shape[2]):
                if obj[i,j,k] == 0:
                    continue
                
                color_set = np.empty([len(camera_set), 3])
                c = 0
                for camera in camera_set:
                    img = cv.imread(data_dir + camera)
                    voxel_cord = np.array([[xmin + i*graity], [ymin + j*graity],[zmin + k*graity], [1]])
                    pix_cord = P.dot(voxel_cord)
                    pix_cord = pix_cord / pix_cord[2,0]
                    color_set[c] = img[int(pix_cord[1,0]), int(pix_cord[0,0])]

                consist, color = radiance.consistency(color_set):
                
                if not consist:
                    obj[i,j,k] = 0
                    carve_cnt = carve_cnt + 1
                else:
                    obj[i,j,k] = 

            
    
    
    
    
    
    
    pass