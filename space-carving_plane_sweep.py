import os
import cv2
import math
import numpy as np 

import silhouette_extract
import projection
import radiance

data_dir = "/home/nizq/Downloads/Datas/bunny_data/images"
           
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
obj_r = obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)
obj_g = obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)
obj_b = obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)


out_cnt = 0

camera_position = {}
camera_P = {}
image_pixel_mark_situ = {}
images = {}


def consistency_check(i, j, k, camera_set):
    color_set = np.empty([len(camera_set), 3])
    c = 0
    for camera in camera_set:
        img = images[camera]
        voxel_cord = np.array([[xmin + i*graity], [ymin + j*graity],[zmin + k*graity], [1]])
        pix_cord = camera_P[camera].dot(voxel_cord)
        pix_cord = pix_cord / pix_cord[2,0]
        
        if image_pixel_mark_situ[camera][int(pix_cord[1,0]), int(pix_cord[0,0])] == 0:
            color_set[c] = img[int(pix_cord[1,0]), int(pix_cord[0,0])]
            c = c+1

    if c != len(camera_set):
        color_set = np.split(color_set, [c, len(camera_set)], axis = 0)[0]
    consist, color = radiance.consistency(color_set)
    
    if not consist:
        obj[i,j,k] = 0
        return False
    else:
        # BGR
        obj_r[i,j,k] = color[2]
        obj_g[i,j,k] = color[1]
        obj_b[i,j,k] = color[0]
        image_pixel_mark_situ[file][int(pix_cord[1,0]), int(pix_cord[0,0])] = 1
        return True

last = []
clock_wise = [[0,-1], [1, -1], [1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1]]

def next_generator_i(i, j, k):
    
    global obj
    global clock_wise
    global last
    if last == []:
        if obj[i, j, k] != 0:
            last = [j,k]
            return next_generator_i(i, j, k)
        for tj in range(j,obj.shape[1]):
            for tk in range(obj.shape[2]):
                if obj[i,tj,tk] != 0:
                    return tj,tk
        return -1, -1
    else:
        if obj[i, j, k] == 0:
            temp = last
            last=[j,k]
            j = temp[0]
            k = temp[1]
            
        start = [last[0]-j,last[1]-k]
        if start == [0, 0]:
            start = [-1, 0]
        
        if obj[i, j, k] != 0:
            last = [j, k]
            
        start_idx = clock_wise.index(start)
        for p in range(start_idx+1, 8):
            temp_j=j+clock_wise[p][0]
            temp_k=k+clock_wise[p][1]
            if obj[i,temp_j,temp_k] != 0:
                return temp_j,temp_k
        for p in range(0, start_idx):
            temp_j=j+clock_wise[p][0]
            temp_k=k+clock_wise[p][1]
            if obj[i,temp_j,temp_k] != 0:
                return temp_j,temp_k
        return -1,-1
        

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
        images[file] = cv2.imread(data_dir +'/'+ file)

        # carve out the visual hull
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
    
    final_file.close()
    # end of the visual hull build
    
    # begin plane_sweep
    carve_cnt = 0
    for i in range(obj.shape[0]):
        if i != 28:
            for j in range(obj.shape[1]):
                for k in range(obj.shape[2]):
                    obj[i,j,k] = 0

        last = []
        # form the camera set
        threshold = xmin + i*graity
        camera_set = [] 
        for key,values in  camera_position.items():
            if(values[0] < threshold):
                camera_set.append(key)
        if len(camera_set) < 1:
            continue

        passed = []
        j, k = next_generator_i(i, 0, -1)
        if j == -1 and k == -1:
            continue
        while(([j, k] not in passed) and [j,k] != [-1,-1]):
            if consistency_check(i, j, k, camera_set):
                passed.append([j, k])
            else:
                obj[i,j,k] = 0
                carve_cnt = carve_cnt+1
            j,k = next_generator_i(i, j ,k)



    print("x sweep1  carved:",carve_cnt)

    color_file = open("./photo_hull_result1.txt", "w")
    head_line = "{} {} {} {} {} {} {} {}\n" # graity, xmin, xmax, ymin, ymax, zmin, zmax, is_colored version
    voxel_elem = "{} "
    voxel_elem_color = "{} {} {} {}\n"
    color_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax, 1))
            
    for i in range(obj.shape[0]):
        for j in range(obj.shape[1]):
            for k in range(obj.shape[2]):
                color_file.write(voxel_elem_color.format(int(obj[i,j,k]),int(obj_r[i,j,k]), int(obj_g[i,j,k]), int(obj_b[i,j,k])))
        
    color_file.close()
    pass