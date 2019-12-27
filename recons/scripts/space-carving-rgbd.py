import os
import cv2
import math
import numpy as np 
from sklearn.preprocessing import normalize

import silhouette_extract
           
graity = 0.005
xmin = -0.15
xmax = 0.15
ymin = -0.2
ymax = 0.1
zmin = -0.15
zmax = 0.15

height = 480
width = 640
hfov = 1.047

obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = bool)
obj_r = obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)
obj_g = obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)
obj_b = obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)

out_cnt = 0
total_carve_cnt = 0

def cameraMatrix(position):
    f = width/(2*math.tan(hfov/2))
    up = np.array([0,0,1])
    calibMatrix = np.array([[f, 0, width/2, 0], 
        [0, f, height/2,0],
        [0,0,1, 0]])
    forward = position/np.linalg.norm(position)
    up = up-up.T.dot(forward)*forward
    right = np.cross(up, forward)
    R = np.array([[right[0], right[1], right[2]], 
        [up[0], up[1], up[2]],
        [forward[0], forward[1], forward[2]]])
    RP = position.reshape(3,1)
    RP = R.dot(RP)
    World2Cam = np.array([[right[0], right[1], right[2], -RP[0, 0]], 
        [up[0], up[1], up[2],-RP[1, 0]],
        [forward[0], forward[1], forward[2],-RP[2, 0]],
        [0,0,0,1]] )
    print(World2Cam)

    return calibMatrix.dot(World2Cam)


if __name__ == "__main__":

    for file in ["4"]:
        carve_cnt = 0
        print("processing "+ file)
        img = cv2.imread("/home/nizq/catkin_ws/src/rgb/"+file+".png")
        inten = np.linalg.norm(img, axis = 2)
        depth = cv2.imread("/home/nizq/catkin_ws/src/depth/"+file+".png",cv2.IMREAD_GRAYSCALE)
        img_mask = silhouette_extract.threshold(inten)
        for i in range(200, 350):
            for j in range(200, 400):
                img_mask[i,j] = 0
        cv2.imshow("mask",img_mask)
        cv2.waitKey(0)
        if file == '4':
            camera_position = np.array([0.866, 0, 0.5])
        else:
            camera_position = np.array([0.433, -0.75, 0.5])
        P = cameraMatrix(camera_position)

        for i in range(obj.shape[0]):
            for j in range(obj.shape[1]):
                for k in range(obj.shape[2]):
                    if obj[i,j,k] == 0:
                        continue

                    voxel_cord = np.array([[xmin + i*graity], [ymin + j*graity],[zmin + k*graity], [1]])
                    dist = np.linalg.norm(voxel_cord-np.array([[camera_position[0]], [camera_position[1]],[camera_position[2]], [1]]))
                    pix_cord = P.dot(voxel_cord)
                    pix_cord = pix_cord / pix_cord[2]
                    pixel_i = int(pix_cord[1])
                    pixel_j = width-int(pix_cord[0])

                    if i == 12 and j ==10 and k == 10:
                        print(voxel_cord, pix_cord, dist, depth[pixel_i, pixel_j], img_mask[pixel_i, pixel_j])


                    # out of the image
                    if pixel_i >= height or pixel_i < 0 or pixel_j >= width or pixel_j < 0:
                        out_cnt = out_cnt+1
                        continue

                    if img_mask[pixel_i, pixel_j] == 0 and np.linalg.norm(dist - depth[pixel_i, pixel_j]) < graity:#seen
                        obj[i, j, k] = 1
                        obj_r[i,j,k] = img[pixel_i, pixel_j, 2]
                        obj_g[i,j,k] = img[pixel_i, pixel_j, 1]
                        obj_b[i,j,k] = img[pixel_i, pixel_j, 0]
                    # Pixels with intensity values 0 (black) denote object occupancy
                    elif dist < depth[pixel_i, pixel_j] or img_mask[pixel_i, pixel_j] > 0:
                        obj[i,j,k] = 0
                        carve_cnt= carve_cnt+1
                    else:# unseen
                        obj[i, j, k] = 1
                        obj_r[i,j,k] = 100
                        obj_g[i,j,k] = 100
                        obj_b[i,j,k] = 100
        print(carve_cnt, "voxels carved this round")
        total_carve_cnt = total_carve_cnt + carve_cnt

    print(obj.size, obj.size-total_carve_cnt, out_cnt)
    color_file = open("./result.txt", "w")
    head_line = "{} {} {} {} {} {} {} {}\n" # graity, xmin, xmax, ymin, ymax, zmin, zmax, is_colored version
    voxel_elem_color = "{} {} {} {}\n"
    color_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax, 1))
            
    for i in range(obj.shape[0]):
        for j in range(obj.shape[1]):
            for k in range(obj.shape[2]):
                color_file.write(voxel_elem_color.format(int(obj[i,j,k]),int(obj_r[i,j,k]), int(obj_g[i,j,k]), int(obj_b[i,j,k])))
        
    color_file.close()
    pass