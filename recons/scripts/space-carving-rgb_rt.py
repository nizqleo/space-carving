#!/usr/bin/env python

import sys
import copy
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from moveit_commander.conversions import pose_to_list
from tf import transformations
from cv_bridge import CvBridge, CvBridgeError
import cv2
import random

import silhouette_extract
import radiance

graity = 0.005
xmin = -0.1
xmax = 0.2
ymin = -0.15
ymax = 0.15
zmin = -0.1
zmax = 0.02

height = 480
width = 640
hfov = 1.047

obj = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = bool)
obj_r = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)
obj_g = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)
obj_b = np.ones((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)

obj_num_of_cams = np.zeros((int((xmax-xmin)/graity), int((ymax-ymin)/graity), int((zmax-zmin)/graity)), dtype = int)

camera_P = {}
image_pixel_mark_situ = {}
images = {}
camera_x_sort = []
camera_y_sort = []
camera_z_sort = []
color_record = {}

out_cnt = 0
total_carve_cnt = 0
bridge = CvBridge()
rgb_flag = False
rgb_ready = True
sampleNum = 10
stamp = 0
rgbImg = []
camera_position = np.array([0.433, 0, 0.25])#)#[-0.0347, 0.246, 0.437])#)
pub = rospy.Publisher("/cameraPosition",Vector3, queue_size = 10 )
numofviews = 6

def takeSecond(elem):
    return elem[1]

def blockRemove():
    global image_pixel_mark_situ
    for camera, x in camera_x_sort:
        for i in range(height):
            for j in range(width):
                image_pixel_mark_situ[camera][i,j] = 0
    
def colorUpdate(level):
    global image_pixel_mark_situ
    global obj_r
    global obj_g
    global obj_b
    global obj
    global color_record

    for i in range(obj.shape[0]):
        for j in range(obj.shape[1]):
            for k in range(obj.shape[2]):
                c = obj_num_of_cams[i,j,k]

                if c > 0:
                    if level == 0 or not color_record.has_key((i,j,k)):
                        color_record[(i,j,k)] = np.array([obj_r[i,j,k] / c, obj_g[i,j,k] / c, obj_b[i,j,k] / c])
                    else:
                        color_record[(i,j,k)] = color_record[(i,j,k)]*c*level+np.array([obj_r[i,j,k], obj_g[i,j,k], obj_b[i,j,k]])
                        color_record[(i,j,k)]/=((level+1)*c)
                if level < 2:
                    obj_num_of_cams[i,j,k] = 0
                obj_r[i,j,k] = 0
                obj_g[i,j,k] = 0
                obj_b[i,j,k] = 0   

def cameraMatrix(position):
    f = width/(2*math.tan(hfov/2))
    up = np.array([0,0,1])
    calibMatrix = np.array([[f, 0, width/2, 0], 
        [0, f, height/2,0],
        [0,0,1, 0]])
    forward = position/np.linalg.norm(position)
    up = up-up.T.dot(forward)*forward
    up = up/np.linalg.norm(up)
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

    return calibMatrix.dot(World2Cam)

def consistency_check(i, j, k, camera_set):
    global image_pixel_mark_situ
    global obj_r
    global obj_g
    global obj_b
    global obj_num_of_cams

    color_set = np.empty([len(camera_set), 3])
    c = 0
    voxel_cord = np.array([[xmin + i*graity], [ymin + j*graity],[zmin + k*graity], [1]])

    for camera in camera_set:
        pix_cord = camera_P[camera].dot(voxel_cord)
        pix_cord = pix_cord / pix_cord[2,0]
        pixel_i = int(pix_cord[1])
        pixel_j = width-int(pix_cord[0])
        
        if pixel_i >= height or pixel_i < 0 or pixel_j >= width or pixel_j < 0:
            continue

        if image_pixel_mark_situ[camera][pixel_i, pixel_j] == 0:
            color_set[c] = images[camera][pixel_i, pixel_j]
            c += 1
    if c == 0:
        return True
    
    # c ==1
    consist = True
    color = color_set[0]
    if c > 1:
        if c != len(camera_set):
            color_set = np.split(color_set, [c, len(camera_set)], axis = 0)[0]
        
        consist, color = radiance.consistency(color_set)
        
    if not consist:
        return False
    else:
        # BGR
        obj_r[i,j,k] += color[2]*c
        obj_g[i,j,k] += color[1]*c
        obj_b[i,j,k] += color[0]*c
        obj_num_of_cams[i,j,k] += c
        ##area block
        voxel_cord1 = np.array([[xmin + i*graity+graity], [ymin + j*graity+graity],[zmin + k*graity+graity], [1]])
        for camera in camera_set:
            pix_cord = camera_P[camera].dot(voxel_cord)
            #value = (pix_cord[2,0]-45)/30*255
            pix_cord = pix_cord / pix_cord[2,0]
            pixel_i = int(pix_cord[1])
            pixel_j = width-int(pix_cord[0])
            if pixel_i >= height or pixel_i < 0 or pixel_j >= width or pixel_j < 0:
                continue
            if image_pixel_mark_situ[camera][int(pix_cord[1,0]), width-int(pix_cord[0,0])] == 0:
                pix_cord1 = camera_P[camera].dot(voxel_cord1)
                pix_cord1 = pix_cord1 / pix_cord1[2,0]
                pix_length = int(np.linalg.norm(pix_cord-pix_cord1))
                D = int(pix_length/2)
                for m in range(max(0, pixel_i-D), min(pixel_i + D, height-1)):
                    for n in range(max(0, pixel_j-D), min(pixel_j + D, width-1)):
                        image_pixel_mark_situ[camera][m,n] = 100#value
        return True        

def rgb_image_callback(msg): 
    global rgb_flag
    global rgb_ready
    global rgbImg
    if rgb_flag == True:   
        rgb_flag = False
        try:
            rgbImg = bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_ready = True
            rospy.loginfo("rbg img saved successfully.")

        except CvBridgeError, e:
            print(e)
            
def image_saver_callback(msg):
    global rgb_flag
    global stamp
    rgb_flag = True
    stamp = msg.data
    rospy.loginfo('message received. start carving. stamp:%d',stamp)

def send_camera_position():
    global camera_position
    global pub
    msg = Vector3()
    msg.x = camera_position[0]
    msg.y = camera_position[1]
    msg.z = camera_position[2]
    pub.publish(msg)
    rospy.loginfo("new position:%f %f %f\n", msg.x, msg.y, msg.z)

def evaluation(position):
    global obj
    print(position)
    position -= 0.05/np.linalg.norm(position)*position
    P = cameraMatrix(position)

    camNumCnt = np.zeros([height, width], dtype = int)
    pixel_dist = np.ones([height, width], dtype = int)
    pixel_dist*=10
    pixel_mark = np.zeros([height, width], dtype = bool)

    cnt = 0
    for i in range(obj.shape[0]):
        for j in range(obj.shape[1]):
            for k in range(obj.shape[2]):
                if obj[i,j,k] == 0:
                    continue

                voxel_cord = np.array([[xmin + i*graity], [ymin + j*graity],[zmin + k*graity], [1]])
                pix_cord = P.dot(voxel_cord)
                dist = abs(pix_cord[2])
                pix_cord = pix_cord / pix_cord[2]
                pixel_i = int(pix_cord[1])
                pixel_j = width-int(pix_cord[0])

                if pixel_i >= height or pixel_i < 0 or pixel_j >= width or pixel_j < 0:
                    continue
            
                cnt +=1
                    
                if dist < pixel_dist[pixel_i, pixel_j]:
                    # area block
                    voxel_cord1 = np.array([[xmin + i*graity+graity], [ymin + j*graity+graity],[zmin + k*graity+graity], [1]])
                    pix_cord1 = P.dot(voxel_cord1)
                    pix_cord1 = pix_cord1 / pix_cord1[2,0]
                    pix_length = int(np.linalg.norm(pix_cord-pix_cord1))
                    D = int(pix_length/2)
                    for m in range(max(0,pixel_i - D), min(pixel_i + D, height-1)):
                        for n in range(max(0,pixel_j - D), min(pixel_j + D, width-1)):
                            pixel_dist[m,n] = dist
                            [m,n] = obj_num_of_cams[i,j,k]
                
    return float(len(camera_x_sort))*height*width-float(np.sum(camNumCnt))/((2*D+1)*(2*D+1))

def next_step_planner():
    global camera_position
    global sampleNum
    # analyse the present model and send message
    # sample on hemisphere
    cnt = 0
    bestPosition = []
    bestScore = 0
    offset = 0.3
    while cnt < sampleNum:
        u = random.random()
        v = random.random()
        r = offset + u*(0.9-offset)
        r*=0.5
        theta = v*(2*math.pi)-math.pi
        x = r*math.cos(theta)
        y = r*math.sin(theta)
        z = max(0, 0.25-x*x-y*y)
        z = math.sqrt(z)
        score = evaluation(np.array([x,y,z]))
        rospy.loginfo("cnt:%d score:%f\n",cnt, score)
        if score > bestScore:
            bestScore = score
            bestPosition = np.array([x,y,z])
        cnt+=1
    
    camera_position = bestPosition
    #camera_position = np.array([-0.0347, 0.246, 0.437])

        
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_image_callback)
    rospy.Subscriber("/saverStamp", Int8, image_saver_callback)
    #rospy.Subscriber("/cameraPositon",Vector3, camera_position_callback)
    
    rgb_ready = False
    
    obj_r *=100
    obj_g *=100
    obj_b *=100
    send_camera_position()
    viewCnt = 0
    print("ready")
    while viewCnt < numofviews:
        rospy.sleep(0.1)
        if rgb_ready:
            rgb_ready = False

            rospy.loginfo("processing..")
            inten = np.linalg.norm(rgbImg, axis = 2)
            img_mask = silhouette_extract.threshold(inten)

            # cv2.imshow("mask",img_mask)
            # cv2.waitKey(0)
            
            camera_position -= 0.025/np.linalg.norm(camera_position)*camera_position
            P = cameraMatrix(camera_position)

            # register camera position
            camera_x_sort.append((stamp, camera_position[0]))
            camera_y_sort.append((stamp, camera_position[1]))
            camera_z_sort.append((stamp, camera_position[2]))
            camera_x_sort.sort(key=takeSecond)
            camera_y_sort.sort(key=takeSecond)
            camera_z_sort.sort(key=takeSecond)
            camera_P[stamp] = P
            image_pixel_mark_situ[stamp] = np.zeros((height, width), dtype = np.int16)
            images[stamp] = rgbImg

            carve_cnt = 0
            surface_cnt = 0

            for i in range(obj.shape[0]):
                for j in range(obj.shape[1]):
                    for k in range(obj.shape[2]):
                        if obj[i,j,k] == 0:
                            continue

                        voxel_cord = np.array([[xmin + i*graity], [ymin + j*graity],[zmin + k*graity], [1]])
                        pix_cord = P.dot(voxel_cord)
                        pix_cord = pix_cord / pix_cord[2]
                        pixel_i = int(pix_cord[1])
                        pixel_j = width-int(pix_cord[0])

                        # out of the image
                        if pixel_i >= height or pixel_i < 0 or pixel_j >= width or pixel_j < 0:
                            continue

                        if img_mask[pixel_i, pixel_j] > 0:
                            #  bg
                            obj[i,j,k] = 0
                            carve_cnt = carve_cnt+1

            rospy.loginfo("visual hull finished. %d carved and %d marked as surface this round. %d unseen. %d views processed.", carve_cnt, surface_cnt, obj.size - total_carve_cnt - carve_cnt - surface_cnt, viewCnt)
            total_carve_cnt += carve_cnt
            print(evaluation(camera_position))

            # color_file = open("./visual_result.txt", "w")
            # head_line = "{} {} {} {} {} {} {} {}\n" # graity, xmin, xmax, ymin, ymax, zmin, zmax, is_colored version
            # voxel_elem_color = "{} {} {} {}\n"
            # color_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax, 1))
                    
            # for i in range(obj.shape[0]):
            #     for j in range(obj.shape[1]):
            #         for k in range(obj.shape[2]):
            #             color_file.write(voxel_elem_color.format(int(obj[i,j,k]),int(obj_r[i,j,k]), int(obj_g[i,j,k]), int(obj_b[i,j,k])))
                
            # color_file.close()

            #########################################_x_ positive_####################################
            carve_cnt = 0
            camera_set = [] 
            p = 0
            threshold = xmin-graity
            for i in range(obj.shape[0]):
                # form the camera set
                threshold += graity
                
                while(p >=0 and p < len(camera_x_sort) and camera_x_sort[p][1] < threshold):
                    camera_set.append(camera_x_sort[p][0])
                    p+=1
                    
                if len(camera_set) < 1:
                    continue

                for j in range(obj.shape[1]):
                    for k in range(obj.shape[2]):
                        if obj[i,j,k] == 0:
                            continue
                        if not consistency_check(i, j, k, camera_set):
                            obj[i,j,k] = 0
                            carve_cnt = carve_cnt+1
                            
            print("x sweep1  carved:",carve_cnt)
            

            blockRemove()

            # # # #########################################_x_negative_####################################
            carve_cnt = 0
            camera_set = [] 
            p = len(camera_x_sort)-1
            threshold = xmin + (obj.shape[0]*graity)
            for i in range(obj.shape[0]-1, 0, -1):
                # form the camera set
                threshold -= graity
                
                while(p >=0 and p < len(camera_x_sort) and camera_x_sort[p][1] > threshold):
                    camera_set.append(camera_x_sort[p][0])
                    p-=1
                    
                if len(camera_set) < 1:
                    continue

                for j in range(obj.shape[1]):
                    for k in range(obj.shape[2]):
                        if obj[i,j,k] == 0:
                            continue
                        if not consistency_check(i, j, k, camera_set):
                            obj[i,j,k] = 0
                            carve_cnt = carve_cnt+1
            print("x sweep2  carved:",carve_cnt)
            for cam in camera_set:
                cv2.imwrite("cam.ppm", image_pixel_mark_situ[cam])
            blockRemove()
            colorUpdate(0)

            # color_file = open("./visual_result_xp.txt", "w")
            # head_line = "{} {} {} {} {} {} {} {}\n" # graity, xmin, xmax, ymin, ymax, zmin, zmax, is_colored version
            # voxel_elem_color = "{} {} {} {}\n"
            # color_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax, 1))
                    
            # for i in range(obj.shape[0]):
            #     for j in range(obj.shape[1]):
            #         for k in range(obj.shape[2]):
            #             if color_record.has_key((i,j,k)):
            #                 color_file.write(voxel_elem_color.format(int(obj[i,j,k]),int(color_record[(i,j,k)][0]), int(color_record[(i,j,k)][1]), int(color_record[(i,j,k)][2])))
            #             else:
            #                 color_file.write(voxel_elem_color.format(int(obj[i,j,k]),0, 0, 0))

        
            # color_file.close()
            # # #########################################_y_positive_####################################
            carve_cnt = 0
            camera_set = [] 
            p = 0
            threshold = ymin-graity
            for j in range(obj.shape[1]):
                # form the camera set
                threshold += graity
                
                while(p >=0 and p < len(camera_y_sort) and camera_y_sort[p][1] < threshold):
                    camera_set.append(camera_y_sort[p][0])
                    p+=1

                if len(camera_set) < 1:
                    continue

                for i in range(obj.shape[0]):
                    for k in range(obj.shape[2]):
                        if obj[i,j,k] == 0:
                            continue
                        if not consistency_check(i, j, k, camera_set):
                            obj[i,j,k] = 0
                            carve_cnt = carve_cnt+1
            print("y sweep1  carved:",carve_cnt)
            blockRemove()

            # # # #########################################_y_negative_####################################
            carve_cnt = 0
            camera_set = [] 
            p = len(camera_y_sort)-1
            threshold = ymin + (obj.shape[1]*graity)
            for j in range(obj.shape[1]-1, 0, -1):
                # form the camera set
                threshold -= graity
                
                while(p >=0 and p < len(camera_y_sort) and camera_y_sort[p][1] > threshold):
                    camera_set.append(camera_y_sort[p][0])
                    p -= 1
                    
                if len(camera_set) < 1:
                    continue

                for i in range(obj.shape[0]):
                    for k in range(obj.shape[2]):
                        if obj[i,j,k] == 0:
                            continue
                        if not consistency_check(i, j, k, camera_set):
                            obj[i,j,k] = 0
                            carve_cnt = carve_cnt+1
            print("y sweep2  carved:",carve_cnt)
            blockRemove()
            colorUpdate(1)
        
            # # # #########################################_z_positive_####################################
            carve_cnt = 0
            camera_set = [] 
            p = 0
            threshold = zmin-graity
            for k in range(obj.shape[2]):
                # form the camera set
                threshold += graity
                
                while(p >=0 and p < len(camera_z_sort) and camera_z_sort[p][1] < threshold):
                    camera_set.append(camera_z_sort[p][0])
                    p+=1

                if len(camera_set) < 1:
                    continue

                for i in range(obj.shape[0]):
                    for j in range(obj.shape[1]):
                        if obj[i,j,k] == 0:
                            continue
                        if not consistency_check(i, j, k, camera_set):
                            obj[i,j,k] = 0
                            carve_cnt = carve_cnt+1
            print("z sweep1  carved:",carve_cnt)
            blockRemove()
            
            # # # #########################################_z_negative_####################################
            carve_cnt = 0
            camera_set = [] 
            p = len(camera_z_sort)-1
            threshold = zmin + (obj.shape[0]*graity)
            for k in range(obj.shape[2]-1, 0, -1):
                # form the camera set
                threshold -= graity
                
                while(p >=0 and p < len(camera_z_sort) and camera_z_sort[p][1] > threshold):
                    camera_set.append(camera_z_sort[p][0])
                    p-=1
                    
                if len(camera_set) < 1:
                    continue

                for j in range(obj.shape[1]):
                    for i in range(obj.shape[0]):
                        if obj[i,j,k] == 0:
                            continue
                        if not consistency_check(i, j, k, camera_set):
                            obj[i,j,k] = 0
                            carve_cnt = carve_cnt+1
            print("z sweep2  carved:",carve_cnt)
            blockRemove()
            colorUpdate(2)

            viewCnt += 1
            if viewCnt == numofviews:
                break
            print("writing to file..")

            color_file = open("./rgb_result_"+str(viewCnt)+".txt", "w")
            head_line = "{} {} {} {} {} {} {} {}\n" # graity, xmin, xmax, ymin, ymax, zmin, zmax, is_colored version
            voxel_elem_color = "{} {} {} {}\n"
            color_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax, 1))
                    
            for i in range(obj.shape[0]):
                for j in range(obj.shape[1]):
                    for k in range(obj.shape[2]):
                        if color_record.has_key((i,j,k)):
                            color_file.write(voxel_elem_color.format(int(obj[i,j,k]),int(color_record[(i,j,k)][0]), int(color_record[(i,j,k)][1]), int(color_record[(i,j,k)][2])))
                        else:
                            color_file.write(voxel_elem_color.format(int(obj[i,j,k]),0, 0, 0))
                
            color_file.close()
            pass
            next_step_planner()
            send_camera_position()


    # write the result into files
    print("carving finished, result info:")
    print(obj.size, obj.size-total_carve_cnt)
    print("writing to file..")

    color_file = open("./rgb_result.txt", "w")
    head_line = "{} {} {} {} {} {} {} {}\n" # graity, xmin, xmax, ymin, ymax, zmin, zmax, is_colored version
    voxel_elem_color = "{} {} {} {}\n"
    color_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax, 1))
            
    for i in range(obj.shape[0]):
        for j in range(obj.shape[1]):
            for k in range(obj.shape[2]):
                if color_record.has_key((i,j,k)):
                    color_file.write(voxel_elem_color.format(int(obj[i,j,k]),int(color_record[(i,j,k)][0]), int(color_record[(i,j,k)][1]), int(color_record[(i,j,k)][2])))
                else:
                    color_file.write(voxel_elem_color.format(int(obj[i,j,k]),0, 0, 0))
        
    color_file.close()
    pass