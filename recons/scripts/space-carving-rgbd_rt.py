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

total_carve_cnt = 0

bridge = CvBridge()
rgb_flag = False
depth_flag = False
rgb_ready = True
depth_ready = True
sampleNum = 10
stamp = 0
rgbImg = []
depthImg = []
camera_position = np.array([0.433, 0, 0.25])#)#[-0.0347, 0.246, 0.437])#)
pub = rospy.Publisher("/cameraPosition",Vector3, queue_size = 10 )
numofviews = 5

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


def depth_image_callback(msg): 
    global depth_flag
    global depth_ready
    global depthImg
    if depth_flag == True:  
        depth_flag = False
        try:
            depthImg = bridge.imgmsg_to_cv2(msg, "32FC1")
            depth_ready = True
            rospy.loginfo("depth img saved successfully.")
            
        except CvBridgeError, e:
            print(e)

            
def image_saver_callback(msg):
    global rgb_flag
    global depth_flag
    global stamp
    rgb_flag = True
    depth_flag = True
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
    global obj_r
    global obj_g
    global obj_b
    global obj

    position -= 0.025/np.linalg.norm(position)*position
    P = cameraMatrix(position)

    pixel_grid = np.zeros([height, width], dtype = int)

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

                if pixel_i >= height or pixel_i < 0 or pixel_j >= width or pixel_j < 0 or pixel_grid[pixel_i, pixel_j] == -1:
                    continue

                if obj_r[i,j,k] == 100 and obj_g[i,j,k] == 100 and obj_b[i,j,k] == 100:
                    pixel_grid[pixel_i, pixel_j] += 1
                
                elif obj[i, j, k] > 0:
                    pixel_grid[pixel_i, pixel_j] = -1
    cnt = 0
    for i in range(height):
        for j in range(width):
            if pixel_grid[i,j] != -1:
                cnt += pixel_grid[i,j]
    return cnt

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
        r = offset + u*(0.8-offset)
        r*=0.5
        theta = v*(2*math.pi)-math.pi
        x = r*math.cos(theta)
        y = r*math.sin(theta)
        z = max(0, 0.25-x*x-y*y)
        z = math.sqrt(z)
        score = evaluation(np.array([x,y,z]))
        rospy.loginfo("cnt:%d score:%d\n",cnt, score)
        if score > bestScore:
            bestScore = score
            bestPosition = np.array([x,y,z])
        cnt+=1
    
    camera_position = bestPosition
    #camera_position = np.array([-0.0347, 0.246, 0.437])

        
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_raw", Image, rgb_image_callback)
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_image_callback) 
    rospy.Subscriber("/saverStamp", Int8, image_saver_callback)
    #rospy.Subscriber("/cameraPositon",Vector3, camera_position_callback)
    
    rgb_ready = False
    depth_ready = False
    
    obj_r *=100
    obj_g *=100
    obj_b *=100
    send_camera_position()
    viewCnt = 0
    print("ready")
    while viewCnt < numofviews:
        rospy.sleep(0.1)
        if rgb_ready and depth_ready:
            rgb_ready = False
            depth_ready = False

            rospy.loginfo("processing..")
            inten = np.linalg.norm(rgbImg, axis = 2)
            img_mask = silhouette_extract.threshold(inten)

            cv2.imshow("mask",img_mask)
            cv2.waitKey(0)
            
            #camera_position = np.array([0.866, 0, 0.5])
            # else:
            #     camera_position = np.array([0.433, -0.75, 0.5])
            camera_position -= 0.025/np.linalg.norm(camera_position)*camera_position
            P = cameraMatrix(camera_position)

            carve_cnt = 0
            surface_cnt = 0

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

                        # out of the image
                        if pixel_i >= height or pixel_i < 0 or pixel_j >= width or pixel_j < 0:
                            continue

                        if img_mask[pixel_i, pixel_j] == 0 and np.linalg.norm(dist - depthImg[pixel_i, pixel_j]) < 3*graity:#seen
                            # surface: not bg, close to range info
                            obj[i, j, k] = 1
                            obj_r[i,j,k] = rgbImg[pixel_i, pixel_j, 2]
                            obj_g[i,j,k] = rgbImg[pixel_i, pixel_j, 1]
                            obj_b[i,j,k] = rgbImg[pixel_i, pixel_j, 0]
                            surface_cnt += 1

                        # Pixels with intensity values 0 (black) denote object occupancy
                        elif dist < depthImg[pixel_i, pixel_j] or img_mask[pixel_i, pixel_j] > 0:
                            # front, bg
                            obj[i,j,k] = 0
                            carve_cnt= carve_cnt+1

            viewCnt = viewCnt+1
            rospy.loginfo("finished. %d carved and %d marked as surface this round. %d unseen. %d views processed.", carve_cnt, surface_cnt, obj.size - total_carve_cnt - carve_cnt - surface_cnt, viewCnt)
            total_carve_cnt += carve_cnt
            
            if viewCnt == numofviews:
                break

            color_file = open("./rgbd_result"+ str(viewCnt)+".txt", "w")
            head_line = "{} {} {} {} {} {} {} {}\n" # graity, xmin, xmax, ymin, ymax, zmin, zmax, is_colored version
            voxel_elem_color = "{} {} {} {}\n"
            color_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax, 1))
                    
            for i in range(obj.shape[0]):
                for j in range(obj.shape[1]):
                    for k in range(obj.shape[2]):
                        color_file.write(voxel_elem_color.format(int(obj[i,j,k]),int(obj_r[i,j,k]), int(obj_g[i,j,k]), int(obj_b[i,j,k])))

                
            color_file.close()
            pass

            next_step_planner()
            send_camera_position()


    # write the result into files
    print("carving finished, result info:")
    print(obj.size, obj.size-total_carve_cnt)
    print("writing to file..")

    color_file = open("./rgbd_result.txt", "w")
    head_line = "{} {} {} {} {} {} {} {}\n" # graity, xmin, xmax, ymin, ymax, zmin, zmax, is_colored version
    voxel_elem_color = "{} {} {} {}\n"
    color_file.write(head_line.format(graity, xmin, xmax, ymin, ymax, zmin, zmax, 1))
            
    for i in range(obj.shape[0]):
        for j in range(obj.shape[1]):
            for k in range(obj.shape[2]):
                color_file.write(voxel_elem_color.format(int(obj[i,j,k]),int(obj_r[i,j,k]), int(obj_g[i,j,k]), int(obj_b[i,j,k])))
        
    color_file.close()
    pass