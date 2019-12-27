#!/usr/bin/env python


import sys
import copy
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from moveit_commander.conversions import pose_to_list
from tf import transformations
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()
ready_to_save_rgb = False
ready_to_save_depth = False
c = 0
pub = rospy.Publisher("/saverStamp",Int8, queue_size = 100)
group=[]
lastPosition = []

def angleComputation(lastPosition):
    lengthxy = math.sqrt(lastPosition[0,0]*lastPosition[0,0] + lastPosition[1,0]*lastPosition[1,0])
    theta = math.atan(lastPosition[2,0]/lengthxy)
    # the vectex is actually position -> (0,0,0)
    phi = math.acos(-lastPosition[0,0]/lengthxy)
    if -lastPosition[1,0] < 0:
        phi *= -1
    return [theta, phi]


def camera_position_callback(data):
    global group
    global c
    global lastPosition
    global pub
    print("moving command received.")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = data.x
    pose_goal.position.y = data.y
    pose_goal.position.z = data.z

    theta, phi = angleComputation(np.array([[data.x], [data.y], [data.z]]))
    orientation = transformations.quaternion_about_axis(theta, (0,1,0))
    orientation = transformations.quaternion_multiply(transformations.quaternion_about_axis(phi, (0,0,1)), orientation)
    pose_goal.orientation.x = orientation[0]
    pose_goal.orientation.y = orientation[1]
    pose_goal.orientation.z = orientation[2]
    pose_goal.orientation.w = orientation[3]
    
    lastPosition = [data.x, data.y, data.z]

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    
    group.stop() 
    group.clear_pose_targets()  

    rospy.sleep(1)

    c+=1
    pub.publish(Int8(c))
    print("in position")



def listener():
    global group

    # check if the execution if finished, call callback
    rospy.Subscriber("/cameraPosition",Vector3, camera_position_callback)
    
    rospy.spin()

# def talker():
#     # publish the next place
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()


if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('planner', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface(synchronous = True)

    rospy.sleep(0.1)

    plane_pose = geometry_msgs.msg.PoseStamped()
    plane_pose.header.frame_id = "world"
    plane_pose.pose.position.x = 0
    plane_pose.pose.position.y = 0
    plane_pose.pose.position.z = 0
    plane_pose.pose.orientation.w = 1.0
    scene.add_plane("ground_plane",plane_pose)

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.05
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    

    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = robot.get_planning_frame()
    # box_pose.pose.orientation.w = 1.0
    # #box_pose.pose.position.z = 0 # slightly above the end effector
    # box_name = "camera"
    # scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))




    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

        # check if the execution if finished, call callback
    rospy.Subscriber("/cameraPosition",Vector3, camera_position_callback)
    
    rospy.spin()

    # c = 0

    # orientation = transformations.quaternion_about_axis(pi/2, (0,0,-1))
    # last_alpha = 0
    # for i in range(3):
    #     alpha = pi/3 - i*pi/9
    #     orientation = transformations.quaternion_multiply(transformations.quaternion_about_axis(alpha-last_alpha, (1,0,0)), orientation)
    #     last_alpha = alpha
    #     pose_goal = geometry_msgs.msg.Pose()
    #     pose_goal.position.x = 0
    #     pose_goal.position.y = math.cos(alpha)
    #     pose_goal.position.z = math.sin(alpha)

    #     pose_goal.orientation.x = orientation[0]
    #     pose_goal.orientation.y = orientation[1]
    #     pose_goal.orientation.z = orientation[2]
    #     pose_goal.orientation.w = orientation[3]
        
    #     group.set_pose_target(pose_goal)
    #     plan = group.go(wait=True)
    #     group.stop() 
    #     group.clear_pose_targets()
        
    #     rospy.loginfo("%d", c)
    #     pub.publish(Int8(c))
    #     c=c+1
        
    #     # rospy.sleep(10)
    #     for j in range(5):
    #         joint_goal = group.get_current_joint_values()
    #         print(joint_goal[0])
    #         joint_goal[0] = joint_goal[0] + pi/3
    #         if joint_goal[0] > 2*pi:
    #             joint_goal[0] = 2*pi

    #         plan = group.go(joint_goal, wait=True)
    #         group.stop()
    #         group.clear_pose_targets()
    #         msg = Int8()
    #         msg.data = c
    #         c = c+1
    #         if msg.data == 9:
    #             rospy.loginfo("%d", msg.data)
    #             pub.publish(msg)
    #             rospy.sleep(10)




            
    # listener()
    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass
