#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*        		===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        		===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1b_boiler_plate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image


##################### FUNCTION DEFINITIONS #######################

def rotation_matrix_to_quaternion(R):
    q0 = np.sqrt((R[0, 0] + R[1, 1] + R[2, 2] + 1) / 4)
    q1 = (R[2, 1] - R[1, 2]) / (4 * q0)
    q2 = (R[0, 2] - R[2, 0]) / (4 * q0)
    q3 = (R[1, 0] - R[0, 1]) / (4 * q0)

    q = np.array([q0, q1, q2, q3]) 
    q = q / np.linalg.norm(q)
    return q

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    # Reshape the coordinates to a list of 4 corners
    corners = np.reshape(coordinates, (4, 2))

    # Calculate the area and width
    area = 0
    width = 0
    for i in range(4):
        a = corners[i][0] - corners[(i+1)%4][0]
        b = corners[i][1] - corners[(i+1)%4][1]
        c = corners[(i+1)%4][0] - corners[(i+2)%4][0]
        d = corners[(i+1)%4][1] - corners[(i+2)%4][1]
        area += abs(a * b + d * c)/2
        width += math.sqrt((a**2 + b**2) * (c**2 + d**2))
    width = math.sqrt(area)

    return area, width

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])
 

def get_quaternion_from_rvec(rx, ry, rz):

    # R, _ = cv2.Rodrigues(np.array([rx, ry, rz]))
    # r1 = cv2.Rodrigues(np.array([-np.pi/2, 0, 0]))
    # r2 = cv2.Rodrigues(np.array([0,0,-np.pi/2]))
    # rvec, _ = cv2.Rodrigues(R*r1*r2)
    
    print("rvec: ", rx, ry, rz)
    theta = np.linalg.norm(np.array([rx, ry, rz]))  # magnitude of rotation vector
    qw = np.cos(theta/2)
    qx = np.sin(theta/2) * rx
    qy = np.sin(theta/2) * ry
    qz = np.sin(theta/2) * rz

    q = np.array([qw, qx, qy, qz])

    q1 = np.array([1/np.sqrt(2),1/np.sqrt(2), 0,0]) # x clk
    q2 = np.array([1/np.sqrt(2),0,1/np.sqrt(2),0]) # y clk
    q3 = np.array([1/np.sqrt(2),0,0 ,1/np.sqrt(2)]) # z clk

    q = quaternion_multiply(q, q3)
    q = quaternion_multiply(q, q3)
    q = quaternion_multiply(q, q3)

    q = quaternion_multiply(q, q1)
    q = quaternion_multiply(q, q1)
    return q

def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    # aruco_area_threshold = 1500
    aruco_area_threshold=150

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 

    # DONE	->  Convert input BGR image to GRAYSCALE for aruco detection 

    # DONE  ->  Use these aruco parameters-
    # DONE      ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    # DONE  ->  Detect aruco marker in the image and store 'corners' and 'ids'
    # DONE      ->  HINT: Handle cases for empty markers detection. 

    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    Detected_ArUco_markers = {}
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids_, rejectedImgPoints = detector.detectMarkers(gray)
    print("____________________________________________________________________________")
    print(ids_)
    if ids_ is None:
        ids_ = []
        print("No aruco in front of me")
        return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

    objpoint = np.array([[-size_of_aruco_m/2, size_of_aruco_m/2, 0],
                         [size_of_aruco_m/2, size_of_aruco_m/2, 0],
                         [size_of_aruco_m/2, -size_of_aruco_m/2, 0],
                         [-size_of_aruco_m/2, -size_of_aruco_m/2, 0],
                         ])
    # cv2.aruco.drawDetectedMarkers(image, corners, ids)
    # https://automaticaddison.com/estimate-aruco-marker-pose-using-opencv-gazebo-and-ros-2/
    # https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
    # new_image = gray.copy()
    # new_image = new_image.astype(float)

    rvecs = []
    tvecs = []
    frame_markers = image.copy()
    i = 0
    for id in ids_:
        for id_Number in id:

            # Update IDs
            ids.append(id_Number)

            # Get corners
            aruco_corners = corners[i][0]
            Detected_ArUco_markers[id_Number] = aruco_corners
            print("New marker id: "+str(id_Number))

            # Get centers
            center_aruco_list.append(get_center(aruco_corners))
            print("center: "+str(center_aruco_list[i]))

            # Get area and width
            area, width  = calculate_rectangle_area(aruco_corners)
            print("area: "+str(area))
            print("width: "+str(width))

            # Remove tags which are far away from arm's reach position based on some threshold defined
            if area < aruco_area_threshold:
                print("area smaller than threshold")
                continue

            width_aruco_list.append(width)
            
            # Calculate distance from RGB camera using pose estimation of aruco marker
            rvec = np.zeros((3,1))
            tvec = np.zeros((3,1)) 
            flag = cv2.SOLVEPNP_ITERATIVE # tried with SOLVEPNP_EPNP, same error.
            retval,  rvec, tvec = cv2.solvePnP(objpoint, aruco_corners, cam_mat, dist_mat, rvec, tvec, flags=flag)        
            rvecs.append(rvec)
            tvecs.append(tvec)    
            # rvec = [[-1,0,0],[0,1,0],[0,0,-1]]@rvec
            distance_from_rgb_list.append(np.linalg.norm(tvec))
            angle_aruco_list.append(np.arctan2(rvec[1][0], rvec[0][0])) # Calculate angle of all pose estimated for aruco marker
            
            print(f"rvec: \n{rvec}\n tvec: \n{tvec} \n dist: {distance_from_rgb_list[i]} \n angle: {angle_aruco_list[i]}")
            i+=1
            frame_markers = cv2.drawFrameAxes(frame_markers, cam_mat, dist_mat, rvec, tvec, size_of_aruco_m, 1)

    frame_markers = cv2.aruco.drawDetectedMarkers(frame_markers, corners, ids_)
    
    

    cv2.imshow("Nice",frame_markers)
    cv2.waitKey(1)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, rvecs

def get_center(corners):   
    return (corners[0] + corners[1] + corners[2] + corners[3]) / 4

def rotation_to_matrix(rotation):
    q1, q2, q3, q0 = rotation/np.linalg.norm(rotation)
    return np.array([
        [q0**2+q1**2-q2**2-q3**2,   2*q3*q0+2*q1*q2,            2*q1*q3-2*q0*q2],
        [2*q1*q2-2*q3*q0,           q0**2-q1**2+q2**2-q3**2,    2*q1*q0+2*q3*q2],
        [2*q0*q2+2*q3*q1,           2*q2*q3-2*q0*q1,            q0**2-q1**2-q2**2+q3**2]
    ])

def tf_to_mat(t):
    translation = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    rotation = np.array([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
    matrix = np.identity(4)
    matrix[:3, :3] = rotation_to_matrix(rotation).T  # See note
    matrix[:3, 3] = translation.T

    return matrix
##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################
        try:
            # print(type(data))
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

        except CvBridgeError as e:
            self.get_logger().error("CvBridge Error: {0}".format(e))


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error("CvBridge Error: {0}".format(e))


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''
        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
            

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above
        if self.cv_image is None or self.depth_image is None:
            return
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids, rvecs = detect_aruco(self.cv_image)
        # detect_aruco(self.depth_image)
        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 
        for i in range(len(ids)):
        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
            angle_aruco = angle_aruco_list[i]
            angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)
            q = get_quaternion_from_rvec(rvecs[i][0][0], rvecs[i][1][0], angle_aruco)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)
            cX = center_aruco_list[i][0]
            cY = center_aruco_list[i][1]
            distance_from_rgb = self.depth_image[int(np.floor(cY))][int(np.floor(cX))]/1000 # depth = depth_image[cY][cX]/1000
        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
            # distance_from_rgb = distance_from_rgb_list[i]
            x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
            y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
            z = distance_from_rgb
            
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 
            image = cv2.circle(self.cv_image, (int(cX), int(cY)), 5, (0, 0, 255), -1)
        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
            frame_id = 'camera_link'
            child_frame_id = "cam_"+str(ids[i])       #   Ex: cam_20, where 20 is aruco marker ID
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = frame_id
            t.child_frame_id = child_frame_id
            t.transform.translation.y = x
            t.transform.translation.z = y
            t.transform.translation.x = z
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            self.br.sendTransform(t)
            # print("TF send result: ", t)

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 
            
        #     print(self.tf_buffer.can_transform('base_link', child_frame_id, now))
        #     self.t_ = TransformStamped()
        #     frame_id = 'base_link'
            t_cam_base = self.tf_buffer.lookup_transform("base_link", "camera_link" ,  self.get_clock().now(), rclpy.duration.Duration(seconds=1))
            t_obj_base_mat = np.dot(tf_to_mat(t_cam_base),tf_to_mat(t))
            print("object", str(ids[i]))
            print("Calculated obj wrt cam Transform")
            print("x:  ", t.transform.translation.x)
            print("y:  ", t.transform.translation.y)
            print("z:  ", t.transform.translation.z)
            print("qx: ", t.transform.rotation.x)
            print("qy: ", t.transform.rotation.y)
            print("qz: ", t.transform.rotation.z)
            print("qw: ", t.transform.rotation.w)

            print("Object wrt cam matrix:")
            print(tf_to_mat(t))

            print("Calculated cam wrt base Transform")
            print("x:  ", t_cam_base.transform.translation.x)
            print("y:  ", t_cam_base.transform.translation.y)
            print("z:  ", t_cam_base.transform.translation.z)
            print("qx: ", t_cam_base.transform.rotation.x)
            print("qy: ", t_cam_base.transform.rotation.y)
            print("qz: ", t_cam_base.transform.rotation.z)
            print("qw: ", t_cam_base.transform.rotation.w)

            print("Camera wrt Base matrix:")
            print(tf_to_mat(t_cam_base))

            q = rotation_matrix_to_quaternion(t_obj_base_mat[:3,:3])

            print("Result matrix:")
            print(t_obj_base_mat)

            print("Calculated result Transform")
            print("x:  ", t_obj_base_mat[0,3])
            print("y:  ", t_obj_base_mat[1,3])
            print("z:  ", t_obj_base_mat[2,3])
            print("qx: ", q[1])
            print("qy: ", q[2])
            print("qz: ", q[3])
            print("qw: ", q[0])

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = "obj_"+str(ids[i]) 
            t.transform.translation.x = t_obj_base_mat[0,3]
            t.transform.translation.y = t_obj_base_mat[1,3]
            t.transform.translation.z = t_obj_base_mat[2,3]

            t.transform.rotation.x = float(q[1])
            t.transform.rotation.y = float(q[2])
            t.transform.rotation.z = float(q[3])
            t.transform.rotation.w = float(q[0])
            self.br.sendTransform(t)

        # #       Use the following frame_id-

        # #   ->  And now publish TF between object frame and base_link
        # #       Use the following frame_id-
        #     child_frame_id = 'obj_' + str(ids[i])      #    Ex: obj_20, where 20 is aruco marker ID
        #     self.t_.child_frame_id = child_frame_id
            # self.br.sendTransform(self.t_)    

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/
        # cv2.imshow('Frame', image)

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1C)
        #               Also, auto eval script will be judging angular difference as well. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()
