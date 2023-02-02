#! /usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

THRESH = 80 # Anyting between 120-80 works 
CONTOUR_COLOR = (0, 255, 0)
CONTOUR_THICKNESS = 6
PREVIOUS_CENTER = 0

def get_contour(frame_gray, frame):
  """
  @brief adds contours to a frame with color CONTOUR_COLOR, thinkness CONTOUR_THICKNESS, by threshold THRESH

  @params ndarray frame_gray
  @params ndarray frame

  @returns ndarray with_contours a frame with contour lines 
  """
  ret, img_bin = cv.threshold(frame_gray, THRESH, 255, 0)
  contours, hierarchy = cv.findContours(img_bin, cv.RETR_TREE, 
                                        cv.CHAIN_APPROX_SIMPLE)
  with_contours = cv.drawContours(np.copy(frame), contours, -1, CONTOUR_COLOR, 
                                  CONTOUR_THICKNESS)
  return with_contours



def tuple_list_eq_3(t: tuple, l: list):
    return l[0] == t[0] and l[1] == t[1] and l[2] == t[2] 

def callback(data):
    global PREVIOUS_CENTER
    prev_used = False
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    rate = rospy.Rate(2)
    move = Twist()
    move.linear.x = 0.2
    #move.angular.z = 0.5
    shape = cv_image.shape

    x_axis_len = shape[1]
    y_axis_len = shape[0]
    
    y_axis_margin = y_axis_len - int(y_axis_len / 12) # pixles
    x_axis_margin = int(CONTOUR_THICKNESS * 1.5)
    
    frame_gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY) # Converts normal rgb to gray scale
    with_contours = get_contour(frame_gray, cv_image)
    

    def find_contur(x_start):
        for i in reversed(range(0, x_start)):
            color = with_contours[y_axis_margin][i]
            if tuple_list_eq_3(CONTOUR_COLOR, color):
                return i
    
    right_coordinates = find_contur(x_axis_len - x_axis_margin)

    if not right_coordinates == None:
        left_coordinates = find_contur(right_coordinates - x_axis_margin)
        if not left_coordinates == None:
            path_center = int((right_coordinates - left_coordinates) / 2) + left_coordinates + 80
        else: 
            path_center = right_coordinates
            prev_used = True
            move.linear.x = 0.05 
    else:
        y_axis_margin = y_axis_len - 450
        right_coordinates = find_contur(x_axis_len - x_axis_margin)
        print(right_coordinates)
        path_center = PREVIOUS_CENTER
        prev_used = True

    PREVIOUS_CENTER = path_center

    if path_center < x_axis_len / 2:
        if prev_used == True:
            move.angular.z = 1.0
        else:
            move.angular.z = 0.5
    elif path_center > x_axis_len /2:
        if prev_used == True:
            move.angular.z = -1.0
        else:
            move.angular.z = -0.5

    cv.imshow("Image window", with_contours)
    cv.waitKey(3)

    #try: 
        #image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
        #print(e)

    move_pub.publish(move)
    return None

#image_pub = rospy.Publisher("image_topic_2",Image)
bridge = CvBridge()
image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image, callback)
move_pub = rospy.Publisher('/cmd_vel', Twist, 
  queue_size=1)


rospy.init_node('image_converter', anonymous=True)

try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
cv.destroyAllWindows()
