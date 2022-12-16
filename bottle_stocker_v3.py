#!/usr/bin/python2
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append("/home/developer/anaconda3/lib/python3.6/site-packages")
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
import baxter_interface
from baxter_interface import CHECK_VERSION
from std_msgs.msg import String
import ast
import threading

# COLOR IN BGR FORMAT
selected_color=(174, 158, 237)#(148,128,244)#(209,114,126)#/(247,105,126)
lower_bound = np.array((87, 67, 136))
upper_bound = np.array((197, 181, 255))


def image_callback(image_data):
    #Convert received image message to OpenCv image
    cv_image = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)

    hsv_im = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_im, lower_bound, upper_bound)

    kernel = np.ones((7,7), np.uint8)

    filtered_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    filtered_mask_2 = cv2.morphologyEx(filtered_mask, cv2.MORPH_OPEN, kernel)

    segmented_im = cv2.bitwise_and(cv_image, cv_image, mask=filtered_mask_2)

    # Draw a boundary of the detected objects
    contours, heirarchy = cv2.findContours(filtered_mask_2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    y_upper=700
    y_lower=500
    x0=500
    x1=750
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        cv2.line(cv_image,(x0,y_lower),(x0,y_upper),(0,0,255),2) # Left Red Line
        cv2.line(cv_image,(x1,y_lower),(x1,y_upper),(0,0,255),2) # Right Red Line
        if(area > 10 and area < 500):
            x, y, w, h = cv2.boundingRect(contour)
            if y_lower<y<y_upper:
                if x0<x<x1:
                    imageFrame = cv2.rectangle(cv_image, (x, y),
                                       (x + w, y + h),
                                       selected_color, 2)
                    cv2.putText(cv_image, "RUN EMPTY COMMAND", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (selected_color))
                    can_pub.publish("1")
                else:
                    can_pub.publish("0")
            else:
                can_pub.publish("0")

    cv2.imshow('Image', cv_image)
    # display image
    cv2.waitKey(1) 
   
if __name__ == '__main__':
    rospy.init_node('Camera_Subscriber',anonymous=True)
    print("Calibrating Grippers...")
    leftGripper = baxter_interface.Gripper('left', CHECK_VERSION)
    rightGripper = baxter_interface.Gripper('right', CHECK_VERSION)
    leftGripper.calibrate(*[])
    rightGripper.calibrate(*[])
    # Subscribe to head_camera image topic
    rospy.Subscriber('/cameras/head_camera/image', Image, image_callback)
    # Publish to can_states
    can_pub=rospy.Publisher('can_states',String,queue_size=10)
    rospy.sleep(0.1)
    rospy.spin()
    # sleep
    cv2.destroyAllWindows() # Destroy CV image window on shut_down
