#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('egolane')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#import my message ego::datax
from egolane.msg import datax

# import the necessary packages
#from collections import deque
#from imutils.video import VideoStream
import numpy
import numpy as np
#import argparse
import cv2
import imutils
import time
import array as arr

bridge=CvBridge()
#pub=None
image=None
angle=1
left=1
right=1
BoolAngle=0
BoolLeft=0
BoolRight=0
BoolImage=0

def angle_callback(angle_msg):
    global BoolAngle, angle
    rospy.loginfo(rospy.get_caller_id() + 'I heard angle %f', angle_msg.x0)
    angle = angle_msg.x0
    BoolAngle=1
    
def left_callback(left_msg):
    global BoolLeft, left
    rospy.loginfo(rospy.get_caller_id() + 'I heard left %f', left_msg.x0)
    left = left_msg.x0
    BoolLeft=1
    
def right_callback(right_msg):
    global BoolRight, right
    rospy.loginfo(rospy.get_caller_id() + 'I heard right %f', right_msg.x0)
    right = right_msg.x0
    BoolRight=1
    
def image_callback(image_msg):
  global BoolImage, image
  try:
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    print ("Imagen Recibida")
    BoolImage=1
    #rospy.sleep(0.1)
  except CvBridgeError as e:
    print(e)

def display():
    global pub
    global image
    global angle
    global left
    global right
    global BoolAngle
    global BoolLeft
    global BoolRight
    global BoolImage

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('display', anonymous=True)

    #pub = rospy.Publisher('messagePoints', datax, queue_size=1)

    rospy.Subscriber("image_topic", Image, image_callback)

    rospy.Subscriber('anglepredicted', datax, angle_callback)
    rospy.Subscriber('LeftDistance', datax, left_callback)
    rospy.Subscriber('RightDistance', datax, right_callback)
    waitloop=rospy.Rate(30)

    while not rospy.is_shutdown():
      if (BoolImage==1):
        (h, w, d) = image.shape
        print("width={}, height={}, depth={}".format(w, h, d))
        distance=320
        startAngle1=270
        if (BoolAngle==1) and (BoolLeft==1) and (BoolRight==1) and (BoolImage==1):
          #(h, w, d) = image.shape
          #print("width={}, height={}, depth={}".format(w, h, d))
          #image = imutils.resize(image, w)
          #cv2.imshow('PRUEBA',image)
          #Display the image
          distance=((left/(left+right))-0.5)*350 + 320
          startAngle1=270+angle
          
        radius=100
        axes = (200,radius)
        angle_elipse=0
        startAngle=180
        endAngle=360
        center=(321,150)
        color=(0,0,0)
        
        radius1=100
        axes1 = (200,radius)
        angle1=0
        
        endAngle1=270;
        center1=(321,150)
        color1=(0,0,255)
          

        
        cv2.ellipse(image, center, axes, angle_elipse, startAngle, endAngle, color, 4)
        cv2.ellipse(image, center1, axes1, angle1, int(startAngle1), endAngle1, color1, -1)
        
        #cv2.line(image,(327,142),(int(distance),142),(255,36,0),15)
        cv2.rectangle(image, (320, 150), (int(distance), 135), (255,36,0),-1)
        cv2.line(image,(121,150),(521,150),(0,0,0),2)
        cv2.line(image,(321,150),(321,50),(0,0,0),2)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image,'0',(318,45), font, 1,(0,0,0),2,cv2.LINE_AA)
        cv2.putText(image,'-90',(50,150), font, 1,(0,0,0),2,cv2.LINE_AA)
        cv2.putText(image,'+90',(526,150), font, 1,(0,0,0),2,cv2.LINE_AA)
            
        BoolAngle=0
        BoolLeft=0
        BoolRight=0
        BoolImage=0
        #show the frame to our screen
        cv2.imshow('Display',image)
        cv2.waitKey(1)
      waitloop.sleep()
        
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
  display()
