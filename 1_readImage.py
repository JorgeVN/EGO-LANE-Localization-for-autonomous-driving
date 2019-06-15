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
from collections import deque
from imutils.video import VideoStream
import numpy
import numpy as np
import argparse
import cv2
import imutils
import time
import array as arr

bridge=CvBridge()
pub=None


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    
def image_callback(image_msg):
  try:
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    #cv_image = cv_img.toImageMsg(image_message, desired_encoding="passthrough")
    print ("Imagen Recibida")
  except CvBridgeError as e:
    print(e)
  #cv2.imshow("Image", cv_image)
  (h, w, d) = image.shape
  print("width={}, height={}, depth={}".format(w, h, d))
  #cv2.waitKey(0)
  blueLower = (100,120, 0)	
  blueUpper = (140, 255, 250)	
  frame = imutils.resize(image, width=600)
  blurred = cv2.GaussianBlur(frame, (11, 11), 0)	
  hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(hsv, blueLower, blueUpper)	
  mask = cv2.dilate(mask, None, iterations=6)
  #cv2.imshow('mask',mask) 
  #cv2.waitKey(0)
  mask = cv2.erode(mask, None, iterations=30)
  #cv2.imshow('mask',mask) 
  #cv2.waitKey(0.01)
  mask = cv2.dilate(mask, None, iterations=24)
  h=frame.shape[0]
  w=frame.shape[1]
  pxi = []
  pxd = []
  py = []	
 #Guardar tambien todos los puntos en un txt para acceder a ellos facilmente
  #pyy=100329709
  #with open('YY_data.txt', 'w') as fy:
   # print >>fy, pyy
  #with open('XXi_data.txt', 'w') as fxi:
   # print >>fxi, pyy
  #with open('XXd_data.txt', 'w') as fxd:
   # print >>fxd, pyy
	
  for y in range (h/2, h):
	#pixeles contorno izquierdo
    for x in range (1, w):
	pix1= mask [y, x]
	if (pix1==255):
		py = np.append(py, y)
		pxi = np.append(pxi, x)
		
		#with open('YY_data.txt', 'a') as fy:
		
		#with open('XXi_data.txt', 'a') as fxi:
		#print py
		break
  #print (pxi)
  #print (py)
  n=0
  for y in range (h/2, h):
  #pixeles contorno derecho
    for x in range (w-1, 0, -1):
      pix1= mask [y, x]
      if (pix1==255):
        pxd = np.append(pxd, x)
	  			
	#with open('XXd_data.txt', 'a') as fxd:
	  #print >>fxd, x
	n=n+1
        break
  #print (pxd)			
  message = datax()
  
  p1=3
  p2=n*0.15
  p3=n*0.40
  p4=n*0.8

  message.y0=py[int (p1)]
  message.y1=py[int (p2)]
  message.y2=py[int (p3)]
  message.y3=py[int (p4)]
  
  message.x0l=pxi[int (p1)]
  message.x1l=pxi[int (p2)]
  message.x2l=pxi[int (p3)]
  message.x3l=pxi[int (p4)]
  
  message.x0r=pxd[int (p1)]
  message.x1r=pxd[int (p2)]
  message.x2r=pxd[int (p3)]
  message.x3r=pxd[int (p4)]

  message.x0=(pxi[int (p1)]+pxd[int (p1)])/2
  message.x1=(pxi[int (p2)]+pxd[int (p2)])/2
  message.x2=(pxi[int (p3)]+pxd[int (p3)])/2
  message.x3=(pxi[int (p4)]+pxd[int (p4)])/2
  
  pub.publish(message)

def listener():
    global pub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    pub = rospy.Publisher('messagePoints', datax, queue_size=1)

    rospy.Subscriber("image_topic", Image, image_callback)

    #Para comprobar que el nodo funciona con el ejemplo de los tutoriales de ROS
    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
