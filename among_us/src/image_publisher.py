#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import scipy
import scipy.misc

# importing OpenCV(cv2) module 
import cv2 
from time import sleep
  
# Save image in set directory 
# Read RGB image 
img = cv2.imread('among_us.png')  

rospy.init_node('VideoPublisher', anonymous=True)

ImagePub = rospy.Publisher('among_us_character', Image, queue_size=10)

among_us_matrix = scipy.misc.imread('among_us.png')


print('scipy read:')
print(among_us_matrix[202][202])
print('cv2 read:')
print(img[0][0])

cv2.imshow('image', img) 

cv2.waitKey(0)


sleep(100000)
image = Image()


#image.height = 
#image.width = 
#image.data = 


#ImagePub.pub()

