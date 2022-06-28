#! /usr/bin/env python3

# Import all the nessecary packages
import rospy
from geometry_msgs.msg import Twist
import cv2
import imutils
import numpy as np


cap = cv2.VideoCapture(1)
class ImageProcessing:
	def __init__(self,lower=(29, 86, 6), upper=(64, 255, 255)):
		self.lower = lower  # The lower limit of the colour range in the form of RGB Values
		self.upper = upper  # The Upper limit of the colour range in the form of RGB Values
		self.radius = None  # The radius of the Circle drawn around the object
		self.center = None  # The center of the circle drawn around the object
		self.velocity_msg = Twist()
		self.pub = rospy.Publisher('/fix_error', Twist, queue_size=10) 

		#self.process_image()
		#self.publish()


	def process_image(self,frame):
		self.frame=frame
		
		blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)

		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		mask = cv2.inRange(hsv, self.lower, self.upper)

		mask = cv2.erode(mask, None, iterations=2)

		mask = cv2.dilate(mask, None, iterations=2)

		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)

		cnts = imutils.grab_contours(cnts)
		
		if len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			self.x, self.y , self.w , self.h = cv2.boundingRect(c)
			cv2.rectangle(frame , (self.x , self.y) , (self.x + self.w , self.y + self.h) , (36,255,12) , 2)

		self.frame = self.publish()

		return self.frame
		
			
	def publish(self) :
		
		#print(self.frame.shape)
		xcenter = self.frame.shape[1]/2
		ycenter = self.frame.shape[0]/2

		obj_xcenter = self.x + self.w/2
		obj_ycenter = self.y + self.h/2

		cv2.line(self.frame , (int(xcenter) , 0) , (int(xcenter) , int(2*ycenter)) , (255,0,0) , 5)
		cv2.line(self.frame , (0 , int(ycenter)) , (int(xcenter*2) , int(ycenter)) , (255,0,0) , 5)
		cv2.circle(self.frame , (int(obj_xcenter) , int(obj_ycenter)) , 5 , (0,0,255) , -1)		

		if xcenter > obj_xcenter + self.w/2 :
			self.velocity_msg.linear.x = -1
		elif xcenter < obj_xcenter - self.w/2 :
			self.velocity_msg.linear.x = 1
		else:
			self.velocity_msg.linear.x = 0

		if ycenter > obj_ycenter + self.h/2 :
			self.velocity_msg.linear.y = 1
		elif ycenter < obj_ycenter - self.h/2 :
			self.velocity_msg.linear.y = -1
		else:
			self.velocity_msg.linear.y = 0

		self.pub.publish(self.velocity_msg)
		

		return self.frame





		 
if __name__=="__main__":

	rospy.init_node("detector",anonymous=True)
	sc = ImageProcessing((20, 100 , 100 ),( 30 , 255, 255))

	while (cap.isOpened() ):
		ret , frame = cap.read()

		if ret :
			
			result=sc.process_image(frame)
			cv2.imshow("Frame" ,result)
			cv2.waitKey(1)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			
            	

	
	#cv2.imshow("Frame",result[0])
	#cv2.imshow("Mask",result[1])
	#print('center ',result[2])
	#print('radius ',result[3])
	#print(result[0].shape)

	#cv2.waitKey(0)


cap.release()
cv2.destroyAllWindows()
