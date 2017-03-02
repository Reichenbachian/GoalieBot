#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct
import math
import numpy as np
from sklearn.linear_model import LinearRegression 

class GoalieBot:
	limiter = 0
	maxCount = 2
	skipPixels = 5
	recentDepthImage = None
	CAMERA_ANGLE = 57*math.pi/180 #the camera horizontal angle is 57 degrees
	NUM_PIXLES = 640
	cameraAngle = 40
	ball = None
	ballPositions = []
	positions = []
	shouldContinue = True
	class Ball:
		SPEC = 20
		LOW_SPEC = 80
		XImagePos = 0.0
		YImagePos = 0.0
		distance = 0.0

	def __init__(self):
		self.bridge = CvBridge()
		# look at the rgb image		
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.ballTrack, queue_size=5)
		
		# give commands to the wheels		
		self.movement = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=5)

		# look at the depth image 
		self.image_depth = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_test, queue_size=5)

	def depth_test(self,data):
		# storing the depth image
		data.encoding = "mono16"
		self.recentDepthImage = self.bridge.imgmsg_to_cv2(data, "passthrough")
		# open the depth image
		if self.ball == None:
			return
		cv2.circle(self.recentDepthImage, (int(self.ball.YImagePos), int(self.ball.XImagePos)), 25, 100000, thickness=10)
		#cv2.imshow("", self.recentDepthImage)
		#cv2.waitKey(3)

	def getPosInDepthImage(self, x, y):
		assert self.recentDepthImage != None
		image = self.recentDepthImage
		x = int(x*image.shape[0]+0.5)
		y = int(y*image.shape[1]+0.5)
		rect_size = 10
		min_depth = image[0][0]
		for i in range(x-rect_size, x+rect_size):
			if i > 0 and i < image.shape[0]:
				for j in range(y-rect_size, y+rect_size):
					if j > 0 and j < image.shape[1]:
						if image[x][y] != 0:
							min_depth = min(image[x][y], min_depth)
		return min_depth

	def ballTrack(self,data):
		if self.shouldContinue == False:
			return
		if self.recentDepthImage == None:
			return
		if self.limiter < self.maxCount:
			self.limiter+=1
			return
		self.limiter = 0
		try:
			img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		self.findBall(img)
		twister = Twist()
		twister.angular.z = 0
		if self.ball != None:
			#if ball.XImagePos < img.shape[0]/2:
			#	twister.angular.z = -1
			#else:
			#	twister.angular.z = 1
			cv2.ellipse(img, (self.ball.YImagePos, self.ball.XImagePos), (20, 20), 0, 0, 180, (255,0,0), 2)
			#cv2.imshow("Image window", img)
			ballCoord = self.polarToCartesian(self.ball.distance, self.ball.YImagePos, img.shape[1])
			self.positions.append(ballCoord)
			if self.ball.distance < .8:
				self.interceptBall()
			print("Found ball! Distance is", ballCoord)
			#cv2.waitKey(3)
		else:
			print("Can't find ball!")
		self.movement.publish(twister)


	def interceptBall(self):
		linreg = LinearRegression()
		X = [x[0] for x in self.positions]
		y = [x[1] for x in self.positions]
		linreg.fit([[Y] for Y in y],[[x] for x in X] )
		#Remove Outliers

		errors = []
		for i in range(len(X)):
			predicted = linreg.predict(X[i])
			error = abs(predicted - y[i])
			errors.append(errors)
		for i in range(int(len(X)/20)):
			max_error = errors.index(max(errors))
			X.pop(max_error)
			y.pop(max_error)
		linreg.fit([[Y] for Y in y], [[x] for x in X])


		XBallPredicted = float(linreg.predict(0)[0][0])
	

		# Move at a rate of 10 HZ
		r = rospy.Rate(10);
		print("Predicted: ", XBallPredicted)
		speed = .5# in m/s: max of .65
		r = rospy.Rate(20);
		for i in range(int(XBallPredicted*20*(1/speed))):
			print(float(i),XBallPredicted*20*(1/speed), "%")
			print("test")
			move_cmd = Twist()
			# turn at angle radians/s
			move_cmd.linear.x = .5
			# publish the velocity
			self.movement.publish(move_cmd)
			# publish again after 10 HZ (0.1 s)
			r.sleep()
		move_cmd.angular.z = 0
		self.movement.publish(move_cmd)
		self.shouldContinue = False


	def findBall(self, image):
		# find the ball in the rgb image, set ball's x-pixel, y-pixel, distance from camera
		width, height, channel = image.shape
		minDepth = 100000000
		sumXBalls = 0
		sumYBalls = 0
		countX = 0
		countY = 0
		for yCoord in range(0, height, self.skipPixels):
			for xCoord in range(0, width, self.skipPixels):
				b, g, r = image[xCoord,yCoord]
				#Please note imread is in BGR!
				if g > b + self.Ball.SPEC and g > r + self.Ball.SPEC and g > self.Ball.LOW_SPEC:
					sumXBalls += xCoord
					sumYBalls += yCoord
					countX += 1
					countY += 1
		if countX == 0:
			return None
		if self.ball == None:
			self.ball = self.Ball()
		self.ball.XImagePos = sumXBalls/countX
		self.ball.YImagePos = sumYBalls/countY
		k=0.1
		cameraHeight = .2
		pixelHeight = self.ball.YImagePos
		y = self.ball.XImagePos - width/2
		phi = math.atan(2*y*math.tan(57/2*math.pi/180)/width);
		k=3.61697483711
		d = math.tan(math.pi/2 - phi - self.cameraAngle*math.pi/180)*cameraHeight*k
		self.ball.distance = d		



	def polarToCartesian(self, depth, xpos, width_in_pix):
		r = depth
		#theta = float(-1)*(xpos-self.NUM_PIXLES/2)/(self.NUM_PIXLES/2)*self.CAMERA_ANGLE/2 + 90
		start_angle = 90 - 57/2
		theta = float(width_in_pix - xpos)/float(width_in_pix) * 57 + start_angle
		#x = self.ball.YImagePos - width_in_pix/2
		#phi = math.atan(2*x*math.tan(57/2*math.pi/180)/	width_in_pix)
		#print(width_in_pix, self.ball.YImagePos)
		#theta = math.tan(math.pi/2 - phi)
		y_coord = r*math.sin(theta*math.pi/180)
		x_coord = r*math.cos(theta*math.pi/180)
		return (x_coord, y_coord)





def main():
	rospy.init_node('goalieBot', anonymous=True)
	#ic = image_converter()
	goalieBot = GoalieBot()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


main()
