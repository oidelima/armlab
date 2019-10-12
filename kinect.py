from PyQt4.QtGui import QImage
from apriltag import apriltag
import numpy as np
import cv2
import freenect
import copy
from numpy import *
import blocks
import time
import math

class Kinect():
	def __init__(self):
		self.currentVideoFrame = np.array([])
		self.currentDepthFrame = np.array([])
		self.previousDepthFrame = np.array([])
		self.currentHiResFrame = np.array([])
		freenect.sync_set_autoexposure(False)
		freenect.sync_set_whitebalance(False)
		if(freenect.sync_get_depth() == None):
			self.kinectConnected = False
		else:
			self.kinectConnected = True

		# mouse clicks & calibration variables
		self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
		self.workcamera_affine = np.float32([[1,0,0],[0,1,0]])
		self.kinectCalibrated = False
		self.last_click = np.array([0,0])
		self.new_click = False
		self.rgb_click_points = np.zeros((5,2),int)
		self.depth_click_points = np.zeros((5,2),int)
		self.rotation_matrix = np.zeros((3,3),float)
		self.translation_matrix = np.zeros((3,1),float)
		self.intrinsic_matrix = np.zeros((3,3),float)
		self.intrinsic_matrix_inverse = np.zeros((3,3),float)
		self.corners_rgb = []
		self.corners_depth = []

		""" Extra arrays for colormaping the depth image"""
		self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthH = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthS = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthV = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthCM = np.array([])

		# Block Depth Isolation
		self.DepthHSVThreshold = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthCMThreshold = np.array([])
		self.BlockMask = np.zeros((480,640,1)).astype(np.uint8)
		self.roi = np.zeros((480,640,3)).astype(np.uint8)

		""" block info """
		self.block_contours = []
		self.block_coordinates = []
		self.block_coordinates_raw = []
		self.block_orientations = []
		self.box_corners = []
		self.blocks = {}

		"""Time variables"""
		self.currentTime = 0.0
		self.previousTime = 0.0

	def captureVideoFrame(self):
		""" Capture frame from Kinect, format is 24bit RGB """
		if(self.kinectConnected):
			self.currentVideoFrame = freenect.sync_get_video()[0]
		else:
			self.loadVideoFrame()
		self.processVideoFrame()

	def processVideoFrame(self):
		self.roiContours(self.currentVideoFrame, 600)

	def captureDepthFrame(self):
		if(self.kinectConnected):
			if(self.kinectCalibrated):
				self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
			else:
				self.currentDepthFrame = freenect.sync_get_depth()[0]
		else:
			self.loadDepthFrame()

	def loadVideoFrame(self):
		self.currentVideoFrame = cv2.cvtColor(v2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED), cv2.COLOR_BGR2RGB)

	def loadDepthFrame(self):
		self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

	def convertFrame(self):
		try:
			img = QImage(self.currentVideoFrame, self.currentVideoFrame.shape[1], self.currentVideoFrame.shape[0], QImage.Format_RGB888)
			return img
		except:
			return None

	def convertDepthFrame(self):
		try:
			self.DepthHSV[...,0] = self.currentDepthFrame
			self.DepthHSV[...,1] = 0x9F
			self.DepthHSV[...,2] = 0xFF
			self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
			self.roiContours(self.DepthCM, 400)
			img = QImage(self.DepthCM, self.DepthCM.shape[1], self.DepthCM.shape[0], QImage.Format_RGB888)
			return img
		except:
			return None

	def convertHSVFrame(self):
		try:
			image = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2HSV)
			self.DepthH[...,0] = self.masking(image[...,0])
			self.DepthH[...,1] = self.masking(image[...,0])
			self.DepthH[...,2] = self.masking(image[...,0])
			self.DepthH = cv2.cvtColor(self.DepthH,cv2.COLOR_HSV2RGB)
			img_h = QImage(self.DepthH, self.DepthH.shape[1], self.DepthH.shape[0], QImage.Format_RGB888)

			self.DepthS[...,0] = self.masking(image[...,1])
			self.DepthS[...,1] = self.masking(image[...,1])
			self.DepthS[...,2] = self.masking(image[...,1])
			self.DepthS = cv2.cvtColor(self.DepthS,cv2.COLOR_HSV2RGB)
			img_s = QImage(self.DepthS, self.DepthS.shape[1], self.DepthS.shape[0], QImage.Format_RGB888)

			self.DepthV[...,0] = self.masking(image[...,2])
			self.DepthV[...,1] = self.masking(image[...,2])
			self.DepthV[...,2] = self.masking(image[...,2])
			self.DepthV = cv2.cvtColor(self.DepthV,cv2.COLOR_HSV2RGB)
			img_v = QImage(self.DepthV, self.DepthV.shape[1], self.DepthV.shape[0], QImage.Format_RGB888)
			return img_h,img_s,img_v
		except:
			return None

	def convertBlockDepthFrame(self):
		try:
			self.DepthHSVThreshold[...,0] = self.currentDepthFrame
			self.BlockMask = np.zeros((480,640,1)).astype(np.uint8)
			self.DepthHSVThreshold[self.currentDepthFrame > 710 ,0] = 0
			self.DepthHSVThreshold[self.currentDepthFrame < 650 ,0] = 0
			self.BlockMask[self.DepthHSVThreshold[...,0] != 0] = 1

			self.DepthHSVThreshold[...,1] = 255
			self.DepthHSVThreshold[...,2] = 255

			self.DepthHSVThreshold[self.currentDepthFrame > 710 ,1] = 0
			self.DepthHSVThreshold[self.currentDepthFrame < 650 ,1] = 0
			self.DepthHSVThreshold[self.currentDepthFrame > 710 ,2] = 0
			self.DepthHSVThreshold[self.currentDepthFrame < 650 ,2] = 0

			self.DepthCMThreshold = cv2.cvtColor(self.DepthHSVThreshold,cv2.COLOR_HSV2RGB)
			self.detectBlocksInDepthImage()
			self.roiContours(self.DepthCMThreshold, 400)

			self.rectangleBlocks()
			self.blockDetector()

			img = QImage(self.DepthCMThreshold, self.DepthCMThreshold.shape[1], self.DepthCMThreshold.shape[0], QImage.Format_RGB888)
			return img
		except:
			return None

	def depthframeSampler(self, dt):
		self.currentTime = time.clock()
		if abs(self.currentTime - self.previousTime) > dt:
			self.previousTime = self.currentTime
			return True
		else:
			return False

	def masking(self, image):
		#return cv2.bitwise_and(self.auto_canny(image,0.33), self.auto_canny(image,.33), mask = self.BlockMask)
		return image
		#return self.auto_canny(image,.7)

	def blockDetector(self):
		self.block_coordinates_raw = []
		self.block_coordinates = []
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontScale = .5
		fontColor = (255,255,255)
		lineType  = 1
		for x in self.block_contours:
			m = cv2.moments(x)
			if abs(m["m00"]) >0:
				cx = int(m["m10"]/m["m00"])
				cy = int(m["m01"]/m["m00"])
				cz = self.currentDepthFrame[cy][cx]
				self.block_coordinates_raw.append([cx,cy,cz])
				camera_coordinates = np.array([[cx],[cy],[1]]).astype(np.float32)
				xy_world = np.matmul(self.workcamera_affine,camera_coordinates)
				z_w = .1236*np.tan(cz/2842.5 + 1.1863)
				cx_w = xy_world[0]
				cy_w = xy_world[1]
				cz_w = .94 - z_w
				self.block_coordinates.append([cx_w,cy_w,cz_w])
				location = str(np.round(cx_w[0],3)) +", "+ str(np.round(cy_w[0],3)) +", "+ str(np.round(cz_w,3))
				cv2.putText(self.DepthCMThreshold, location, (cy,cx), font, fontScale, fontColor, lineType)

	def centroidBlock(self, contour):
			m = cv2.moments(contour)
			if abs(m["m00"]) > 0:
				cx = int(m["m10"]/m["m00"])
				cy = int(m["m01"]/m["m00"])
				cz = self.currentDepthFrame[cy][cx]
				return cx,cy
			return 0,0

	def cornersBlock(self,contour):
		#perimeter = cv2.arcLength(contour, True)
		#approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
		rect = cv2.minAreaRect(contour)
		corners = cv2.boxPoints(rect)
		corners = np.int0(box)
		return corners

	def orientationBlock(self, corners):
		try:
			x1 = corners[0][0]
			y1 = corners[0][1]
			x2 = corners[1][0]
			y2 = corners[1][1]
			x3 = corners[2][0]
			y3 = corners[2][1]
			x4 = corners[3][0]
			y4 = corners[3][1]
			slope1 = math.atan2((y1 - y4),(x1 - x4))
			slope2 = math.atan2((y2 - y3),(x2 - x3))
			return slope1,slope2
		except:
			return None

	def meanPose(self, contours):
		try:
			if len(contours) !=0:
				x = []
				y = []
				z = []
				slope1 = []
				slope2 = []
				for contour in contours:
					area = cv2.contourArea(contour)
					if  area > 200:
						xy = self.centroidBlock(contour)
						x.append(xy[0])
						y.append(xy[1])
						rect = cv2.minAreaRect(contour)
						corners = cv2.boxPoints(rect)
						slopes = self.orientationBlock(corners)
						slope1.append(slopes[0])
						slope2.append(slopes[1])
				if len(x) !=0:
					x = sum(x)/len(x)
					y = sum(y)/len(y)
					slope1 = sum(slope1)/len(slope1)
					slope2 = sum(slope2)/len(slope2)
				return x,y,slope1,slope2
		except:
			return None

	def blockPose(self,color,block):
		try:
			contour = cv2.findContours(block, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			coordinates = self.meanPose(contour[1])
			self.blocks[color] = {}
			self.blocks[color]["centroid"] = (coordinates[0],coordinates[1])
			self.blocks[color]["orientation"] = (coordinates[2], coordinates[3])
		except:
			return None

	def detectBlocksInDepthImage(self):
		image = self.currentVideoFrame.astype(np.float32)
		image = cv2.GaussianBlur(image,(3,3),0)
		#cv2.imwrite("redimage.jpg",cv2.bitwise_and(self.currentVideoFrame[...,0],self.currentVideoFrame[...,0], mask = self.BlockMask))
		#cv2.imwrite("greenimage.jpg",cv2.bitwise_and(self.currentVideoFrame[...,1],self.currentVideoFrame[...,1], mask = self.BlockMask))
		#cv2.imwrite("blueimage.jpg",cv2.bitwise_and(self.currentVideoFrame[...,2],self.currentVideoFrame[...,2], mask = self.BlockMask))
		hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		hue_image = cv2.bitwise_and(hsv_image[...,0], hsv_image[...,0], mask = self.BlockMask)
		saturation_image = cv2.bitwise_and(hsv_image[...,1], hsv_image[...,1], mask = self.BlockMask)
		value_image = cv2.bitwise_and(hsv_image[...,2], hsv_image[...,2], mask = self.BlockMask)
		ret, value_threshold = cv2.threshold(value_image.astype(np.uint8),0,10, cv2.THRESH_BINARY)
		#cv2.imwrite("valuethresh.jpg",value_threshold)
		if self.kinectCalibrated:
			try:
				green, blue, yellow, red, orange, purple, pink, black = self.colorbuckets()

				self.blockPose("green",green)
				self.blockPose("blue",blue)
				self.blockPose("yellow",yellow)
				self.blockPose("red",red)
				self.blockPose("pink",pink)
				self.blockPose("purple",purple)
				self.blockPose("black",black)
				self.blockPose("orange",orange)
			except:
				return None

		contours = cv2.findContours(value_threshold, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		self.block_contours = []
		if len(contours) != 0:
			self.block_contours = contours[1]
		self.isolateBlockContours()

	def roiContours(self, image, threshold):
		for contour in self.block_contours:
				if cv2.contourArea(contour) > threshold:
					cv2.drawContours(image,contour,-1,(0,0,0),1)

	def rectangleBlocks(self):
		try:
			self.block_orientations = []
			if len(self.block_contours) != 0 :
				for contour in self.block_contours:
					perimeter = cv2.arcLength(contour, True)
					approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
					rect = cv2.minAreaRect(contour)
					box = cv2.boxPoints(rect)
					box = np.int0(box)
					if cv2.contourArea(contour) > 400:
						cv2.drawContours(self.currentVideoFrame,[box],0,(0,0,255),2)
						self.box_corners.append(box)
			for corners in self.box_corners:
				x1 = corners[0][0]
				y1 = corners[0][1]
				x2 = corners[1][0]
				y2 = corners[1][1]
				x3 = corners[2][0]
				y3 = corners[2][1]
				x4 = corners[3][0]
				y4 = corners[3][1]
				slope1 = math.atan2((y1 - y2),(x1 - x2))
				slope2 = math.atan2((y3 - y4),(x3 - x4))
				#print("corners: ",(x1,y1,x2,y2,x3,y3,x4,y4))
				#print("Slopes: ", slope1,slope2)
		except:
			return None

	def isolateBlockContours(self):
		contours = []
		for contour in self.block_contours:
			if contour.shape[0] > 25 and contour.shape[0] < 200:
				contours.append(contour)
		self.block_contours = contours

	def colorbuckets(self):

		image = self.currentVideoFrame
		image = cv2.GaussianBlur(image,(3,3),0)
		image = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
		value = image[...,2]
		hue = image[...,0]
		hue_green = cv2.inRange(hue, 50, 70)
		hue_blue = cv2.inRange(hue, 110, 130)
		hue_yellow = cv2.inRange(hue, 20, 40)
		hue_red = cv2. inRange(hue, 170, 179)
		hue_orange = cv2.inRange(hue, 0, 15)
		hue_pink = cv2.inRange(hue, 160, 170)
		hue_purple = cv2.inRange(hue, 130,145)
		value_black = cv2.inRange(value, 0 , 110)

		self.roi = cv2.bitwise_and(hue, hue, mask = self.BlockMask)
		red = cv2.bitwise_and(hue_red, hue_red, mask = self.BlockMask)
		green = cv2.bitwise_and(hue_green, hue_green, mask = self.BlockMask)
		blue = cv2.bitwise_and(hue_blue, hue_blue, mask = self.BlockMask)
		yellow = cv2.bitwise_and(hue_yellow, hue_yellow, mask = self.BlockMask)
		pink = cv2.bitwise_and(hue_pink, hue_pink, mask = self.BlockMask)
		purple = cv2.bitwise_and(hue_purple, hue_purple, mask = self.BlockMask)
		orange = cv2.bitwise_and(hue_orange, hue_orange, mask = self.BlockMask)
		black = cv2.bitwise_and(value_black,value_black, mask = self.BlockMask)

		return green, blue, yellow, red, orange, purple, pink, black

	def getAffineTransform(self, coord1, coord2):
		"""
		Given 2 sets of corresponding coordinates,
		find the affine matrix transform between them.

		TODO: Rewrite this function to take in an arbitrary number of coordinates and
		find the transform without using cv2 functions
		"""
		#coord1 are RGB coordinates and coord2 are Depth Camera coordinates
		pts1 = coord2[0:5].astype(np.float32)
		pts2 = coord1[0:5].astype(np.float32)
		#Defining Affine Matrix components in vector_x, RGB coordinates in matrix A, and Depth Coordinates in vector_b
		vector_x = np.ones((8,1))

		matrix_A = np.array([[ pts1[0][0], pts1[0][1],   1, 	   	 0, 		 0, 0],
							 [ 		    0, 		  	0, 	 0, pts1[0][0], pts1[0][1], 1],
							 [ pts1[1][0], pts1[1][1],   1, 	   	 0, 		 0, 0],
							 [ 		    0, 		  	0, 	 0, pts1[1][0], pts1[1][1], 1],
							 [ pts1[2][0], pts1[2][1],   1, 	   	 0, 		 0, 0],
							 [ 		    0, 		  	0,   0, pts1[2][0], pts1[2][1], 1],
							 [ pts1[3][0], pts1[3][1], 	 1, 	   	 0, 		 0, 0],
							 [ 		    0, 		  	0,   0, pts1[3][0], pts1[3][1], 1]]).astype(np.float32)

		vector_b = np.array([[pts2[0][0]],
							 [pts2[0][1]],
							 [pts2[1][0]],
							 [pts2[1][1]],
							 [pts2[2][0]],
							 [pts2[2][1]],
							 [pts2[3][0]],
							 [pts2[3][1]]]).astype(np.float32)

		#Calculating Pseudo-inverse of Matrix A of RGB coordinates
		matrix_A_transpose = np.transpose(matrix_A)
		matrix_A_inv = np.matmul(np.linalg.inv(np.matmul(matrix_A_transpose,matrix_A)),matrix_A_transpose)

		#Calculating affine matrix components
		vector_x = np.matmul(matrix_A_inv,vector_b)

		affineMatrixTransformation = np.array([[vector_x[0], vector_x[1], vector_x[2]],
											   [vector_x[3], vector_x[4], vector_x[5]],
											   [	  	  0,	       0, 	   		1]])

		self.depth2rgb_affine = vector_x.reshape(2,3)
		return self.depth2rgb_affine

	def registerDepthFrame(self, frame):
		transformedDepthFrame = np.zeros((len(frame),len(frame[0])))
		n_rows = len(frame)
		n_columns = len(frame[0])

		for x in range(n_columns):
			for y in range(n_rows):
				vector_depth = np.array([x,y,1])
				x_t = int(np.matmul(self.depth2rgb_affine[0],vector_depth))
				y_t = int(np.matmul(self.depth2rgb_affine[1],vector_depth))
				if x_t < n_columns and x_t >= 0 and y_t < n_rows and y_t >= 0:
					transformedDepthFrame[y_t][x_t] = frame[y][x]
		return transformedDepthFrame

	def loadCameraCalibration(self):
		self.intrinsic_matrix = np.array([[492.43967452, 0.0, 305.98100007],[0.0, 492.06649804, 286.20904428],[0.0, 0.0, 1.0]])
		self.intrinsic_matrix_inverse = np.linalg.inv(self.intrinsic_matrix)
		return self.intrinsic_matrix

	def affineworkspace(self, coordinates):
		d = 0.305
		pts2 = np.array([[-d, d],[-d, -d],[d, -d],[d, d],[0.0, 0.0]]).astype(np.float32)
		pts1 = coordinates[0:5].astype(np.float32)
		"""coordinates = self.apriltagtransformation()
		pts1 = [coordinates[1], coordinates[0], coordinates[2], coordinates[3]]"""

		#Defining Affine Matrix components in vector_x, RGB coordinates in matrix A, and Depth Coordinates in vector_b
		vector_x = np.ones((10,1))

		matrix_A = np.array([[ pts1[0][0], pts1[0][1], 1, 		   0, 		   0, 0],
							 [			0, 			0, 0, pts1[0][0], pts1[0][1], 1],
							 [ pts1[1][0], pts1[1][1], 1, 		   0, 		   0, 0],
							 [			0, 			0, 0, pts1[1][0], pts1[1][1], 1],
							 [ pts1[2][0], pts1[2][1], 1, 		   0, 		   0, 0],
							 [			0, 			0, 0, pts1[2][0], pts1[2][1], 1],
							 [ pts1[3][0], pts1[3][1], 1, 		   0, 		   0, 0],
							 [			0, 			0, 0, pts1[3][0], pts1[3][1], 1]]).astype(np.float32)

		vector_b = np.array([[pts2[0][0]],
							 [pts2[0][1]],
							 [pts2[1][0]],
							 [pts2[1][1]],
							 [pts2[2][0]],
							 [pts2[2][1]],
							 [pts2[3][0]],
							 [pts2[3][1]]]).astype(np.float32)

		# Calculating Pseudo-inverse of Matrix A of RGB coordinates
		matrix_A_transpose = np.transpose(matrix_A)

		matrix_A_inv = np.matmul(np.linalg.inv(np.matmul(matrix_A_transpose,matrix_A)),matrix_A_transpose)

		# Calculating affine matrix components
		vector_x = np.matmul(matrix_A_inv,vector_b)

		affineMatrixTransformation = np.array([[vector_x[0], vector_x[1], vector_x[2]],
											   [vector_x[3], vector_x[4], vector_x[5]],
											   [	      0,	       0, 	       1]])
		self.workcamera_affine = vector_x.reshape(2,3)
		return self.workcamera_affine

	def workspaceTransform(self, object_points, image_points):
		camera_intrinsic_matrix = self.loadCameraCalibration()
		dist_coeffs = np.array([[0.4799379,-0.94025256,  0.00780928,  0.00295796, -0.06855816]]).astype(np.float32)
		(success, rot_vec, trans_vec) = cv2.solvePnP(object_points, image_points, camera_intrinsic_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
		self.rotation_matrix = np.array([[0.0, -rot_vec[2], rot_vec[1]],
										[rot_vec[2], 0.0, -rot_vec[0]],
										[-rot_vec[1], rot_vec[0], 0.0]])
		self.translation_matrix = trans_vec
		return trans_vec

	def auto_canny(self,image, sigma):
		#image = cv2.GaussianBlur(image, (5,5),0)
		v = np.median(image)
		lower = int(max(100, (1.0 - sigma) * v))
		upper = int(min(200, (1.0 + sigma) * v))
		edges = cv2.Canny(image, lower, upper)
		cv2.imwrite("edges.jpg",edges)
		return edges

	def rgbedges(self):
		image = freenect.sync_get_video_with_res()[0]
		image = cv2.GaussianBlur(image, (5,5),2)
		image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		ret,image  = cv2.threshold(image,220,255,cv2.THRESH_BINARY)
		#cv2.imwrite("gray_rgb.jpg",image)
		image = cv2.Canny(image, 200, 250)
		#cv2.imwrite("gray_rgb_edge.jpg",image)
		return image

	def depthedges(self):
		image = self.currentDepthFrame
		image = cv2.GaussianBlur(image, (5,5),2)
		image  = cv2.inRange(image, 717, 725)
		image = image
		#cv2.imwrite("gray_depth.jpg",image)
		image = cv2.Canny(image, 100, 250)
		#cv2.imwrite("gray_depth_edge.jpg",image)
		return image

	def cornerscal(self, image):
		try:
			#print("trying to find corners")
			corners = cv2.goodFeaturesToTrack(image.astype(np.float32), 4,.1,100)
			return corners
		except:
			#print("could not find four corners try again")
			return None


	def houghlines(self):
		#print("finding lines")
		depth = self.currentDepthFrame*0
		rgbedges = self.rgbedges()
		depthedges = self.depthedges()

		rgblines = []
		counter  =  20
		while(len(rgblines) != 4 and counter != 0):
			#print("Looking for color lines")
			rgblines = cv2.HoughLines( rgbedges, 1, np.pi/100, 100)

		#print("lines found rgb")
		depthlines = []
		counter = 20
		while(len(depthlines) != 4 and counter != 0):
			#print("Looking for depth lines")
			depthlines = cv2.HoughLines( depthedges, 1, np.pi/90, 90 - (20 - counter))
			counter = counter - 1

		#print("lines found depth")


		depthcoordinates = []
		for i in range(len(depthlines)):
			theta = depthlines[i][0][1]
			rho = depthlines[i][0][0]
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))
			#print(x0,y0,x1,y1,x2,y2)
			cv2.line(depth,(x1,y1),(x2,y2),(255,255,255),1)
		#cv2.imwrite("g_depth.jpg",depth)
		self.corners_depth = self.cornerscal(depth)
		#print("Depth corners",self.corners_depth)
		depth = depth*0

		for i in range(len(rgblines)):
			theta = rgblines[i][0][1]
			rho = rgblines[i][0][0]
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 1000*(-b))
			y1 = int(y0 + 1000*(a))
			x2 = int(x0 - 1000*(-b))
			y2 = int(y0 - 1000*(a))
			#print(x0,y0,x1,y1,x2,y2)
			cv2.line(depth,(x1,y1),(x2,y2),(255,255,255),1)


		#cv2.imwrite("g_rgb.jpg",depth)
		self.corners_rgb = self.cornerscal(depth)
		#print("RGB corners",self.corners_rgb)

		depth_corner_lb = self.corners_depth[1]
		depth_corner_lt = self.corners_depth[2]
		depth_corner_rt = self.corners_depth[3]
		depth_corner_rb = self.corners_depth[0]


		rgb_corner_lb = self.corners_rgb[2]
		rgb_corner_lt = self.corners_rgb[3]
		rgb_corner_rt = self.corners_rgb[1]
		rgb_corner_rb = self.corners_rgb[0]

		self.corners_depth = [depth_corner_lb, depth_corner_lt, depth_corner_rt, depth_corner_rb]
		self.corners_rgb = [rgb_corner_lb, rgb_corner_lt, rgb_corner_rt, rgb_corner_rb]


""" def apriltagtransformation(self):
		detector = apriltag("tagStandard41h12", threads=4, decimate=2.0)
		d = .060
		object_points = np.array([[-d,-d,0.0],
								  [d, -d, 0.0],
								  [d, d, 0.0],
								  [-d, d, 0.0]],dtype = "double")

		image = self.currentVideoFrame
		image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		detections = detector.detect(image)
		translation_vector = []
		for tag in detections:
			image_points = tag['lb-rb-rt-lt']
			translation = self.workspaceTransform(object_points, image_points)
			x_average = (image_points[0][0] + image_points[1][0] + image_points[2][0] + image_points[3][0])/4
			y_average = (image_points[0][1] + image_points[1][1] + image_points[2][1] + image_points[3][1])/4
			translation_vector.append([x_average,y_average])
		return translation_vector"""

