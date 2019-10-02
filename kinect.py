import cv2
#from apriltag import apriltag
import numpy as np
from PyQt4.QtGui import QImage
import freenect
import copy
from numpy import *

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
		self.depthframecounter = 0

		""" Extra arrays for colormaping the depth image"""
		self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthH = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthS = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthV = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthCM=np.array([])

		# Block Depth Isolation
		self.DepthHSVThreshold = np.zeros((480,640,3)).astype(np.uint8)
		self.DepthCMThreshold = np.array([])
		self.BlockMask = np.zeros((480,640,1)).astype(np.uint8)
		self.roi = np.zeros((480,640,3)).astype(np.uint8)

		""" block info """
		self.block_contours = np.array([])
		self.block_coordinates = []
		self.block_coordinates_raw = []
		self.block_colors = {}
		self.block_orientations = []

	def captureVideoFrame(self):
		""" Capture frame from Kinect, format is 24bit RGB """
		if(self.kinectConnected):
			self.currentVideoFrame = freenect.sync_get_video()[0]
			#self.currentVideoFrame = freenect.sync_get_video_with_res()[0]
		else:
			self.loadVideoFrame()
		self.processVideoFrame()

	def processVideoFrame(self):
		cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),1)

	def captureDepthFrame(self):
		""" Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution. """
		if(self.kinectConnected):
			if(self.kinectCalibrated):
				if self.depthframecounter%2 ==0:
					self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
					self.previousDepthFrame = self.currentDepthFrame
					self.depthframecounter += 1
				else:
					self.currentDepthFrame = self.previousDepthFrame
					self.depthframecounter += 1
			else:
				self.currentDepthFrame = freenect.sync_get_depth()[0]
		else:
			self.loadDepthFrame()

	def loadVideoFrame(self):
		self.currentVideoFrame = cv2.cvtColor(
			cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED), cv2.COLOR_BGR2RGB)

	def loadDepthFrame(self):
		self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

	def convertFrame(self):
		""" Converts frame to format suitable for Qt  """
		try:
			img = QImage(self.currentVideoFrame, self.currentVideoFrame.shape[1], self.currentVideoFrame.shape[0], QImage.Format_RGB888)
			return img
		except:
			return None

	def convertDepthFrame(self):
		""" Converts frame to a colormaped format suitable for Qt Note: this cycles the spectrum over the lowest 8 bits	"""
		try:
			""" Convert Depth frame to rudimentary colormap """
			self.DepthHSV[...,0] = self.currentDepthFrame
			self.DepthHSV[...,1] = 0x9F
			self.DepthHSV[...,2] = 0xFF
			self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
			cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),1)

			img = QImage(self.DepthCM, self.DepthCM.shape[1], self.DepthCM.shape[0], QImage.Format_RGB888)
			return img
		except:
			return None

	def convertGrayFrame(self):
		""" Converts frame to format to Gray for Qt  """
		try:
			image = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2GRAY)
			img = QImage(image, image.shape[1], image.shape[0], QImage.Format_Indexed8)
			return img
		except:
			return None
   
	def convertHSVFrame(self):
		""" Converts frame to format to HSV for Qt  """
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
		""" Converts frame to a colormaped format suitable for Qt Note: this cycles the spectrum over the lowest 8 bits	"""
		try:
			#Determine ROI mask
			self.BlockMask = np.zeros((480,640,1)).astype(np.uint8)
			self.DepthHSVThreshold[...,0] = self.currentDepthFrame
			self.DepthHSVThreshold[self.currentDepthFrame > 710 ,0] = 0
			self.DepthHSVThreshold[self.currentDepthFrame < 650 ,0] = 0
			self.BlockMask[self.DepthHSVThreshold[...,0] != 0] = 1
			
			#Isolating Blocks
			self.DepthHSVThreshold[...,1] = 255
			self.DepthHSVThreshold[...,2] = 255
			
			self.DepthHSVThreshold[self.currentDepthFrame > 710 ,1] = 0
			self.DepthHSVThreshold[self.currentDepthFrame < 650 ,1] = 0
			
			self.DepthHSVThreshold[self.currentDepthFrame > 710 ,2] = 0
			self.DepthHSVThreshold[self.currentDepthFrame < 650 ,2] = 0

			self.DepthCMThreshold = cv2.cvtColor(self.DepthHSVThreshold,cv2.COLOR_HSV2RGB)
			
			self.detectBlocksInDepthImage()
			
			cv2.drawContours(self.DepthCMThreshold,self.block_contours,-1,(0,0,0),1)
			
			self.colorbuckets()
			img = QImage(self.DepthCMThreshold, self.DepthCMThreshold.shape[1], self.DepthCMThreshold.shape[0], QImage.Format_RGB888)
			return img
		except:
			return None
	
	def auto_canny(self,image, sigma):
		# compute the median of the single channel pixel intensities
		v = np.median(image)
 
		# apply automatic Canny edge detection using the computed median
		lower = int(max(0, (1.0 - sigma) * v))
		upper = int(min(255, (1.0 + sigma) * v))
		edged = cv2.Canny(image, lower, upper)
		return edged

	def masking(self, image):
		#return cv2.bitwise_and(self.auto_canny(image,0.33), self.auto_canny(image,.33), mask = self.BlockMask)
		return image
		#return self.auto_canny(image,.7)

	def blockDetector(self):
		""" TODO: Implement your block detector here. You will need to locate blocks in 3D space """
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
				cz_w = z_w
				self.block_coordinates.append([cx_w,cy_w,cz_w])
				location = str(np.round(cx_w[0],3)) +", "+ str(np.round(cy_w[0],3)) +", "+ str(np.round(cz_w,3)) 
				cv2.putText(self.DepthCMThreshold, location, (cy,cx), font, fontScale, fontColor, lineType)
		pass

	def accuracycheck(self):
			DepthCM = self.DepthCM
			DepthCM[self.currentDepthFrame > 723, 0] = 0
			DepthCM[self.currentDepthFrame < 717, 0] = 0
			DepthCM[...,1] = 0
			DepthCM[...,2] = 0
			edges = cv2.Canny(DepthCM, 100, 150)
			cv2.imwrite("Dedges.jpg", edges )
			lines = cv2.HoughLines(edges,1,np.pi/180,200)
			#print("\n d lines : ",lines)

			VideoCM = self.currentVideoFrame
			VideoCM[self.currentDepthFrame > 723, 0] = 0
			VideoCM[self.currentDepthFrame < 717, 0] = 0
			VideoCM[self.currentDepthFrame > 723, 1] = 0
			VideoCM[self.currentDepthFrame < 717, 1] = 0
			VideoCM[self.currentDepthFrame > 723, 2] = 0
			VideoCM[self.currentDepthFrame < 717, 2] = 0
			edges = cv2.Canny(VideoCM, 150, 200)
			cv2.imwrite("vedges.jpg", edges)
			lines = cv2.HoughLines(edges,1,np.pi/180,200)
			#print("\n v lines : ",lines)

	def roi(self):
		pass


	def colorbuckets(self):	
		try:
			self.block_orientations = []
			if len(self.block_contours) != 0 :
				for contour in self.block_contours:
					#print("Checking for contour")
					perimeter = cv2.arcLength(contour, True)
					approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
					#print("\ncontour is : ",approx)
					#x, y, w, h = cv2.boundingRect(approx)
					rect = cv2.minAreaRect(contour)	
					box = cv2.boxPoints(rect)
					box = np.int0(box)
					cv2.drawContours(self.currentVideoFrame,[box],0,(0,0,255),2)
					#print("box coordinates ",box)
					#self.block_orientations.append((x,y,w,h))
					#image = cv2.rectangle(self.currentVideoFrame, (x,y), (x+w, y+h), (0,0,0),2)
					#cv2.imwrite("contours.jpg",image)
			# 		print("finding orientation")
			# 		if len(contour)>4:
			# 			result = cv2.fitEllipse(contour)
			# 			print(result)
			# 			cv2.ellipse(self.currentVideoFrame,result,(0,0,0))
			# 			self.block_orientations.append(result[-1])
			# print("\n\nAngle : ", self.block_orientations)
			# print("In color bucket")
			# height_mask = self.currentDepthFrame
			# height_mask[height_mask > 725] = 0
			# height_mask[height_mask < 715] = 0
			# image = self.currentVideoFrame
			# image = cv2.GaussianBlur(image,(3,3),0)
			# image = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
			# cv2.imwrite("hsv.jpg",image)
			# value = image[...,2]
			# ret, thresh = cv2.threshold(value,210,255,cv2.THRESH_BINARY)
			# height_mask = height_mask.astype(np.uint8)

			# self.roi = cv2.bitwise_and(height_mask, thresh.astype(np.uint8))
			# ret, self.roi = cv2.threshold(self.roi,127,255,cv2.THRESH_BINARY)
			# cv2.imwrite("roi.jpg",self.roi)
			# image = cv2.GaussianBlur(self.roi,(7,7),0)
			# edges = self.auto_canny(self.roi,.15)
			# cv2.imwrite("edges.jpg",edges)
		
			# print("In color bucket")
			# image = self.currentVideoFrame
			# image = cv2.GaussianBlur(image,(3,3),0)
			# image = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
			# cv2.imwrite("hsv.jpg",image)
			# value = image[...,2]
			# cv2.imwrite("value.jpg",value)
			# hue = image[...,0]
			# hue_green = cv2.inRange(hue, 50, 70)
			# hue_blue = cv2.inRange(hue, 110, 130)
			# hue_red = cv2. inRange(hue, 160, 170)
			# hue_orange = cv2.inRange(hue, 5, 25)
			# hue_pink = cv2.inRange(hue, 150, 165)
			# hue_yellow = cv2.inRange(hue, 20, 40)
			# hue_purple = cv2.inRange(hue, 140,160)
			# cv2.imwrite("hue.jpg",hue)

			# self.roi = cv2.bitwise_and(hue, hue, mask = self.BlockMask)
			# cv2.imwrite("roi.jpg",self.roi)
			# red = cv2.bitwise_and(hue_red, hue_red, mask = self.BlockMask)
			# cv2.imwrite("red.jpg",red)
			# green = cv2.bitwise_and(hue_green, hue_green, mask = self.BlockMask)
			# cv2.imwrite("green.jpg",green)
			# blue = cv2.bitwise_and(hue_blue, hue_blue, mask = self.BlockMask)
			# cv2.imwrite("blue.jpg",blue)
			# yellow = cv2.bitwise_and(hue_yellow, hue_yellow, mask = self.BlockMask)
			# cv2.imwrite("yellow.jpg",yellow)
			# pink = cv2.bitwise_and(hue_pink, hue_pink, mask = self.BlockMask)
			# cv2.imwrite("pink.jpg", pink)
			# purple = cv2.bitwise_and(hue_purple, hue_purple, mask = self.BlockMask)
			# cv2.imwrite("purple.jpg", purple)
			# orange = cv2.bitwise_and(hue_orange, hue_orange, mask = self.BlockMask)
			# cv2.imwrite("orange.jpg", orange)
			# print("color guassian blur complete")
			# image = self.currentVideoFrame
			# image = cv2.GaussianBlur(image,(3,3),0)
			# print("color guassian blur complete")
			# image_green = cv2.inRange(image,(0,200,0),(0,255,0))
			# image_blue = cv2.inRange(image ,(0,0,200),(0,0,255))
			# image_red = cv2. inRange(image ,(200,0,0),(255,0,0))
			# image_orange = cv2.inRange(image ,(255,69,0),(255,165,0))
			# image_pink = cv2.inRange(image ,(255,105,180),(255,192,203))
			# image_yellow = cv2.inRange(image ,(255,255,0),(255,255,204))
			# image_purple = cv2.inRange(image ,(75,0,130),(153,50,204))
			# cv2.imwrite("color.jpg",image)
			
			# image_red[...,0] = cv2.bitwise_and(image_red[...,0], image_red[...,0], mask = self.BlockMask)
			# image_red[...,1] = cv2.bitwise_and(image_red[...,1], image_red[...,1], mask = self.BlockMask)
			# image_red[...,2] = cv2.bitwise_and(image_red[...,2], image_red[...,2], mask = self.BlockMask)
			# cv2.imwrite("image_red.jpg",image_red)

			# image_green[...,0] = cv2.bitwise_and(image_green[...,0], image_green[...,0], mask = self.BlockMask)
			# image_green[...,1] = cv2.bitwise_and(image_green[...,1], image_green[...,1], mask = self.BlockMask)
			# image_green[...,2] = cv2.bitwise_and(image_green[...,2], image_green[...,2], mask = self.BlockMask)
			# cv2.imwrite("image_green.jpg",image_green)

			# image_blue[...,0] = cv2.bitwise_and(image_blue[...,0], image_blue[...,0], mask = self.BlockMask)
			# image_blue[...,1] = cv2.bitwise_and(image_blue[...,1], image_blue[...,1], mask = self.BlockMask)
			# image_blue[...,2] = cv2.bitwise_and(image_blue[...,2], image_blue[...,2], mask = self.BlockMask)
			# cv2.imwrite("image_blue.jpg",image_blue)

			# image_yellow[...,0] = cv2.bitwise_and(image_yellow[...,0], image_yellow[...,0], mask = self.BlockMask)
			# image_yellow[...,1] = cv2.bitwise_and(image_yellow[...,1], image_yellow[...,1], mask = self.BlockMask)
			# image_yellow[...,2] = cv2.bitwise_and(image_yellow[...,2], image_pink[...,2], mask = self.BlockMask)
			# cv2.imwrite("image_yellow.jpg",image_yellow)

			# image_pink[...,0] = cv2.bitwise_and(image_pink[...,0], image_pink[...,0], mask = self.BlockMask)
			# image_pink[...,1] = cv2.bitwise_and(image_pink[...,1], image_pink[...,1], mask = self.BlockMask)
			# image_pink[...,2] = cv2.bitwise_and(image_pink[...,2], image_pink[...,2], mask = self.BlockMask)
			# cv2.imwrite("image_pink.jpg", image_pink)

			# image_purple[...,0] = cv2.bitwise_and(image_purple[...,0], image_purple[...,0], mask = self.BlockMask)
			# image_purple[...,1] = cv2.bitwise_and(image_purple[...,1], image_purple[...,1], mask = self.BlockMask)
			# image_purple[...,2] = cv2.bitwise_and(image_purple[...,2], image_orange[...,2], mask = self.BlockMask)
			# cv2.imwrite("image_purple.jpg", image_purple)

			# image_orange[...,0] = cv2.bitwise_and(image_orange[...,0], image_orange[...,0], mask = self.BlockMask)
			# image_orange[...,1] = cv2.bitwise_and(image_orange[...,1], image_orange[...,1], mask = self.BlockMask)
			# image_orange[...,2] = cv2.bitwise_and(image_orange[...,2], image_red[...,2], mask = self.BlockMask)
			# cv2.imwrite("image_orange.jpg", image_orange)

			# image = cv2.GaussianBlur(self.roi,(7,7),0)
			# edges = self.auto_canny(self.roi,.15)
			# cv2.imwrite("edges.jpg",edges)



		except:
			return None
	
	def workspaceedges(self):
			v = np.median(image)
			# apply automatic Canny edge detection using the computed median
			lower = int(max(0, (1.0 - sigma) * v))
			upper = int(min(255, (1.0 + sigma) * v))
			edged = cv2.Canny(image, lower, upper)
			cv2.imwrite("edges.jpg",edged)
		
			return None


	def detectBlocksInDepthImage(self):
		"""	TODO: Implement a blob detector to find blocks in the depth image """
		self.blockDetector()
		image = self.currentVideoFrame.astype(np.float32)
		hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
		hue_image = cv2.bitwise_and(hsv_image[...,0], hsv_image[...,0], mask = self.BlockMask)
		saturation_image = cv2.bitwise_and(hsv_image[...,1], hsv_image[...,1], mask = self.BlockMask)
		value_image = cv2.bitwise_and(hsv_image[...,2], hsv_image[...,2], mask = self.BlockMask)
		ret, value_threshold = cv2.threshold(value_image.astype(np.uint8),0,10, cv2.THRESH_BINARY)
		contours=cv2.findContours(value_threshold, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		if len(contours) != 0:  
			self.block_contours = contours[1]

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
		""" TODO: Using an Affine transformation, transform the depth frame to match the RGB frame """
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
		""" TODO: Load camera intrinsic matrix from file. """
		self.intrinsic_matrix = np.array([[492.43967452, 0.0, 305.98100007],[0.0, 492.06649804, 286.20904428],[0.0, 0.0, 1.0]])
		self.intrinsic_matrix_inverse = np.linalg.inv(self.intrinsic_matrix)
		return self.intrinsic_matrix
	
	def affineworkspace(self, coordinates):
		pts2 = np.array([[-.305, .305],[-.3050, -.3050],[.3050, -.3050],[.3050, .3050],[0.0, 0.0]]).astype(np.float32)
		pts1 = coordinates[0:5].astype(np.float32)
		#print("point s: ",pts1)
		#Defining Affine Matrix components in vector_x, RGB coordinates in matrix A, and Depth Coordinates in vector_b
		vector_x = np.ones((10,1))
	
		matrix_A = np.array([[ pts1[0][0], pts1[0][1], 1, 		   0, 		   0, 0],
							 [			0, 			0, 0, pts1[0][0], pts1[0][1], 1],
							 [ pts1[1][0], pts1[1][1], 1, 		   0, 		   0, 0],
							 [			0, 			0, 0, pts1[1][0], pts1[1][1], 1],
							 [ pts1[2][0], pts1[2][1], 1, 		   0, 		   0, 0],
							 [			0, 			0, 0, pts1[2][0], pts1[2][1], 1],
							 [ pts1[3][0], pts1[3][1], 1, 		   0, 		   0, 0],
							 [			0, 			0, 0, pts1[3][0], pts1[3][1], 1],
							 [ pts1[4][0], pts1[4][1], 1, 		   0, 		   0, 0],
							 [			0, 			0, 0, pts1[4][0], pts1[4][1], 1]]).astype(np.float32)

		vector_b = np.array([[pts2[0][0]],
							 [pts2[0][1]],
							 [pts2[1][0]],
							 [pts2[1][1]],
							 [pts2[2][0]],
							 [pts2[2][1]],
							 [pts2[3][0]],
							 [pts2[3][1]],
							 [pts2[4][0]],
							 [pts2[4][1]]]).astype(np.float32)

		# Calculating Pseudo-inverse of Matrix A of RGB coordinates
		matrix_A_transpose = np.transpose(matrix_A)
	
		matrix_A_inv = np.matmul(np.linalg.inv(np.matmul(matrix_A_transpose,matrix_A)),matrix_A_transpose)
	
		# Calculating affine matrix components
		vector_x = np.matmul(matrix_A_inv,vector_b)
	
		affineMatrixTransformation = np.array([[vector_x[0], vector_x[1], vector_x[2]],
						   [vector_x[3], vector_x[4], vector_x[5]],
						   [	  0,	       0, 	   1]])
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

		# def apriltagdetection(self):
	#detector = apriltag("tagStandard41h12", threads=4, decimate=2.0)
		#object_points = np.array([[-0.014,-0.014,0.0],
		# 							  [0.014, -0.014, 0.0],
		# 							  [0.014, 0.014, 0.0],
		# 							  [-0.014, 0.014, 0.0]],dtype = "double")

		# 	image = freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_HIGH)[0]
		# 	image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		# 	detections = detector.detect(image)		
		# 	#print("april tagging")

		# 	for tag in detections:
		# 		#print("\ntag ", tag)
		# 		image_points = tag['lb-rb-rt-lt']
		# 		#print("\nTranslation matrix : ",self.workspaceTransform(object_points, image_points))