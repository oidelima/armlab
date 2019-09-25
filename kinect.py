import cv2
from apriltag import apriltag
import numpy as np
from PyQt4.QtGui import QImage
import freenect
import copy

class Kinect():
	def __init__(self):
		self.currentVideoFrame = np.array([])
		self.currentDepthFrame = np.array([])
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

		""" block info """
		self.block_contours = np.array([])
		self.block_coordinates = []
		self.block_coordinates_raw = []

	def captureVideoFrame(self):
		"""                      
		Capture frame from Kinect, format is 24bit RGB    
		"""
		if(self.kinectConnected):
			self.currentVideoFrame = freenect.sync_get_video()[0]
			#self.currentHiResFrame = freenect.sync_get_video_with_res()[0]
		else:
			self.loadVideoFrame()
		self.processVideoFrame()
		

	def processVideoFrame(self):
		cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),1)


	def captureDepthFrame(self):
		"""                      
		Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
		"""
		if(self.kinectConnected):
			if(self.kinectCalibrated):
				self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
			else:
				self.currentDepthFrame = freenect.sync_get_depth()[0]
		else:
			self.loadDepthFrame()

	
	def loadVideoFrame(self):
		self.currentVideoFrame = cv2.cvtColor(
			cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
			cv2.COLOR_BGR2RGB
			)

	def loadDepthFrame(self):
		self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

	def convertFrame(self):
		""" Converts frame to format suitable for Qt  """
		try:
			img = QImage(self.currentVideoFrame,
							 self.currentVideoFrame.shape[1],
							 self.currentVideoFrame.shape[0],
							 QImage.Format_RGB888
							 )
			return img
		except:
			return None

	def convertDepthFrame(self):
		""" Converts frame to a colormaped format suitable for Qt  
			Note: this cycles the spectrum over the lowest 8 bits
		"""
		try:

			""" 
			Convert Depth frame to rudimentary colormap
			"""
			self.DepthHSV[...,0] = self.currentDepthFrame
			self.DepthHSV[...,1] = 0x9F
			self.DepthHSV[...,2] = 0xFF
			self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
			cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),1)

			img = QImage(self.DepthCM,
							 self.DepthCM.shape[1],
							 self.DepthCM.shape[0],
							 QImage.Format_RGB888
							 )
			return img
		except:
			return None

	def getAffineTransform(self, coord1, coord2):
		"""
		Given 2 sets of corresponding coordinates, 
		find the affine matrix transform between them.

		TODO: Rewrite this function to take in an arbitrary number of coordinates and 
		find the transform without using cv2 functions
		"""
	### coord1 are RGB coordinates and coord2 are Depth Camera coordinates
		pts1 = coord2[0:5].astype(np.float32)
		pts2 = coord1[0:5].astype(np.float32)
	### Defining Affine Matrix components in vector_x, RGB coordinates in matrix A, and Depth Coordinates in vector_b
		vector_x = np.ones((8,1))
		matrix_A = np.array([[pts1[0][0], pts1[0][1], 1, 0.0, 0.0, 0.0],
							 [0.0, 0.0, 0.0, pts1[0][0], pts1[0][1], 1],
							 [pts1[1][0], pts1[1][1], 1, 0.0, 0.0, 0.0],
							 [0.0, 0.0, 0.0, pts1[1][0], pts1[1][1], 1],
							 [pts1[2][0], pts1[2][1], 1, 0.0, 0.0, 0.0],
							 [0.0, 0.0, 0.0, pts1[2][0], pts1[2][1], 1],
							 [pts1[3][0], pts1[3][1], 1, 0.0, 0.0, 0.0],
							 [0.0, 0.0, 0.0, pts1[3][0], pts1[3][1], 1]]).astype(np.float32)
		vector_b = np.array([[pts2[0][0]],
							 [pts2[0][1]],
							 [pts2[1][0]],
							 [pts2[1][1]],
							 [pts2[2][0]],
							 [pts2[2][1]],
							 [pts2[3][0]],
							 [pts2[3][1]]]).astype(np.float32)
	### Calculating Pseudo-inverse of Matrix A of RGB coordinates
		matrix_A_transpose = np.transpose(matrix_A)
	
		matrix_A_inv = np.matmul(np.linalg.inv(np.matmul(matrix_A_transpose,matrix_A)),matrix_A_transpose)
	
	### Calculating affine matrix components
		vector_x = np.matmul(matrix_A_inv,vector_b)
	
		affineMatrixTransformation = np.array([[vector_x[0], vector_x[1], vector_x[2]],
						   [vector_x[3], vector_x[4], vector_x[5]],
						   [	  0,	       0, 	   1]])
		self.depth2rgb_affine = vector_x.reshape(2,3)
		return self.depth2rgb_affine


	def registerDepthFrame(self, frame):
		"""
		TODO:
		Using an Affine transformation, transform the depth frame to match the RGB frame
		"""
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
		"""
		TODO:
		Load camera intrinsic matrix from file.
		"""
		self.intrinsic_matrix = np.array([[523.07835767, 0.0,303.93789919],[0.0, 522.81772202, 278.02015252],[0.0, 0.0, 1.0]])
		self.intrinsic_matrix_inverse = np.linalg.inv(self.intrinsic_matrix)
		return self.intrinsic_matrix
	
	
	def affineworkspace(self, coordinates):
		pts2 = np.array([[-.305, .305],[-.3050, -.3050],[.3050, -.3050],[.3050, .3050],[0.0, 0.0]]).astype(np.float32)
	### coord1 are RGB coordinates and coord2 are Depth Camera coordinates
		pts1 = coordinates[0:5].astype(np.float32)
		print("point s: ",pts1)
	### Defining Affine Matrix components in vector_x, RGB coordinates in matrix A, and Depth Coordinates in vector_b
		vector_x = np.ones((8,1))
	
		matrix_A = np.array([[pts1[0][0], pts1[0][1], 1, 0.0, 0.0, 0.0],
							 [0.0, 0.0, 0.0, pts1[0][0], pts1[0][1], 1],
							 [pts1[1][0], pts1[1][1], 1, 0.0, 0.0, 0.0],
							 [0.0, 0.0, 0.0, pts1[1][0], pts1[1][1], 1],
							 [pts1[2][0], pts1[2][1], 1, 0.0, 0.0, 0.0],
							 [0.0, 0.0, 0.0, pts1[2][0], pts1[2][1], 1],
							 [pts1[3][0], pts1[3][1], 1, 0.0, 0.0, 0.0],
							 [0.0, 0.0, 0.0, pts1[3][0], pts1[3][1], 1],
							 [pts1[4][0], pts1[4][1], 1, 0.0, 0.0, 0.0],
							 [0.0, 0.0, 0.0, pts1[4][0], pts1[4][1], 1]]).astype(np.float32)

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

	### Calculating Pseudo-inverse of Matrix A of RGB coordinates
		matrix_A_transpose = np.transpose(matrix_A)
	
		matrix_A_inv = np.matmul(np.linalg.inv(np.matmul(matrix_A_transpose,matrix_A)),matrix_A_transpose)
	
	### Calculating affine matrix components
		vector_x = np.matmul(matrix_A_inv,vector_b)
	
		affineMatrixTransformation = np.array([[vector_x[0], vector_x[1], vector_x[2]],
						   [vector_x[3], vector_x[4], vector_x[5]],
						   [	  0,	       0, 	   1]])
		self.workcamera_affine = vector_x.reshape(2,3)
		return self.workcamera_affine

	def apriltagdetection(self):
		detector = apriltag("tagStandard41h12", threads=4, decimate=1.0)
		object_points = np.array([[-0.028,-0.028,0.0],
								  [0.028, -0.028, 0.0],
								  [0.028, 0.028, 0.0],
								  [-0.028, 0.028, 0.0]],dtype = "double")

		image = freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_HIGH)[0]
		image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
		detections = detector.detect(image)		
		for tag in detections:
			print("taging ", tag['id'])
			image_points = tag['lb-rb-rt-lt']
			print("Translation matrix : ",self.workspaceTransform(object_points, image_points))





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
			self.DepthH[...,0] = image[...,0]
			self.DepthH[...,1] = image[...,0]
			self.DepthH[...,2] = image[...,0]
			self.DepthH = cv2.cvtColor(self.DepthH,cv2.COLOR_HSV2RGB)
			img_h = QImage(self.DepthH, self.DepthH.shape[1], self.DepthH.shape[0], QImage.Format_RGB888)

			self.DepthS[...,0] = image[...,1]
			self.DepthS[...,1] = image[...,1]
			self.DepthS[...,2] = image[...,1]
			self.DepthS = cv2.cvtColor(self.DepthS,cv2.COLOR_HSV2RGB)
			img_s = QImage(self.DepthS, self.DepthS.shape[1], self.DepthS.shape[0], QImage.Format_RGB888)

			self.DepthV[...,0] = image[...,2]
			self.DepthV[...,1] = image[...,2]
			self.DepthV[...,2] = image[...,2]
			self.DepthV = cv2.cvtColor(self.DepthV,cv2.COLOR_HSV2RGB)
			img_v = QImage(self.DepthV, self.DepthV.shape[1], self.DepthV.shape[0], QImage.Format_RGB888)
			return img_h,img_s,img_v
		except:
			return None

	def convertBlockDepthFrame(self):
		""" Converts frame to a colormaped format suitable for Qt  
			Note: this cycles the spectrum over the lowest 8 bits
		"""
		try:

			""" 
			Convert Depth frame to rudimentary colormap
			"""
			#self.DepthHSVThreshold[...,0] = self.currentDepthFrame/2
			#print(self.currentDepthFrame)
			self.BlockMask = np.zeros((480,640,1)).astype(np.uint8)
			self.DepthHSVThreshold[...,0] = self.currentDepthFrame
			self.DepthHSVThreshold[self.currentDepthFrame > 710 ,0] = 0
			self.DepthHSVThreshold[self.currentDepthFrame < 650 ,0] = 0
			#self.DepthHSVThreshold[...,0] = (self.DepthHSVThreshold[...,0] - 650)*256/65
			self.BlockMask[self.DepthHSVThreshold[...,0] != 0] = 255
			
			self.DepthHSVThreshold[...,1] = 0x9F
			self.DepthHSVThreshold[...,2] = 0xFF
			self.DepthHSVThreshold[self.currentDepthFrame > 710 ,1] = 255
			self.DepthHSVThreshold[self.currentDepthFrame < 650 ,1] = 255
			self.DepthHSVThreshold[self.currentDepthFrame > 710 ,2] = 255
			self.DepthHSVThreshold[self.currentDepthFrame < 650 ,2] = 255
			self.DepthCMThreshold = cv2.cvtColor(self.DepthHSVThreshold,cv2.COLOR_HSV2RGB)
			self.detectBlocksInDepthImage()
			cv2.drawContours(self.DepthCMThreshold,self.block_contours,-1,(0,0,0),1)
			img = QImage(self.DepthCMThreshold, self.DepthCMThreshold.shape[1], self.DepthCMThreshold.shape[0], QImage.Format_RGB888)
			return img
		except:
			return None


	def blockDetector(self):
		"""
		TODO:
		Implement your block detector here.  
		You will need to locate
		blocks in 3D space
		"""
		#camera_coordinates = np.array([[x],[y],[1]]).astype(np.float32)
		#xy_world = np.matmul(self.kinect.workcamera_affine,camera_coordinates) 
		#z_w = .1236*np.tan(z/2842.5 + 1.1863) 
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

	def detectBlocksInDepthImage(self):
		"""
		TODO:
		Implement a blob detector to find blocks
		in the depth image
		"""
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


		return None

	def workspaceTransform(self, object_points, image_points):
		camera_intrinsic_matrix = self.loadCameraCalibration()
		dist_coeffs = np.array([[2.81543125e-01,-1.41320279e+00,-1.51367772e-03,4.17417744e-03,2.92511446e+00]]).astype(np.float32)
		(success, rot_vec, trans_vec) = cv2.solvePnP(object_points, image_points, camera_intrinsic_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
		self.rotation_matrix = np.array([[0.0, -rot_vec[2], rot_vec[1]],
										[rot_vec[2], 0.0, -rot_vec[0]],
										[-rot_vec[1], rot_vec[0], 0.0]])
	
	#self.rotation_matrix = rot_vec	
		self.translation_matrix = trans_vec
		#return self.rotation_matrix, self.translation_matrix
		return trans_vec