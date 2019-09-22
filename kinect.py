import cv2
import apriltag
import numpy as np
from PyQt4.QtGui import QImage
import freenect

class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
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
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
        self.processVideoFrame()
        

    def processVideoFrame(self):
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)


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
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

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
	
    def workspaceTransform(self, coordinates):
	camera_matrix = self.loadCameraCalibration()
	dist_coeffs = np.array([[2.81543125e-01,-1.41320279e+00,-1.51367772e-03,4.17417744e-03,2.92511446e+00]]).astype(np.float32)
	#flags = cv2.CV_ITERATIVE
	model_points = np.array([[-.3050, .3050, .9400],[-.3050, -.3050, .9400],[.3050, -.3050, .9400],[.3050, .3050, .9400],[0.0, 0.0, .9400]]).astype(np.float32)
	
	image_points = []	
	for coordinate in coordinates:
		coordinate = np.array([[coordinate[0]],[coordinate[1]],[1]]).astype(np.float32)
		print("Coordinate : ", coordinate)
		image_points.append(np.matmul(self.intrinsic_matrix_inverse,coordinate)[:2])
	image_points = np.array(image_points)
	print("image points : ",image_points)
	(success, rot_vec, trans_vec) = cv2.solvePnP(model_points, image_points, camera_matrix,dist_coeffs)
	self.rotation_matrix = np.array([[0.0, -rot_vec[2], rot_vec[1]],
				[rot_vec[2], 0.0, -rot_vec[0]],
				[-rot_vec[1], rot_vec[0], 0.0]])
	
	#self.rotation_matrix = rot_vec	
	self.translation_matrix = trans_vec
	return self.rotation_matrix, self.translation_matrix

    def apriltagdetection(self):
	detector = apriltag.Detector(families='tag36h11', border=1, nthreads=4, quad_decimate=1.0,quad_blur=0.0,refine_edges=True,debug=False,quad_contours=True)
	image = self.currentVideoFrame	
	image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	print(image)
	detections = detector.detect(image)
	print detections

    def convertGrayFrame(self):
        """ Converts frame to format to Gray for Qt  """
        try:
	    image = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2GRAY)
            img = QImage(image,
                             image.shape[1],
                             image.shape[0],
                             QImage.Format_Indexed8
                             )
            return img
        except:
            return None
   
    def convertHSVFrame(self):
        """ Converts frame to format to HSV for Qt  """
        try:
	    image = cv2.cvtColor(self.currentVideoFrame, cv2.COLOR_RGB2HSV)
            img = QImage(image,
                             image.shape[1],
                             image.shape[0],
                             QImage.Format_RGB888
                             )
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
	image = self.currentVideoFrame
	image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	
        pass

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        pass
