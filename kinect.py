import cv2
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
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)

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
        intrinsic_matrix = np.array([[523.07835767, 0.0,303.93789919],[0.0, 522.81772202, 278.02015252],[0.0, 0.0, 1.0]])
        return intrinsic_matrix
    
    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        pass

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        pass
