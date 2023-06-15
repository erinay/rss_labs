import cv2
import numpy as np
import pdb
import os

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template=None, debugging=0):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########

	# Useful Params
	kernel=np.ones((5,5), np.uint8)
	width = img.shape[1]
	height = img.shape[0]
	# if template:
	# 	# Get image from template file
	# 	# # template_img = cv2.imread(template)
	# 	# template_data = cv2.imread(template, cv2.IMREAD_UNCHANGED)
	# 	# template_img = template_data[:,:,:3]
	# 	img = cv2.imread(template)
	# 	template_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


	# 	dim = template_img.shape
	# 	width = dim[1]
	# 	height = dim[0]
	# 	lookahead_upper = height*6/10
	# 	lookahead_lower = height*9/10
	# 	image_1 = cv2.rectangle(template_img, (0,0), (width, lookahead_upper), (0,0,0), -1)
	# 	image = cv2.rectangle(image_1, (0,lookahead_lower), (width, height), (0,0,0), -1)

	# 	# Grab hsv converted image
	# 	# hsv_tmp = cv2.cvtColor(template_data, cv2.COLOR_BGR2HSV)
	# 	param = img

	# 	# Callback to grab data based on clicks
	# 	cv2.namedWindow("capturing_detection_boundary")
	# 	cv2.setMouseCallback('capturing_detection_boundary', clickBGR, param)
	# 	while True: 
	# 		cv2.imshow("capturing_detection_boundary", template_img)
	# 		if cv2.waitKey(0) == 27:
	# 			break	
	# 	cv2.destroWindow("capturing_detection_boundary")

	# For testing purposes
	# dim = img.shape
	# width = dim[1]
	# height = dim[0]
	# lookahead_upper = int(height*5.75/12)
	# lookahead_lower = int(height*11.5/12)
	# image_l = cv2.rectangle(img, (0,0), (width, lookahead_upper), (0,0,0), -1)
	# img = cv2.rectangle(image_l, (0,lookahead_lower), (width, height), (0,0,0), -1)
	# points = np.array([[width*0.25, height*0.8], [width*0.8, height*0.8], [width/2, 100]], dtype=np.int32)
	# img = cv2.fillPoly(image_u, pts=[points], color=(0,0,0))
	# cv2.imshow('triangle',img)

	# # Grabbed hsv values from template analysis
	min_gray = 180
	#min_gray = 210 #for car 42
	max_gray = 255

	gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	# cv2.imshow('pre-blur', gray_img)
	cv2.GaussianBlur(gray_img, (5,5), 0)
	# cv2.imshow('blur', gray_img)
	# Mask, erode to remove outliers, dilate to solidify cone outline
	mask = cv2.inRange(gray_img, min_gray, max_gray)
	img_erosion = cv2.erode(mask, kernel, iterations=1)
	img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)
	
	image, contour, hierarchy = cv2.findContours(img_dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	#contour, _ = cv2.findContours(img_dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	sorted_contours=sorted(contour, key=cv2.contourArea, reverse=True)

	########### YOUR CODE ENDS HERE ###########
	dst = cv2.Canny(mask, 100, 200)
    
    # Copy edges to the images that will display the results in BGR
	cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
	cpdst = np.copy(cdst)
	lines = cv2.HoughLinesP(dst, 1, np.pi / 180, 10, None, 3, 220)
	
	left = []
	right = []
    
	if lines is not None:
		for i in range(0, len(lines)):
			l = lines[i][0]
			cv2.line(cpdst, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

	# cv2.imshow("Source", dst)
	# cv2.imshow("Detected Lines (in red)", cpdst)


	# Get Averages
	#Filtering lines
	for line in lines:
		# print(line.reshape(4))
		x1, y1, x2, y2 = line.reshape(4)
		lin_reg = np.polyfit([x1,x2], [y1,y2],1)
		slope = lin_reg[0]
		intercept = lin_reg[1]

		x_avg = (x1+x2)/2

		if (slope<-0.25 and x_avg<width/2):
			#print('lieft')
			#print(slope)
			left.append(lin_reg)
		elif(slope>0.25 and x_avg>width/2):
			# print('right')
			# print(slope)
                        if(slope < 1):
			    right.append(lin_reg)
        #line_intercepts
        #if len(right)!=0:
         #   right_copy = right.copy() 
          #  intercepts = right[:][1]
        #    mean_b = np.mean(intercepts)
         #   for index, b in enumerate(intercepts):
        #        error = b-mean_b
         #       if np.abs(error)>20:
          #          left_copy.remove(b)
                        
           # left=left_copy
                    

	# Add lines if it isn't found
	#img height,width =- 376, 672
	if len(left)==0:
		lin_reg = (-4,500)
		left.append(lin_reg)
	if len(right)==0:
		lin_reg = (4, -2000)
		right.append(lin_reg)

	# Visualize for debugging
	left_avg = np.average(left, axis=0)
	# print("left")
	# print(left_avg)
	right_avg = np.average(right, axis=0)
	# print("right")
	# print(right_avg)
	left_points  = make_points(left_avg, height)
	right_points = make_points(right_avg, height)


	avg_img = np.copy(cdst)
	cv2.line(avg_img, (left_points[0], left_points[1]), (left_points[2], left_points[3]), (0,0,255), 3, cv2.LINE_AA)
	cv2.line(avg_img, (right_points[0], right_points[1]), (right_points[2], right_points[3]), (0,0,255), 3, cv2.LINE_AA)
	#cv2.imshow("Debug Lines", avg_img)
	

	# find intersection (m,b)
	# Turn more leftward
	inter_x = int((right_avg[1]-left_avg[1])/(left_avg[0]-right_avg[0]))#-5
	#inter_x = int((right_avg[1]-left_avg[1]))
	inter_y = int(left_avg[0]*inter_x+left_avg[0]) #intersection point


	## 2nd option: average of top of detected line
	#lx_u = (height*5.5/12-left_avg[1])/left_avg[0]
	#rx_u = (height*5.5/12-right_avg[1])/right_avg[0]
	#inter_x = int((lx_u+rx_u)/2)

	# Visualize (x,y) point; where y is arbitrary
	# TODO: find optimal y
	y_viz = int(height*0.8)
	x_viz = int(inter_x-width/2)

	viz_target = np.copy(cdst)
	cv2.circle(viz_target, center=(inter_x, y_viz), radius=5, color=(0,255,0), thickness=-1)
	cv2.line(viz_target, (width/2, 0),(width/2, height), (0,0,255), thickness=2)
	#cv2.imshow('target', viz_target)

	#cv2.waitKey()


	pts=(x_viz, y_viz)
	return left_points, right_points, pts



	# Return bounding box
	# return bounding_box

def clickBGR(event, x, y, flags, params):
	gray_img = cv2.cvtColor(params,cv2.COLOR_BGR2GRAY)
	cv2.imshow('hsv', gray_img)	
	if event == cv2.EVENT_LBUTTONDOWN:
		minx = x
		miny = y
		min_hsv = gray_img[y][x]
		print(min_hsv)

	# return min_hsv, max_hsv

def make_points(line_reg, height):
	slope, y_int = line_reg 
	y1 = height
	y2 = int(y1*3/5)
	x1 = int((y1-y_int) // slope)
	x2 = int((y2-y_int) // slope)

	return np.array([x1, y1, x2, y2])	

if __name__ == "__main__":

	img_path = "./test_images_track/pic13.png"
	cone_template = "./test_images_track/1.png"
	
	img_processed = cv2.imread(img_path)
	# template_data = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
	# img_processed = template_data[:,:,:3]

	# Find hsv using template
	cd_color_segmentation(img_processed, template=cone_template, debugging=1)

	# Check after finding hsv valu
	# cd_color_segmentation(img_processed, debugging=0)

