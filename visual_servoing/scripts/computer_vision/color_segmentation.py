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
	HSV = np.array([179/360, 255/100, 255/100])
	kernel=np.ones((3,3), np.uint8)

	# # HSV extract range by clicking 
	# if template:
	# 	# Get image from template file
	# 	template_img = cv2.imread(template)
	# 	# template_data = cv2.imread(template, cv2.IMREAD_UNCHANGED)
	# 	# template_img = template_data[:,:,:3]

	# 	# Grab hsv converted image
	# 	# hsv_tmp = cv2.cvtColor(template_data, cv2.COLOR_BGR2HSV)
	# 	param = template_img

	# 	# Callback to grab data based on clicks
	# 	cv2.namedWindow("capturing_detection_boundary")
	# 	cv2.setMouseCallback('capturing_detection_boundary', clickBGR, param)
	# 	while True: 
	# 		cv2.imshow("capturing_detection_boundary", template_img)
	# 		if cv2.waitKey(0) == 27:
	# 			break	
	# 	cv2.destroWindow("capturing_detection_boundary")

	# # Grabbed hsv values from template analysis
	# min_hsv = np.array([7, 230, 168]) #23, 249, 254
	# max_hsv = np.array([29, 255, 255])
	min_hsv = np.array([6, 158, 168]) #23, 249, 254
	max_hsv = np.array([27, 255, 255])
	hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

	# Mask, erode to remove outliers, dilate to solidify cone outline
	mask = cv2.inRange(hsv_img, min_hsv, max_hsv)
	img_erosion = cv2.erode(mask, kernel, iterations=1)
	img_dilation = cv2.dilate(img_erosion, kernel, iterations=3)
	
	image, contour, hierarchy = cv2.findContours(img_dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	sorted_contours=sorted(contour, key=cv2.contourArea, reverse=True)
	# print(contour.shape)
	
	if len(contour)==0:
		bounding_box = ((0,0),(0,0))
	else:
		x,y,w,h = cv2.boundingRect(sorted_contours[0])
		bounding_box = ((x,y),(x+w,y+h))
	# print(bounding_box)
	

	# Debugging image surfaces
	if(debugging):
		cv2.imshow('initial', img)
		cv2.imshow('Input', mask)
		cv2.imshow('Erosion', img_erosion)
		cv2.imshow('Dilation', img_dilation)
		image_processed =  img
		image_final = cv2.rectangle(img_processed, bounding_box[0], bounding_box[1], (255,0,0), 2)
		cv2.imshow('Final', image_final)
		cv2.waitKey(0)
		# image_print(mask)
		image_final = cv2.rectangle(img, bounding_box[0], bounding_box[1], (255,0,0), 2)
		image_print(image_final)
	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box

def clickBGR(event, x, y, flags, params):
	hsv_img = cv2.cvtColor(params,cv2.COLOR_BGR2HSV)
	cv2.imshow('hsv', hsv_img)	
	if event == cv2.EVENT_LBUTTONDOWN:
		minx = x
		miny = y
		min_hsv = hsv_img[y][x]
		print([minx, miny])
		print(min_hsv)

	# return min_hsv, max_hsv

if __name__ == "__main__":

	img_path = "./test_images_cone/test17.jpg"
	# cone_template = "./test_images_cone/cone_template.png"
	cone_template = "./test_images_cone/bad1.png"
	
	img_processed = cv2.imread(img_path)
	# template_data = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
	# img_processed = template_data[:,:,:3]

	img_processed = cv2.imread(img_path)
	# print(img_processed)

	# Find hsv using template
	cd_color_segmentation(img_processed, template=cone_template, debugging=1)

	# Check after finding hsv valu
	# cd_color_segmentation(img_processed, debugging=1)

