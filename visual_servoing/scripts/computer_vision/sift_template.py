import cv2
import imutils
import numpy as np
import pdb
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
	Helper function to print out images, for debugging.
	Press any key to continue.
	"""
	winname = "Image"
	cv2.namedWindow(winname)        # Create a named window
	cv2.moveWindow(winname, 40,30)  # Move it to (40,30)
	cv2.imshow(winname, img)
	cv2.waitKey()
	cv2.destroyAllWindows()

def cd_sift_ransac(img, template):
	"""
	Implement the cone detection using SIFT + RANSAC algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	# Minimum number of matching features
	MIN_MATCH = 10
	# Create SIFT
	sift = cv2.xfeatures2d.SIFT_create()

	# Compute SIFT on template and test image
	kp1, des1 = sift.detectAndCompute(template,None)
	kp2, des2 = sift.detectAndCompute(img,None)

	# Find matches
	bf = cv2.BFMatcher()
	matches = bf.knnMatch(des1,des2,k=2)

	# Find and store good matches
	good = []
	for m,n in matches:
		if m.distance < 0.75*n.distance:
			good.append(m)

	# If enough good matches, find bounding box
	if len(good) > MIN_MATCH:
		src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

		# Create mask
		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
		matchesMask = mask.ravel().tolist()

		h, w = template.shape
		pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

		########## YOUR CODE STARTS HERE ##########

		x_min = y_min = x_max = y_max = 0
		dst = cv2.perspectiveTransform(pts, M)
		# draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
		# 				   singlePointColor=None,
		# 				   matchesMask=matchesMask,  # draw only inliers
		# 				   flags=2)
		# img3 = cv2.drawMatches(img, kp1, template, kp2, good, None, **draw_params)
		# img3 = cv2.polylines(img3, [np.int32(dst)], True, (0, 0, 255), 3, cv2.LINE_AA)
		#
		# cv2.imshow("result", img3)
		# cv2.waitKey()
		dst = np.int32(dst)
		bot_left = dst[0,0,:]
		top_left = dst[1,0,:]
		top_right = dst[2,0,:]
		bot_right = dst[3,0,:]
		########### YOUR CODE ENDS HERE ###########

		# Return bounding box
		return ((bot_left[0], bot_left[1]), (top_right[0], top_right[1]))
	else:

		print( "[SIFT] not enough matches; matches: ", len(good))

		# Return bounding box of area 0 if no match found
		return ((0,0), (0,0))

def cd_template_matching(img, template):
	"""
	Implement the cone detection using template matching algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	template_canny = cv2.Canny(template, 50, 200)

	# Perform Canny Edge detection on test image
	grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	img_canny = cv2.Canny(grey_img, 50, 200)

	# Get dimensions of template
	(img_height, img_width) = img_canny.shape[:2]

	# Keep track of best-fit match
	best_match = None

	# Loop over different scales of image

	for scale in np.linspace(1.5, .5, 50):
		# Resize the image
		resized_template = imutils.resize(template_canny, width = int(template_canny.shape[1] * scale))
		(h,w) = resized_template.shape[:2]
		# Check to see if test image is now smaller than template image
		if resized_template.shape[0] > img_height or resized_template.shape[1] > img_width:
			continue

		########## YOUR CODE STARTS HERE ##########
		# Use OpenCV template matching functions to find the best match
		# across template scales.
		accuracy_img = cv2.matchTemplate(img_canny,resized_template,cv2.TM_CCORR_NORMED)
		min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(accuracy_img)

		if best_match is None or max_val > best_match:
			best_match = max_val
			bounding_box = (max_loc,(max_loc[0] + w, max_loc[1] + h))
			saved_h = h
			saved_w = w
			saved_loc = max_loc

		# Remember to resize the bounding box using the highest scoring scale
		# x1,y1 pixel will be accurate, but x2,y2 needs to be correctly scaled

		########### YOUR CODE ENDS HERE ###########
	dst = [[saved_loc[0],saved_loc[1]], [saved_loc[0], saved_loc[1] + saved_h], [saved_loc[0] + saved_w, saved_loc[1] + saved_h], [saved_loc[0] + saved_w, saved_loc[1]]]
	im3 = cv2.polylines(img, [np.int32(dst)], True, (0, 0, 255), 2, cv2.LINE_AA)
	cv2.imshow("result",im3)
	cv2.waitKey()
	# Remember to resize the bounding box using the highest scoring scale
		# x1,y1 pixel will be accurate, but x2,y2 needs to be correctly scaled


		########### YOUR CODE ENDS HERE ###########

	return bounding_box
