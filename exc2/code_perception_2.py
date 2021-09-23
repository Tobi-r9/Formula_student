'''
Change the fields below for your code to make it more presentable:

@author :       Ajinkya Khoche
email:          khoche@kth.se
Date:           2018/09/03   
Description:    This program
				- Reads a video from 'test_videos' folder and reads every frame 
				till end of video. It stores every frame in variable of same name.
				- Your algorithm should process 'frame' variable (or frame_cloned. 
				its good to clone the frame to preserve original data)
				- The result of your algorithm should be lists of 'bounding_boxes'
				and 'labels'. 
				- The helper code takes 'bounding_boxes' to draw rectangles on the
				positions where you found the cones. It uses corresponding 'labels'
				to name which type of cone was found within 'bounding_boxes'.  

				Color Convention that we follow:
				---------------- 
					0-  YELLOW
					1-  BLUE
					2-  ORANGE
					3-  WHITE
					4-  BLACK

				This basically means that if labels[i] = 0, then you can set the i_th
				bounding_box as 'yellow cone'    
'''
import cv2 
import numpy as np

def template_matching(template, frame, threshold):
	"""template matching given a template and an image

	Args:
		template (np.array): template image
		frame (np.array): image on which template should be found
		threshold (float): float for thresholding the results

	Returns:
		list: list with bounding boxes
	"""
	rectangles = []
	result = cv2.matchTemplate(
					frame, 
					template, 
					cv2.TM_SQDIFF_NORMED
					)
	locations = np.where(result <= threshold)
	locations = list(zip(*locations[::-1]))			
	if  locations:
		box_width = template.shape[1]
		bow_height = template.shape[0]
		
		for loc in locations:
			top_left = loc
			bottom_right = (int(top_left[0]) + box_width, int(top_left[1]) + bow_height)
			rect = [int(loc[0]), int(loc[1]), bottom_right[0], bottom_right[1]]
			rectangles.append(rect)
			rectangles.append(rect)

		# print(rectangles)					
		rectangles, weights = cv2.groupRectangles(rectangles, groupThreshold=1, eps=1)
	return rectangles

def main():
	# Read video from disk and count frames
	cap = cv2.VideoCapture('test_videos/20180626_102839414.mp4')
	frameCount = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
	fps = int(cap.get(cv2.CAP_PROP_FPS))
	count = 0
	template_blue = cv2.imread("reference_img/video_1_blau.png")
	template_orange = cv2.imread("reference_img/video_1_orange.png")
	# template_edge = cv2.Canny(template_img, 50, 100)
	'''
	NOTE: For this example, bounding_box is manually defined.
	For your assignment you need to comment the definitions of
	'bounding_boxes' and 'labels' below and use bounding_box 
	obtained from your algorithm. Its the structure which is 
	provided
	'''
	threshold = 0.4
	bounding_box = []
	labels = []
	# bounding_box.append(np.array((100,100,150,200)))
	# bounding_box.append(np.array((200,200,300,400)))
	# bounding_box.append(np.array((500,500,600,700)))
	# labels = [0,2,1]
	# Read every frame till the end of video
	while count < frameCount:
		ret, frame = cap.read()
		if ret == True:
			count = count + 1
			bounding_box = []
			frame_cloned = np.copy(frame)
			
			hsv_frame = cv2.cvtColor(frame_cloned, cv2.COLOR_BGR2HSV)
			frame = cv2.inRange(hsv_frame, (0, 180, 129), (15, 229, 243))
			orange_hsv = cv2.cvtColor(template_orange, cv2.COLOR_BGR2HSV)
			orange_tmp = cv2.inRange(orange_hsv, (0, 180, 129), (15, 229, 243))
			
			rectangles_orange = template_matching(orange_tmp, frame, threshold)
			labels_orange = [2 for _ in rectangles_orange]
			labels += labels_orange
			bounding_box.append(rectangles_orange[0])
			'''
				#
				#
				#
				#

				Your algorithm to process frame comes here
				The result may be:
				- a list of bounding_boxes (format corresponding to cv2.rectangle) and 
				- a list of labels (integer for simplicity)

				feel free to choose any other formulation of results
				#
				# 
				#
				# 
				'''
			for box, i in zip(bounding_box, range(len(bounding_box))):
				'''
				A quick note. A bounding box can have two formulations:
				1. xmin, ymin, w, h : which means first two numbers signify 
				the top left coordinate of rectangle and last two signify 
				its width and height respectively
				
				2. xmin, ymin, xmax, ymax : the first two coordinates signify
				the top left coordinate and last two coordinates signify the
				bottom right coordinate of rectangle.

				In our example, we use formulation 2, but its easy to interchange.
				follow comments.
				'''
				xmin = box[0]
				ymin = box[1]
				xmax = box[2]   # w = box[2]
				ymax = box[3]   # h = box[3]

				if labels[i] == 0:
					cv2.rectangle(frame_cloned ,(xmin,ymin), (xmax,ymax), (0, 255, 255), 5)     #cv2.rectangle(frame_cloned ,(xmin,ymin), (xmin + w,ymin + h), (0,255,0), 5)
					cv2.putText(frame_cloned, 'yellow cone', (int(xmin),int(ymin)-10), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
				if labels[i] == 1:
					cv2.rectangle(frame_cloned ,(xmin,ymin), (xmax,ymax), (255, 0, 0), 5)     #cv2.rectangle(frame_cloned ,(xmin,ymin), (xmin + w,ymin + h), (0,255,0), 5)
					cv2.putText(frame_cloned, 'blue cone', (int(xmin),int(ymin)-10), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
				if labels[i] == 2:
					cv2.rectangle(frame_cloned ,(xmin,ymin), (xmax,ymax), (0,165,255), 5)     #cv2.rectangle(frame_cloned ,(xmin,ymin), (xmin + w,ymin + h), (0,255,0), 5)
					cv2.putText(frame_cloned, 'orange cone', (int(xmin),int(ymin)-10), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)

			cv2.imshow('Original frame', frame)
			cv2.waitKey(10)
			cv2.imshow('Result of cone detection', frame_cloned)
			cv2.waitKey(10)
if __name__ == '__main__':
	main()