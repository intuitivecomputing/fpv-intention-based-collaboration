import os
import cv2

def frames2video(path_to_img, img_files, video_name="new_video", fps=30, size=None, 
				is_color=True, format="XVID"):
	"""
	Create a video from a list of images.
	By default, the video will have the size of the first image.
    It will resize every image to this size before adding them to the video.
	---
	Parameters:
		path_to_img  |  path to images folder
		video_name   |  the name of output video
		img_files |  list of images to use in the video
		fps          |  frame per second
	    size         |	size of each frame
		is_color     |	color
		format       |	video format  see http://www.fourcc.org/codecs.php
	---	
	Return:          
		video		 |	a video file  see http://opencv-python-tutroals.readthedocs.org/en/latest/.
    
    """

	fourcc = 0  #uncompressed
	# fourcc = cv2.VideoWriter_fourcc(*format)  #compressed

	video = None
	for image_name in img_files:
		img = cv2.imread(path_to_img + image_name)
		if video is None:
			if size is None:
				size = img.shape[1], img.shape[0]
			video = cv2.VideoWriter(video_name, fourcc, float(fps), size, is_color)
		if size[0] != img.shape[1] and size[1] != img.shape[0]:
			img = cv2.resize(img, size)
		video.write(img)
	return video

if __name__ == "__main__":
	video_name = "Ikea_Chair_Assembly_rgb.avi"
	path_to_img = "Ikea_Chair_Assembly/collab-rgb-frames-cam-3-collection-4/"
	img_files = os.listdir(path_to_img)
	img_files.sort()
	video = frames2video(path_to_img, img_files, video_name,  fps=30, size=None,is_color=True, format="XVID")
	video.release()  #save files
