#!/usr/bin/env python

################################################################################
## {Description}: Processing Time Lapse Images into a Video
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from __future__ import print_function
from __future__ import division

import sys
import rospy
import cv2
import imutils
from datetime import datetime
import signal
import time
import os
import rospkg
from imutils import paths
import progressbar

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class VideoTimeLapse_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('VideoTimeLapse_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# initialize the FourCC and video writer
		self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
		self.writer = None

		# grab the paths to the images, then initialize output file name and
		# output path 
		self.rospack = rospkg.RosPack()
		self.p = os.path.sep.join([self.rospack.get_path('timelapse_capture_camera'), 
			"output", "images", "2020-04-16-1004"])
		self.imagePaths = list(paths.list_images(self.p))

		self.p_out = os.path.sep.join([self.rospack.get_path('timelapse_capture_camera'), 
			"output", "videos"])
		self.outputFile = "{}.avi".format(self.p.split(os.path.sep)[8])
		self.outputPath = os.path.join(self.p_out, self.outputFile)
		os.makedirs(self.outputPath)

		rospy.logwarn("Building {}...".format(self.outputPath))

		# initialize the progress bar
		self.widgets = ["Building Video: ", progressbar.Percentage(), " ", 
			progressbar.Bar(), " ", progressbar.ETA()]
		self.pbar = progressbar.ProgressBar(maxval=len(self.imagePaths), 
			widgets=self.widgets).start()

	def makeVid(self):
		# loop over all sorted input image paths
		for (i, imagePath) in enumerate(sorted(self.imagePaths, key=get_number)):
			# load the image
			image = cv2.imread(imagePath)

			# initialize the video writer if needed
			if self.writer is None:
				(H, W) = image.shape[:2]
				self.writer = cv2.VideoWriter(self.outputPath, self.fourcc, 
					30, (W, H), True)

		# write the image to output video
		self.writer.write(image)
		self.pbar.update(i)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("VideoTimeLapse_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

# function to get the frame number from the image path
def get_number(imagePath):
	return int(imagePath.split(os.path.sep)[-1][:-4])

def main(args):
	vn = VideoTimeLapse_node()

	try:
		vn.makeVid()
#		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("VideoTimeLapse_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("VideoTimeLapse_node [ONLINE]...")
	main(sys.argv)
