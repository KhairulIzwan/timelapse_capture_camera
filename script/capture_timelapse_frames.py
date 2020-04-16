#!/usr/bin/env python

################################################################################
## {Description}: Capturing a Time Lapse Sequence from USB type camera
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

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class TimeLapse_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('TimeLapse_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Give the OpenCV display window a name
		self.cv_window_name = "Camera Preview"

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# initialize the output directory path and create the output
		# directory
		rospy.logwarn("Create an output folder")
		self.rospack = rospkg.RosPack()
		self.p = os.path.sep.join([self.rospack.get_path('timelapse_capture_camera'), 
"output", "images"])
		self.outputDir = os.path.join(self.p, datetime.now().strftime("%Y-%m-%d-%H%M"))
		os.makedirs(self.outputDir)

		# choose display
		self.display = 1

		# set the frame count to zero
		self.count = 0

		# Subscribe to the raw camera image topic
		self.imgRaw_sub = rospy.Subscriber("/cv_camera/image_raw", 
				Image, self.callback)

		# Subscribe to the camera info topic
		self.imgInfo_sub = rospy.Subscriber("/cv_camera/camera_info", 
				CameraInfo, self.getCameraInfo)

	def callback(self,data):
		# Convert the raw image to OpenCV format
		self.cvtImage(data)

		# Overlay some text onto the image display
		self.timestamp()

		# Save the Image
		self.saveImage()

		# check to see if the frame should be displayed to our screen
		if self.display:
			# Refresh the image on the screen
			self.displayImg()

	# Get the width and height of the image
	def getCameraInfo(self, msg):
		self.image_width = msg.width
		self.image_height = msg.height

	# Convert the raw image to OpenCV format
	def cvtImage(self, data):
		try:
			# Convert the raw image to OpenCV format """
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# TODO:
			#self.cv_image = imutils.rotate(self.cv_image, angle=180)
			self.cv_image = cv2.flip(self.cv_image,1)
			self.cv_image_copy = self.cv_image.copy()

		except CvBridgeError as e:
			print(e)

	# Overlay some text onto the image display
	def timestamp(self):
		self.ts = datetime.now().strftime("%A %d %B %Y %I:%M:%S%p")
		cv2.putText(self.cv_image, self.ts, (10, self.cv_image.shape[0] - 10), 
			cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

	# Refresh the image on the screen
	def displayImg(self):
		cv2.imshow(self.cv_window_name, self.cv_image)
		cv2.waitKey(1)

	# write the current frame to output directory
	def saveImage(self):
		self.filename = "{}.jpg".format(str(self.count).zfill(16))
		cv2.imwrite(os.path.join(self.outputDir, self.filename), self.cv_image)

		# increment the frame count and sleep for specified number of
		# seconds
		self.count += 1
		time.sleep(1)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("TimeLapse_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

def main(args):
	vn = TimeLapse_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("TimeLapse_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("TimeLapse_node [ONLINE]...")
	main(sys.argv)
