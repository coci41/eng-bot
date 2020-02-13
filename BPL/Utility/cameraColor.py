#Utilizziamo questa classe per acquisire i frame per la calibrazione del range HSV

import cv2

class CameraImage:

	def __init__(self, device=0):

		self.dev = cv2.VideoCapture(device)
		self.__devid = device
		self.dev.set(3, 320)
		self.dev.set(4, 240)

	def setup(self):
		if not self.dev.isOpened():
			self.dev.open(self.__devid)

	def get_image(self, gray=False):
		ret, frame = self.dev.read()
		if ret:
			if gray:
				frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			return frame
		return None

