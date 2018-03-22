import numpy as np
import cv2 as cv2
import glob as glob
from matplotlib import pyplot as plt

images = glob.glob('a.jpg')

for fname in images:
	img = cv2.imread(fname)
	plt.imshow(img, cmap = 'gray', interpolation = 'bicubic')
	plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
	plt.show()
	cv2.waitKey(0)

	cv2.destroyAllWindows()
