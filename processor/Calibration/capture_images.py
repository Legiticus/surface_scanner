
'''
@file capture_images.py
@author Levi Smith
@brief Script for capturing calabration images
'''

import cv2
from datetime import date

print("Openning video capture...")
cap = cv2.VideoCapture(0)

print("Press s to same image and q to quit")

img_num = 0
cur_date = date.today()

while True:
	ret, frame = cap.read()

	if not ret:
		print("Failed to grab frame form camera")
		break

	cv2.imshow("Camera Feed", frame)

	key = cv2.waitKey(1) & 0xFF

	if (key == ord('s')):
		img_name = f"calibration{img_num}_{cur_date}.jpg"
		suc = cv2.imwrite(f"./images/{img_name}", frame)
		if suc:
			print(f"Saved: {img_name}")
			img_num += 1
		else: print(f"Failed to save: {img_name}")
	elif key == ord('q'):
		print(f"Captured {img_num} images")
		break

cap.release()
cv2.destroyAllWindows()

