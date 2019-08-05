import rospy
import bigWindow
import sys
# import from parent directory
sys.path.append('../')
import video
import cv2
import time

rospy.init_node('watch', anonymous=True)

videoConnection = video.connect()

while True:
  try:
    frame = videoConnection.getFrame('front')
    cv2.imshow("Image window", frame)
    cv2.waitKey(3)
  except Exception as e:
    time.sleep(1)
    pass