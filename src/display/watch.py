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
  names = videoConnection.getFrameNames()
  frames = []
  try:
    for name in names:
      frames.append(videoConnection.getFrame(name))
  except:
    pass
  else:
    bigWindow.displayWindows(names, frames)
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break