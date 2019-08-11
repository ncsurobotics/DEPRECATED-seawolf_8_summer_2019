# read info from cameras and update repository contents

# should have camera select options, such as flip which camera is which
# should have update rate options

#!/usr/bin/env python
from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import sys
# import from parent directory
sys.path.append('../')
import video
import stats
import time

import numpy as np
import cv2

from seawolf_8.msg import StoreImage

# stream for each camera, (front and down)
def startCams():
  streams = {}
  streams['front'] = cv2.VideoCapture(0)
  
  # uncomment this line so to enable second camera
  #streams['down'] = cv2.VideoCapture(1)
  # delete this line, right now its just copying the front frame
  streams['down'] = streams['front']
  return streams


def publishCamsForever(streams):
  # used to send out images from the camera
  #publisher = rospy.Publisher("frame_post", StoreImage, queue_size=10)

  videoStoreConnection = video.connect()
  statsConnection = stats.connect()
  #frameRate = statsConnection.get('cameraRate')
  frameRate = 0

  while(True):
      # Capture frame
      for name in streams:
        cap = streams[name]
        ret, frame = cap.read()
        videoStoreConnection.postFrame(name, frame)
        time.sleep(frameRate)
        

def main(args):
  rospy.init_node('cameras', anonymous=True)
  try:
    print('starting')
    camStreams = startCams()
    publishCamsForever(camStreams)
  except KeyboardInterrupt:
    print("Shutting down")
  finally:
    # When everything done, release the capture
    for name in camStreams:
      camStreams[name].release()
  #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)