#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from seawolf_8.msg import StoreImage
from seawolf_8.srv import PostFrame, GetFrame, GetFrameNames



class VideoStore:
  def __init__(self):
    # map of stream name to image, images are in ROS image format, not opencv format
    self.streams = {}
    # used to convert between opencv images and ros images
    self.bridge = CvBridge()
    
  def getFrame(self, req):
    rospy.logdebug('Getting frame %s', req.name)
    for stream in self.streams:
      print(stream)
    if req.name not in self.streams:
      return None, True, 'Error: Name: ' + req.name + ' not in video-store'
    return self.streams[req.name], False, ''

  def postFrame(self, req):
    rospy.logdebug('Posting frame %s', req.name)
    self.streams[req.name] = req.image
    # return no error and no error message
    return False, ''
  
  def getFrameNames(self, req):
    rospy.logdebug('Getting frame names')
    return ','.join(self.streams.keys())

def main(args):
  rospy.init_node('video_store', anonymous=True, log_level=rospy.DEBUG)
  try:
    print('starting')
    store = VideoStore()
    getFrame = rospy.Service('get_frame', GetFrame, store.getFrame)
    postFrame = rospy.Service('post_frame', PostFrame, store.postFrame)
    getFrameNames = rospy.Service('get_frame_names', GetFrameNames, store.getFrameNames)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)