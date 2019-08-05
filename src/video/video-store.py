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
from seawolf_8.srv import PostFrame, GetFrame



class VideoStore:
  def __init__(self):
    # map of stream name to image, images are in ROS image format, not opencv format
    self.streams = {}
    # used to convert between opencv images and ros images
    self.bridge = CvBridge()
    #self.subscriber = rospy.Subscriber("frame_post",StoreImage,self.callback)
    # handle requests for updating and fetching frames
    
  def getFrame(self, req):
    if req.name not in self.streams:
      print('NOT HERE')
      return None, True, 'Error: Name: ' + req.name + ' not in video-store'
    return self.streams[req.name], False, ''

  def postFrame(self, req):
    self.streams[req.name] = req.image
    # return no error and no error message
    return False, ''

  # todo scrap this
  def callback(self, store_image):
    try:
      ros_image = store_image.image
      cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('video_store', anonymous=True)
  try:
    print('starting')
    store = VideoStore()
    getFrame = rospy.Service('get_frame', GetFrame, store.getFrame)
    postFrame = rospy.Service('post_frame', PostFrame, store.postFrame)
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)