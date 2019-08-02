#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from seawolf_8.msg import StoreImage


class VideoStore:
  def __init__(self):
    # map of stream name to image
    self.streams = {}
    # used to convert between opencv images and ros images
    self.bridge = CvBridge()
    self.subscriber = rospy.Subscriber("camera",StoreImage,self.callback)
    self.subscriber = rospy.Subscriber("mission",StoreImage,self.callback)

  def handle_get(self, name):
    pass

  def handle_put(self, name, image):
    pass

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
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)