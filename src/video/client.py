"""
Client to interact with video store. Can post and fetch images stored by name.
"""

import rospy
from seawolf_8.srv import GetFrame, PostFrame
from cv_bridge import CvBridge, CvBridgeError

class VideoStoreClient:

  def __init__(self):
    self.post_frame = rospy.ServiceProxy('post_frame', PostFrame, persistent=True)
    self.get_frame = rospy.ServiceProxy('get_frame', GetFrame, persistent=True)

    # used to convert between opencv images and ros images
    self.bridge = CvBridge()

  def getFrame(self, name):
    if not isinstance(name, str):
      raise Exception('Name: ' + str(name) + ' is not of type string')
    # get the image by calling the service
    try:
      resp = self.get_frame(name)
    except rospy.service.ServiceException as e:
      raise Exception('Could not connect to get_frame service')
    if resp.error:
      print(dir(resp))
      raise Exception(resp.errorMessage)
    try:
      # convert ros image to cv image
      cv_image = self.bridge.imgmsg_to_cv2(resp.image, "bgr8")
    except Exception as e:
      raise Exception('Could not convert ROS image to cv image for ' + name) 
    return cv_image

  def postFrame(self, name, cv_image):
    if not isinstance(name, str):
      raise Exception('Name: ' + str(name) + ' is not of type string')
    try:
      # convert cv image to ros image
      ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
    except Exception as e:
      raise Exception('Could not convert cv image to ROS image for ' + name, e)
    # send out the image
    try:
      resp = self.post_frame(name, ros_image)
    except rospy.service.ServiceException as e:
      raise Exception('Could not connect to post_frame service', e)
    if resp.error:
      raise Exception(resp.errorMessage)
    return

  #def getAllFrameNames

def connect():
  return VideoStoreClient()