import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from seawolf_8.msg import StoreImage

def loadMissions(conf):
  return ['a1', 'b2', 'c3']

missions = loadMissions('conf')

