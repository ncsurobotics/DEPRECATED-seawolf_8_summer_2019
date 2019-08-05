"""
ROS Node for holding shared seawolf stats. For instance, attributes
like Yaw, Pitch, and Thruster Values could be stored here.
"""

#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Empty
from seawolf_8.msg import SetStats
from seawolf_8.msg import StatsInfo
from seawolf_8.srv import SetStat, GetStat


"""
All information held by StatsInfo msg. This class stores all the StatsInfo
variables in a dictionary.
"""
class Stats:
  def __init__(self):
    # currently hardwiring default values,
    # would be better to make it read from a conf file
    self.vars = {
      'yaw' : 1,
      'pitch' : 0.0,
      'name' : 'seawolf'
    }
    self.publisher = rospy.Publisher("stats_info", StatsInfo, queue_size=10)

  """
  Set stat based on SetStats msg. Converts value in message from string to type of expected value.
  """
  def setStat(self, req):
    #print(self.statsInfo)
    name, value = req.name, req.value
    currentType = type(self.vars[name])
    oldValue = self.vars[name]
    try:
      self.vars[name] = currentType(value)
    except:
      # this returns an error message
      return '', '', True
    return str(oldValue), str(type(oldValue)), False
  
  """
  Create stats info msg.
  """
  def getStatsInfoMsg(self):
    statsInfo = StatsInfo()
    statsInfo.yaw = vars['yaw']
    statsInfo.pitch = vars['pitch']
    statsInfo.name = vars['name']
  
  def getStat(self, req):
    try:
      return str(self.vars[req.name]), str(type(self.vars[req.name])), False
    except Exception as e:
      return '', '', True
    

  """
  ToString method.
  """
  def __str__(self):
    return str(self.vars)

def main(args):
  rospy.init_node('stats', anonymous=True)
  stats = Stats()
  rospy.Service('set_stat', SetStat, stats.setStat)
  rospy.Service('get_stat', GetStat, stats.getStat)
  
  try:
    print('spinning')
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)