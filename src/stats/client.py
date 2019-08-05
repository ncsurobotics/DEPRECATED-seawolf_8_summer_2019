from types import types
import rospy
from seawolf_8.srv import SetStat, GetStat

class StatsClient:

  def __init__(self):
    self.set_stat = rospy.ServiceProxy('set_stat', SetStat, persistent=True)
    self.get_stat = rospy.ServiceProxy('get_stat', GetStat, persistent=True)

  def get(self, name):
    if not isinstance(name, str):
      raise Exception('Name: ' + str(name) + ' is not of type string')
    try:
      resp = self.get_stat(name)
    except Exception as e:
      raise Exception('Could not get stat: ' + name)
    if resp.error:
      raise Exception('Could not get stat: ' + name)
    return types[resp.type](resp.value)

  def set(self, name, value):
    if not isinstance(name, str):
      raise Exception('Name: ' + str(name) + ' is not of type string')
    value = str(value)
    try:
      resp = self.set_stat(name, value)
    except Exception as e:
      raise Exception('Could not set stat: ' + name + ' to value: ' + value)
    if resp.error:
      raise Exception('Could not set stat: ' + name + ' to value: ' + value)
    return types[resp.type](resp.value)

def connect():
  return StatsClient()