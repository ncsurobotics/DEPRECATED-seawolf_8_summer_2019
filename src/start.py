"""
Run this to start everything. This starts all seawolf's nodes.
Some nodes are dependent on other nodes to run, so this script makes sure
that the required nodes have started up properly.
"""

import rosnode
import time
import os
from os import system, chdir
import subprocess

"""
Waits for specified node to start before returning.

node = the node that may or may not have started
timeout = how long to wait before giving up and deciding that the node won't start.
pollTime = how often the function waits before checking if the node is up.
"""
# TODO: This will fail in rare cases where one node name is a subset of the other
def hasNodeStarted(node, timeout=5, pollTime=.2):
  startTime = time.time()
  while time.time() - startTime <= timeout:
    try:
      nodes = rosnode.get_node_names()
      for node_name in nodes:
        if node in node_name:
          return True
    except rosnode.ROSNodeIOException as e:
      time.sleep(pollTime)
    else:
      time.sleep(pollTime)
  return False

"""
Runs the shell command in the background.
"""
def startBackgroundProcess(command):
  subprocess.Popen(command.split(' '), stdout=subprocess.PIPE)

try:
  startBackgroundProcess( "roscore")
except Exception as e:
  # master node may have already started (can't call roscore twice)
  pass
# check that master node has started
if not hasNodeStarted('rosout'):
  raise Exception('Could not start master node')

# start video nodes
chdir('./video')

startBackgroundProcess('python2 video-store.py')
if not hasNodeStarted('video_store'):
  raise Exception('Could not start video store node')

startBackgroundProcess('python2 cameras.py')
if not hasNodeStarted('cameras'):
  raise Exception('Could not start cameras node')

"""
# start display nodes
chdir('../display')
startBackgroundProcess('python watch.py')
if not hasNodeStarted('watch'):
  raise Exception('Could not start watch node')
"""

# start dashboard for viewing all nodes
system('rosrun node_manager_fkie node_manager')
