import client
import datetime
import sys
import cv2
import os
import rospy
import signal

"""
Program to record frames from video-store. User can specify frame update delay and streams.
Streams will default to all. Uses opencv library to create video file.

Usage:
  python record.py [streams]
Usage examples:
  python record.py front down
  python record.py
"""

def record(streams, frameRate=120.0):
  # connect to the video store
  videoStoreConnection = client.connect()

  # used in recording video
  fourcc = cv2.cv.CV_FOURCC(*'XVID')

  # used to write the video to a file
  videoWriters = []

  # name the folder the current timestamp, spaces are replaced by underscores
  folder = str(datetime.datetime.now()).replace(' ', '_')
  folderPath = 'recordings/' + folder + '/'

  # create the folder
  os.mkdir(folderPath)

  # create the video writers for each stream
  for stream in streams:
    # get initial frame to get size
    rospy.logdebug('Connecting to video store')
    image = videoStoreConnection.getFrame(stream)
    height, width, _ = image.shape
    # file path should be something like 'recordings/2019-08-24_04:12:16.234588/front'
    filepath = folderPath + stream + '.avi'
    rospy.logdebug('Stream will be saved to %s', filepath)
    
    videoWriter = cv2.VideoWriter( filepath, fourcc, frameRate, (width, height))
    videoWriters.append(videoWriter)
    
  rospy.logdebug('Recording video streams')
  # record the streams
  while True:
    for i in range(len(streams)):
      # get the current frame of the stream
      frame = videoStoreConnection.getFrame(stream)
      videoWriters[i].write(frame)

def finish(signum, arg):
  rospy.logdebug('Finished recording streams')
  exit()

if __name__ == "__main__":
  rospy.init_node('video_store', anonymous=True, log_level=rospy.DEBUG)
  # the stream names are all the args besides the first one
  streams = sys.argv[1:]

  # default to all streams if none are given as args
  if len(streams) == 0:
    videoStoreConnection = client.connect()
    streams = videoStoreConnection.getFrameNames()
  
  # exit gracefully with signal handler to catch ctrl-c
  signal.signal(signal.SIGINT, finish)

  print('Recording streams:', streams, '. To stop recording, press ctrl-c.')
  record(streams)