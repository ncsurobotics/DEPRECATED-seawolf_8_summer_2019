import cv2
import numpy as np

def displayWindows(names, images, widthLimit):
  images2DArray = []
  row = []
  rowWidth = 0
  h, w, _ = images[0].shape
  
  for img in images:
    row.append(img)
    rowWidth += w
    lastRowAdded = False
    if rowWidth >= widthLimit:
      images2DArray.append(row)
      row = []
      rowWidth = 0
      lastRowAdded = True
  if not lastRowAdded:
    images2DArray.append(row)
  height = len(images2DArray) * h
  width = len(images2DArray[0]) * w
  windowImage = np.zeros((height,width,3), np.uint8)
  for i in range(len(images2DArray)):
    row = images2DArray[i]
    for j in range(len(row)):
      windowImage[h * i:h * i + h, w * j:w * j + w] = img[::]
      cv2.putText(windowImage, names[i], (w * j, h * i + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1, 8)
  cv2.imshow('Window', windowImage)
