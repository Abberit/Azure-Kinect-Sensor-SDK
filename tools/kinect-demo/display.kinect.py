import cv2
import numpy as np

imageWidth = 1280
imageHeight = 720

def onMouseMove(event, x, y, flag, param):
  if(event == cv2.EVENT_MOUSEMOVE):
    print(f"At ({x}, {y}) the depth is: {param[imageWidth * y + x]}")

def displayImage():
  windowTitle = "AzureKinectTest"

  bgraBytesCount = 4

  # reading files into bytes structure

  imageFilePath = "E:/camera_output/sample/2/image.720p.bin"
  imageFileSize = 1280*720*bgraBytesCount

  depthFilePath = "E:/camera_output/sample/2/depth.720p.bin"
  depthFileSize = 1280*720*2 # 2 bytes to stored depth in mm

  depthRawFilePath = "E:/camera_output/sample/2/depth.1024x1024.raw.bin"
  depthRawFileSize = 1024*1024*2 # 2 bytes to stored depth in mm

  imageBinaryFile = open(imageFilePath, "rb")
  imageBytes = imageBinaryFile.read(imageFileSize)

  depthBinaryFile = open(depthFilePath, "rb")
  depthBytes = depthBinaryFile.read(depthFileSize)

  depthRawBinaryFile = open(depthRawFilePath, "rb")
  depthRawBytes = depthRawBinaryFile.read(depthRawFileSize)

  # working with bytes

  imageNpArray = np.frombuffer(imageBytes, dtype=np.uint8)
  imageNpArray = imageNpArray.reshape((imageHeight, imageWidth, bgraBytesCount))

  img = cv2.cvtColor(imageNpArray, cv2.COLOR_BGRA2BGR)

  depthNpArray = np.frombuffer(depthBytes, dtype=np.uint16)

  depthRawNpArray = np.frombuffer(depthRawBytes, dtype=np.uint16)
  nonZeroCount = np.count_nonzero(depthRawNpArray);

  print(f"Non-zero depth values '{nonZeroCount}' out of '{1024*1024}' ({(nonZeroCount/(1024*1024))*100}%)")
  
  cv2.namedWindow(windowTitle)
  cv2.setMouseCallback(windowTitle, onMouseMove, param=depthNpArray)

  cv2.imshow(windowTitle, img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()

  return

displayImage()