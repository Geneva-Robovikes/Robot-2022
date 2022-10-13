from cscore import CameraServer
from networktables import NetworkTables

import cv2
import json
import numpy as np
import time


def main():
   NetworkTables.startClientTeam(3067)

   with open('/boot/frc.json') as f:
      config = json.load(f)
   camera = config['cameras'][0]

   width = camera['width']
   height = camera['height']

   inst = CameraServer.getInstance()
   inst.startAutomaticCapture()

   input_stream = inst.getVideo()
   output_stream = inst.putVideo('Processed', width, height)

   # Table for vision output information
   vision_nt = NetworkTables.getDefault().getTable('SmartDashboard')

   # Allocating new images is very expensive, always try to preallocate
   img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

   # Wait for NetworkTables to start
   time.sleep(0.5)

   while True:
      start_time = time.time()

      frame_time, input_img = input_stream.grabFrame(img)
      output_img = np.copy(input_img)

      # Notify output of error and skip iteration
      if frame_time == 0:
         output_stream.notifyError(input_stream.getError())
         continue

      # Convert to HSV and threshold image
      hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
      binary_img = cv2.inRange(hsv_img, (60, 65, 200), (100, 255, 255))

      contour_list, _ = cv2.findContours(binary_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
      print(contour_list)

      xpos = 80
      ypos = 60

      for contour in contour_list:

         # Ignore small contours that could be because of noise/bad thresholding
         if cv2.contourArea(contour) < 15:
            continue

         cv2.drawContours(output_img, contour, -1, color = (255, 255, 255), thickness = -1)

         rect = cv2.minAreaRect(contour)
         center, size, angle = rect
         center = tuple([int(dim) for dim in center]) # Convert to int so we can draw

         # Draw rectangle and circle
         cv2.drawContours(output_img, [cv2.boxPoints(rect).astype(int)], -1, color = (0, 0, 255), thickness = 2)
         cv2.circle(output_img, center = center, radius = 3, color = (0, 0, 255), thickness = -1)

         xpos = center[0]
         ypos = center[1]

      vision_nt.putNumber('X', xpos)
      vision_nt.putNumber('Y', ypos)

      processing_time = time.time() - start_time
      fps = 1 / processing_time
      cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
      output_stream.putFrame(output_img)

main()