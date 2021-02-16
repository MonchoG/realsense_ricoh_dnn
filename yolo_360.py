import numpy as np
import sys
import os


from detectors.yolo_detector.yolo import Yolo

import cv2
from camera_drivers.realsense.RealSense435i import RealSense435i as depth_cam

import time

# yolo 3
# yolo = Yolo(config=r'damage_detection\yolo\weights\yolo-coco\yolov3.cfg',
#                  weights=r'damage_detection\yolo\weights\yolo-coco\yolov3.weights',
#                  labels=r'damage_detection\yolo\weights\yolo-coco\coco.names',confidence_param=0.3, thresh_param=0.5)
# yolo 3 tiny add

# Yolo 4 tiny
# yolo = Yolo(config='damage_detection\yolo\weights\yolo4coco\yolo4tiny.cfg',
#                  weights='damage_detection\yolo\weights\yolo4coco\yolo4tiny.weights',
#                  labels='damage_detection\yolo\weights\yolo-coco\coco.names', confidence_param=0.3, thresh_param=0.5)

# yolo 4
# yolo = Yolo(config='detectors\yolo_detector\weights\yolo4coco\yolo4.cfg',
#             weights='detectors\yolo_detector\weights\yolo4coco\yolo4.weights',
#             labels='detectors\yolo_detector\weights\yolo-coco\coco.names', confidence_param=0.3, thresh_param=0.5, use_cuda=True)

# # yolo 4 tiny is default
yolo = Yolo(confidence_param=0.3, thresh_param=0.25)

# File path....
# cap = cv2.VideoCapture('video.mp4')
cap = cv2.VideoCapture(1)
cap.set(3, 1280)
cap.set(4, 720)
# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (1280, 720))

length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print(length)


try:

    frame_count = 0
    start_time = time.time()
    frame_time = start_time

    while cap.isOpened():  # True
        #
        # read data from camera
        #
        ret, frame = cap.read()

        # maintain frame timing
        frame_count += 1
        last_time = frame_time
        frame_time = time.time()

        frame_detection = yolo.detect(frame)
        frame_draw = yolo.draw_results_no_depth(frame_detection, frame)
        if frame_draw is not None:
            #out.write(frame_draw)
            # left = length - frame_count
            # print("wrote frame, {} left" .format(left))
            cv2.imshow('360Camera', frame_draw)
            #cv2.imwrite('{}.JPEG'.format(frame_count), frame_draw)

        # Press esc or 'q' to close the image window
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    cap.release()
    out.release()
