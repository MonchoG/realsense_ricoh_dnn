import numpy as np
import sys
import os
from detectors.yolo_detector.yolo import Yolo

from detectors.mask_rcnn.mrcnn import MRCNN
import cv2
from camera_drivers.realsense.RealSense435i import RealSense435i as depth_cam

import time

mask_rnn = MRCNN(use_cuda=True)


enable_rgb = True
enable_depth = True
enable_imu = False
device_id = None

width = 1280
height = 720
channels = 3

# set to non-zero to calculate the max frame rate using given number of frames
profile_frames = -1

try:
    camera = depth_cam(width=width, height=height, channels=channels,
                       enable_rgb=enable_rgb, enable_depth=enable_depth, enable_imu=enable_imu, device_id=device_id)

    frame_count = 0
    start_time = time.time()
    frame_time = start_time

    while True:
        #
        # read data from camera
        #
        color_image, depth_image, depth_frame, acceleration_x, acceleration_y, acceleration_z, gyroscope_x, gyroscope_y, gyroscope_z = camera.run()
        # perform detection
        detection = mask_rnn.detect(color_image)
        color_image = mask_rnn.draw_results(
            detection, color_image, depth_frame, camera.depth_scale)
        # maintain frame timing
        frame_count += 1
        last_time = frame_time
        frame_time = time.time()

        if enable_imu and not profile_frames:
            print("imu frame {} in {} seconds: \n\taccel = {}, \n\tgyro = {}".format(
                str(frame_count),
                str(frame_time - last_time),
                str((acceleration_x, acceleration_y, acceleration_z)),
                str((gyroscope_x, gyroscope_y, gyroscope_z))))

        if enable_rgb or enable_depth:
            if color_image is not None:
                cv2.imshow('RealSense', color_image)

            # Press esc or 'q' to close the image window
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
        if profile_frames > 0:
            if frame_count == profile_frames:
                print("Aquired {} frames in {} seconds for {} fps".format(str(frame_count), str(
                    frame_time - start_time), str(frame_count / (frame_time - start_time))))
                break
finally:
    camera.shutdown()
