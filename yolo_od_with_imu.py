import numpy as np
import sys
import os


from detectors.yolo_detector.yolo import Yolo

import cv2
from camera_drivers.realsense.RealSense435i import RealSense435i as depth_cam

import time
import math



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
yolo = Yolo(config='detectors/yolo_detector/weights/yolo4coco/yolo4.cfg',
                 weights='detectors/yolo_detector/weights/yolo4coco/yolo4.weights',
                 labels='detectors/yolo_detector/weights/yolo-coco/coco.names', confidence_param=0.3, thresh_param=0.5, use_cuda=True)

# # yolo 4 tiny is default
#yolo = Yolo(confidence_param=0.3, thresh_param=0.5)


enable_rgb = True
enable_depth = True
enable_imu = True
device_id = None

width = 1280
height = 720
channels = 3

# set to non-zero to calculate the max frame rate using given number of frames
profile_frames = -1


dist = 0

norm_avg = 0
norm_avgs = []

firstAccel = True

x = 0
y = 0
z = 0
newX = 0
newY = 0
newZ = 0

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
        
        # maintain frame timing
        frame_count += 1
        last_time = frame_time
        frame_time = time.time()

        # IMU stuff
        norm_accel = np.sqrt(np.power(
            acceleration_x, 2) + np.power(acceleration_y, 2)+np.power(acceleration_z, 2))  # || gives norm
        # norm_gyro = np.sqrt(
        #     np.power(gyro.x, 2) + np.power(gyro.y, 2)+np.power(gyro.z, 2))  # || gives norm

        if len(norm_avgs) == 6000:
            norm_avg = sum(norm_avgs) / len(norm_avgs)
        if frame_count <= 6000:
            norm_avgs.append(norm_accel)
            print(len(norm_avgs))

        elif frame_count > 6000:
            # what is this math...
            newX = math.acos(acceleration_x / norm_accel)
            newY = math.acos(acceleration_y / norm_accel)
            newZ = math.acos(acceleration_z / norm_accel)

            # first loop
            if firstAccel:
                firstAccel = False
                x = newX
                y = newY
                z = newZ
            else:
                # update values
                last_z = z
                x = x * 0.98 + newX * 0.002
                y = y * 0.98 + newY * 0.002
                z = z * 0.98 + newZ * 0.002
                delta_t = frame_time - last_time

            if norm_avg != 0:
                curr_accel = abs(norm_accel - norm_avg)
                if curr_accel >= 0.1:
                    # when changed to * 0.25 which is 250 fps / 1000 which is 0.25 frames per ms distance on X was relatively ok, when moving only on Z axis otherwise it gets noisy
                    #dist += last_z + (z * delta_t * 0.25)/2
                    # delta_t is the actual time frame difference
                    dist += last_z + (z * delta_t * delta_t)/2
                print("Dist {} || Current acceleration: {} || Norm acceleration {} || Norm_avg acceleration {}".format(
                    dist, curr_accel, norm_accel, norm_avg))
                    # perform detection
                detection = yolo.detect(color_image)
                color_image = yolo.draw_results(
                    detection, color_image, depth_frame, camera.depth_scale)

        
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
