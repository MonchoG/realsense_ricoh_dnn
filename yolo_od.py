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
# yolo = Yolo(config='damage_detection/yolo/weights/yolo4coco/yolo4.cfg',
#                  weights='damage_detection/yolo/weights/yolo4coco/yolo4.weights',
#                  labels='damage_detection/yolo/weights/yolo-coco/coco.names', confidence_param=0.3, thresh_param=0.5, use_cuda=True)

# # yolo 4 tiny is default
yolo = Yolo(confidence_param=0.3, thresh_param=0.5)


enable_rgb = True
enable_depth = True
enable_imu = True
device_id = None

width = 1280
height = 720
channels = 3

# set to non-zero to calculate the max frame rate using given number of frames
profile_frames = -1


bag_dir_path = os.path.join(os.path.dirname(__file__), '/output/bag/')

# set the file name recorded
if not os.path.exists(bag_dir_path):
    os.makedirs(bag_dir_path)

filename = "test3imu"
# add ext for bag file
if '.bag' not in filename:
    filename += '.bag'

write_bag_path = "output/bag/" + filename
read_bag_path = "output/bag/test2imu.bag"

try:
    camera = depth_cam(width=width, height=height, channels=channels,
                       enable_rgb=enable_rgb, enable_depth=enable_depth, enable_imu=enable_imu, record_bag=write_bag_path, read_bag=None, device_id=device_id)

    frame_count = 0
    start_time = time.time()
    frame_time = start_time

    while True:
        #
        # read data from camera
        #
        color_image, depth_image, depth_frame, acceleration_x, acceleration_y, acceleration_z, gyroscope_x, gyroscope_y, gyroscope_z = camera.run()
        # perform detection
        detection = yolo.detect(color_image)
        color_image = yolo.draw_results(
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
                color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
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
