import pyrealsense2 as rs
import numpy as np
import cv2
import os
from detectors.yolo_detector.yolo import Yolo
yolo = Yolo(confidence_param=0.3, thresh_param=0.5)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 848, 480, rs.format.y8, 10)

# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
# config.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 30)

pipeline.start(config)

# Start streaming


try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        infrared_frame = frames.get_infrared_frame()
        infrared_image = np.asanyarray(infrared_frame.get_data())

        detection = yolo.detect(infrared_image)
        detection_imgg = yolo.draw_results_no_depth(detection, infrared_image)

        if not detection_imgg:
            continue
        # if not depth_frame:
        #     continue
        else:
            cv2.imshow('RealSense1', detection_imgg)

        # Press esc or 'q' to close the image window
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()
