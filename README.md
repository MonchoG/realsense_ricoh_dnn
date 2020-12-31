# Object detection with Realsense and Ricoh theta V
Opencv inferencing pipeline using Ricoh Theta V 360 camera device and Realsense d435i device for obstacle and object detection.

The project requires OpenCV build from sources, since it requires the DNN module. The pipeline can be used both with CPU and GPU. Any version of opencv that has the DNN module should satisfy - in my setup i used opencv 4.4.0.
Requires the Realsense SDK - tested on 2.40.0 on Windows 10/Ubuntu 18.04

# On Jetson Nano
The setup was tested on Jetson Nano
Opencv and Realsense sdk were installed from:
  - https://github.com/jetsonhacks/installRealSenseSDK - Build script for realsense SDK. 
    - Install using the buildLibRealsense script
    - Check if the Librealsense version satisfies
    - Check if the NVCC path is correct -  Jetpack 4.4 (the jetson nano os) came with cuda 10.2 installed so i had to update the path.
    - Add this CMAKE flag  -D PYTHON3_EXECUTABLE=path/to/python3 (can give =$(which python3) to find path to python, if not known)
- https://github.com/JetsonHacksNano/buildOpenCV - Build script for opencv
  - Check if Opencv version satisfies - I updated to 4.4.0
  - Add this CMAKE flag  -D PYTHON3_EXECUTABLE=path/to/python3 (can give =$(which python3) to find path to python, if not known)


# Contents
## Camera drivers
### realsense
Wrapper class for the Realsense D435i depth camera device. Can start pipeline which provides RGB, depth,infrared, acceleration and/or gyroscope readings. (On linux IMU requires extra packages with instalation of SDK check Realsense building from sources..)
### ricoh_theta
Client to communicate with Ricoh Theta V REST api. Can control the camera settings, start/stop recording, transfer images and videos. Uses python requests module.
## Detectors
All of the detectors use Opencv DNN and are similar. The difference between the 2 is the framework that is used to read the weights and model from(Darknet and Tensorflow). If depth data and parameters are provided, it is possible compute distance to the detections if enough data is available
### mask rcnn
- Requires model tensorflow pretrained model and weigts - configuration(.pbtxt), weights(frozen inference graph -  .pb) and labels paths. Can update the default ones in the class file to use without passing those params, or can pass the paths manually
  - Pretrained model on COCO can be found at : 
### yolo detector
- Requires model darkent pretrained model and weights - configuration(.cfg), weights(.weights) and labels paths. Can update the default ones in the class file to use without passing those params, or can pass the paths manually
  - Pretrained model on COCO can be found at : 


# Installation on Nano
1. sudo update upgrade...
2. Check swap....
3. Clone opencv repo
4. Make changes to opencv build script... paths, cuda flags, python flags in make
5. Build Install wait..... 
6. Test if all ok - python -> import cv2 -> buildInformation -> cuda ??
7. Clone realsense repo
8. MNake changes to build script ... paths, cuda, python flag...
9. Build install wait
10. Test with the realsense sdk app...
11. Test in python...
12. Testing with Yolo
    1. get pretrained model - repo...
    2. put them detectors/yolo_detector/weights
    3. run yolo_od - detects object, labels it and measures distance to it..
       1. There are few distance measurements methods, the one that is put as default showed the most promising result
13. Testing with mask rcnn - same steps;
    1.   get model and config from: ....
    2.   put them in detectors/mask_rcnn/weights
    3.   run mrcnn_od.py - detects object, labels, masks it and measures distance to it

# Testing 360 camera
1. Open thetav.py
2. Update camera id and password
3. Make sure Client machine (nano/pc) is connected to Ricoh device AP
4. Run theta.py 
   1. It will print out device information, options, and start a 5 second recording or continous shoot (depending on imaging mode of device)

# Testing 360 camera and D435i at the same time:
1. Open main.py - It requires Yolo weights; Can be changed to Mask RCNN as well just instantiate Mask rcnn detector instead of yolo...
2. Change theta device authentication parameters
3. Run main.py 
   - Streams from D435i will start
   - 360 device will start recording
   - Output from object detection on the RGB and Depth streams from D435i
   - When output window is closed with 'q' or 'esc' 360 device will stop recording and download the recorded video to the working directory
  