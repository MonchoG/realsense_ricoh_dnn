import os
import numpy as np
import pyrealsense2 as rs
import time
import cv2

labelsPath = os.path.join(
    'detectors/mask_rcnn/weights/object_detection_classes_coco.txt')
# derive the paths to the Mask R-CNN weights and model configuration
weightsPath = os.path.join(
    'detectors/mask_rcnn/weights/frozen_inference_graph.pb')
configPath = os.path.join(
    'detectors/mask_rcnn/weights/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt')


class MRCNN:
    def __init__(self, config=configPath,
                 weights=weightsPath,
                 labels=labelsPath,
                 confidence_param=0.5, thresh_param=0.85, use_cuda=False):
        print("[INFO] loading Mask RCNN from disk...")
        self.net = cv2.dnn.readNetFromTensorflow(
            os.path.join(weights), os.path.join(config))
        if use_cuda:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.LABELS = open(os.path.join(labels)).read().strip().split("\n")
        np.random.seed(42)
        self.COLORS = np.random.randint(
            0, 255, size=(len(self.LABELS), 3), dtype="uint8")
        self.ln = self.net.getLayerNames()
        self.ln = [self.ln[i[0] - 1]
                   for i in self.net.getUnconnectedOutLayers()]
        self.confidence_param = confidence_param
        self.thresh_param = thresh_param

    def detect(self, input_image):
        blob = cv2.dnn.blobFromImage(input_image, swapRB=True, crop=False)
        self.net.setInput(blob)
        start = time.time()
        (boxes, masks) = self.net.forward(
            ["detection_out_final", "detection_masks"])
        result = (boxes, masks)
        end = time.time()
        # show timing information on Mask RCNN
        print("[INFO] Mask RCNN took {:.6f} seconds".format(end - start))
        return result

    def draw_results_no_depth(self, layerOutputs, input_image):

        return self.draw_results(layerOutputs, input_image)

    def draw_results(self, layerOutputs, input_image, aligned_depth_frame=None, depth_scale=None, verbose=None):

        boxes = layerOutputs[0]
        masks = layerOutputs[1]
        confidences = []
        classIDs = []

        for i in range(0, boxes.shape[2]):
            # extract the class ID of the detection along with the
            # confidence (i.e., probability) associated with the
            # prediction
            classID = int(boxes[0, 0, i, 1])
            confidence = boxes[0, 0, i, 2]

            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            if confidence > self.confidence_param:
                # scale the bounding box coordinates back relative to the
                # size of the frame and then compute the width and the
                # height of the bounding box
                (H, W) = input_image.shape[:2]
                box = boxes[0, 0, i, 3:7] * np.array([W, H, W, H])
                (startX, startY, endX, endY) = box.astype("int")
                boxW = endX - startX
                boxH = endY - startY

                # extract the pixel-wise segmentation for the object,
                # resize the mask such that it's the same dimensions of
                # the bounding box, and then finally threshold to create
                # a *binary* mask
                mask = masks[i, classID]
                mask = cv2.resize(mask, (boxW, boxH),
                                  interpolation=cv2.INTER_NEAREST)
                mask = (mask > self.thresh_param)
                # extract the ROI of the image but *only* extracted the
                # masked region of the ROI
                roi = input_image[startY:endY, startX:endX][mask]


                # grab the color used to visualize this particular class,
                # then create a transparent overlay by blending the color
                # with the ROI
                color = self.COLORS[classID]
                blended = ((0.4 * color) + (0.6 * roi)).astype("uint8")

                # store the blended ROI in the original frame
                input_image[startY:endY, startX:endX][mask] = blended


                # draw the bounding box of the instance on the frame
                color = [int(c) for c in color]
                
                cv2.rectangle(input_image, (startX, startY), (endX, endY),
                              color, 2)
                # Get center and draw dot
                cx = int((startX + endX)/2)  
                cy = int((startY + endY)/2)
                cv2.circle(input_image, (cx,  cy), radius=1, color = color, thickness=3)
                
                distance_mask = None
                distance_center = None
          
                if aligned_depth_frame and depth_scale:
                    depth = np.asanyarray(aligned_depth_frame.get_data())
                    # Crop depth data using mask of detection:
                    depth = depth[startY:endY, startX:endX][mask].astype(float)
                  
                    # Get data scale from the device and convert to meters
                    depth = depth * depth_scale
                    try:
                        # To compute distance to mask; This computation is more accurate
                        distance_mask, _, _, _ = cv2.mean(depth)
                        # To compute distance from center; This computation is accurate though it distance are not always 100% certain and vary
                        distance_center = aligned_depth_frame.get_distance(int(cx), int(cy))     
                    except Exception:
                        print("could not compute distance from mask")
    
                # draw a bounding box rectangle and label on the image
                if distance_mask or distance_center:
                    text = "{}: {:.4f} : Distance: center{:.4f}m |from mask {:.4f}".format(
                        self.LABELS[classID], confidence, distance_center, distance_mask)
                else:
                    # draw the predicted label and associated probability of
                    # the instance segmentation on the frame
                    text = "{}: {:.4f}".format(
                        self.LABELS[classID], confidence)
                cv2.putText(input_image, text, (startX, startY - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return input_image
#mrc = MRCNN()
