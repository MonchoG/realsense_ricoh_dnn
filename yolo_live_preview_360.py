import requests
import json
from requests.auth import HTTPDigestAuth
import time
import cv2
import numpy as np
from detectors.yolo_detector.yolo import Yolo



yolo = Yolo(confidence_param=0.3, thresh_param=0.5)


url = "".join(("http://192.168.1.1:80/osc/commands/execute"))
body = json.dumps({"name": "camera.getLivePreview"})
try:
    response = requests.post(url, data=body, headers={
        'content-type': 'application/json'}, auth=HTTPDigestAuth('THETAYL00160236', '00160236'), stream=True, timeout=5)
    print("Preview posted; checking response")
    if response.status_code == 200:
        bytes = ''
        jpg = ''
        i = 0
        for block in response.iter_content(chunk_size=10000):

            if (bytes == ''):
                bytes = block
            else:
                bytes = bytes + block

            # Search the current block of bytes for the jpq start and end
            a = bytes.find(b'\xff\xd8')
            b = bytes.find(b'\xff\xd9')

            # If you have a jpg
            if a != - 1 and b != -1:
                image = bytes[a:b + 2]
                bytes = bytes[b + 2:]
                i = cv2.imdecode(np.frombuffer(
                    image, dtype=np.uint8), cv2.IMREAD_ANYCOLOR)

                detection = yolo.detect(i)
                color_image = yolo.draw_results_no_depth(detection, i)
                cv2.imshow('i', color_image)
                if cv2.waitKey(1) == 27:
                    exit(0)
    else:
        print("theta response.status_code _preview: {0}".format(
            response.status_code))
        response.close()
except Exception as err:
    print("theta error _preview: {0}".format(err))
