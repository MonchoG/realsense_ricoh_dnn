import requests
import json
from requests.auth import HTTPDigestAuth
import time
import cv2
import numpy as np

# Returns response from /osc/info ednpoint
def get_device_info():
    request = requests.get("http://192.168.1.1/osc/info")
    print(request.json())
    return request


def post_device_state():
    request = requests.post('http://192.168.1.1/osc/state')
    print(request.json())
    return request


def start_capture():
    request = requests.post(
        'http://192.168.1.1/osc/commands/execute', json={'name': 'camera.startCapture'})
    print(request.json())
    return request


def stop_capture():
    request = requests.post(
        'http://192.168.1.1/osc/commands/execute', json={'name': 'camera.stopCapture'})
    print(request.json())
    return request


def take_picture():
    request = requests.post(
        'http://192.168.1.1/osc/commands/execute', json={'name': 'camera.takePicture'})
    print(request.json())
    return request
# Can use request.json() for object


def list_files():
    request = requests.post(
        "http://192.168.1.1/osc/commands/execute", json={'name': 'camera.listFiles',
                                                         'parameters':
                                                             {"fileType": "all",
                                                              "entryCount": 10}})
    print(request.json())
    return request


def get_file(file_url="http://192.168.1.1/files/90014a68423861503e03c359b0ad5700/100RICOH/R0010010.MP4", save_path="video.mp4"):
    # send a HTTP request to the server and save
    # the HTTP response in a response object called response
    response = requests.get(file_url, stream = True)
    
    with open(save_path, 'wb') as f:
        for chunk in response.iter_content(chunk_size=1024):
            # writing one chunk at a time to video file
            if chunk:
                # write the contents of the response (r.content)
                # to a new file in binary mode.
                f.write(response.content)
        return response


def get_live_preview():
    fr_c = 0
    url = "".join(("http://192.168.1.1:80/osc/commands/execute"))
    body = json.dumps({"name": "camera.getLivePreview"})
    try:
        response = requests.post(url, data=body, headers={
                                 'content-type': 'application/json'}, auth=HTTPDigestAuth('THETAYL00248307', '00248307'), stream=True, timeout=5)
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
                    print("image - loading")
                    image = bytes[a:b + 2]
                    bytes = bytes[b + 2:]
                    i = cv2.imdecode(np.frombuffer(
                        image, dtype=np.uint8), cv2.IMREAD_ANYCOLOR)
                    cv2.imshow('i', i)
                    fr_c += 1
                    #cv2.imwrite("images/img{}.jpg".format(fr_c), i)
                    if cv2.waitKey(1) == 27:
                        exit(0)
        else:
            print("theta response.status_code _preview: {0}".format(
                response.status_code))
            response.close()
    except Exception as err:
        print("theta error _preview: {0}".format(err))


def set_options():
    url = "".join(("http://192.168.1.1:80/osc/commands/execute"))
    body = json.dumps({"name": "camera.setOptions",	"parameters": {	"options": {
        "captureMode": "video",	"sleepDelay": 1200, "offDelay": 600, "videoStitching": "ondevice",
        "_microphoneChannel": "1ch", "_gain": "mute",	"_shutterVolume": 100,
        "previewFormat": {"width": 3840, "height": 1920, "framerate": 30}}}})
    HEADERS = {'content-type': 'application/json'}
    try:
        req = requests.post(url, data=body, headers=HEADERS, auth=HTTPDigestAuth(
            'THETAYL00248307', '00248307'), timeout=3)
        print(req.json(),)
    except Exception as e:
        print("Error {}".format(e))
        pass


def get_options():
    url = "".join(("http://192.168.1.1:80/osc/commands/execute"))
    body = json.dumps({"name": "camera.getOptions",	"parameters":
                       {"optionNames": [
                           "captureMode",
                           "videoStitching",
                           "previewFormat",
                           "iso",
                           "remainingSpace"]
                        }})
    HEADERS = {'content-type': 'application/json'}
    try:
        req = requests.post(url, data=body, headers=HEADERS, auth=HTTPDigestAuth(
            'THETAYL00248307', '00248307'), timeout=3)
        print(req.json())
    except Exception as e:
        print("Error {}".format(e))
        pass


#list_files()
# get_options()
# set_options()
# get_options()

# get_device_info()
# post_device_state()
# get_live_preview()

# start_capture()
# stop_capture()
#get_file()
