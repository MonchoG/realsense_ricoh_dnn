import requests
import json
from requests.auth import HTTPDigestAuth
import time

# Client wrapper to access ricoh theta V through WiFi api and control device
class RicohTheta:
    def __init__(self, device_id, device_password):
        self.base_url = 'http://192.168.1.1'
        self.device_id = device_id
        self.device_password = device_password

        # set device storage prop for easy access
        device_state = self.get_device_state()
        self.storage_uri = device_state["state"]["storageUri"]
        
        # set shooting mode prop
        device_option = self.get_device_options()
        self.shooting_mode = device_option["results"]["options"]["captureMode"]

    # Base url is the ip 'http://192.168.1.1'
    # path should be /path/method for ex '/osc/info'

    def get_request(self, path):
        get = requests.get(self.base_url + path)
        return get

    # Base url is the ip 'http://192.168.1.1'
    # path should be /path/method for ex '/osc/info'
    # body is specific to the endpoint JSON dict with parameters
    def post_request(self, path, body=None):
        post = requests.post(self.base_url + path, data=body,  headers={
            'content-type': 'application/json'}, auth=HTTPDigestAuth(self.device_id, self.device_password), timeout=5)
        return post

    # Calls the "osc/info" endpoint of ricoh Api using GET
    # Acquires basic information of the camera and supported functions.
    def get_device_info(self):
        request = self.get_request("/osc/info")
        return request.json()

    # Calls the "/osc/commands/execute" endpoint of ricoh Api using post
    # Returns the following camera options : captureMode,videoStitching,iso,remainingSpace
    def get_device_options(self):
        body = json.dumps({"name": "camera.getOptions",	"parameters":
                           {"optionNames": [
                               "captureMode",
                               "_imageStitching",
                               "videoStitching",
                               "fileFormat",
                               "fileFormatSupport",
                               "previewFormat",
                               "iso",
                               "remainingSpace"]
                            }})
        request = self.post_request("/osc/commands/execute", body)
        return request.json()

    # Calls the "/osc/commands/execute" endpoint of ricoh Api using post
    # Sets the device to video mode, the parameters can be modified according to the API and need
    # Sets to 4k Mp4
    def set_device_videoMode(self):
        body = json.dumps({"name": "camera.setOptions",	"parameters": {	"options": {
            "captureMode": "video",	"sleepDelay": 1200, "offDelay": 600, "videoStitching": "ondevice",
           # "fileFormat": {"type": "mp4", "width": 3840, "height": 1920,  "_codec": "H.264/MPEG-4 AVC"},
            "_microphoneChannel": "1ch", "_gain": "mute",	"_shutterVolume": 100,
            "previewFormat": {"width": 3840, "height": 1920, "framerate": 30}}}})
        request = self.post_request("/osc/commands/execute", body)
        return request.json()

    # Calls the "/osc/commands/execute" endpoint of ricoh Api using post
    # Sets the device to video mode, the parameters can be modified according to the API and need
    def set_device_imageMode(self):
        body = json.dumps({"name": "camera.setOptions",
                           "parameters": {
                               "options": {
                                   "captureMode": "image",	"sleepDelay": 1200, "offDelay": 600, "_imageStitching": "dynamicAuto",
                                   # "fileFormat": {"type": "jpeg", "width": 5376, "height": 2688},
                                   "_microphoneChannel": "1ch", "_gain": "mute",	"_shutterVolume": 100,
                                   "previewFormat": {"width": 3840, "height": 1920, "framerate": 30}}}})
        request = self.post_request("/osc/commands/execute", body)
        return request.json()
    # Calls the "osc/state" endpoint of ricoh Api using POST
    # Acquires the camera states. Use CheckForUpdates to check whether the state object has changed its contents.
    #  Returns json object containing
    # { fingerprint : String, Takes a unique valueper current state ID.
    #  state : object, Camera state (refer to Api docs for all props of this object)}
    # For ex. device storage and battery level can be extracted from this response.

    def get_device_state(self):
        request = self.post_request("/osc/state")
        print(pretty_response(request.json()))
        return request.json()

    # Starts continuous shooting.
    # The shooting method changes according to the shooting mode (captureMode) and _mode settings.
    def start_capture(self):
        body = json.dumps({"name": "camera.startCapture"})
        request = self.post_request("/osc/commands/execute", body)
        return request.json()

    # Stops continuous shooting.
    # The output “results” is none when the shooting method is the interval shooting with limited number, the composite shooting, multi bracket shooting or time shift shooting.
    # In case of the video shooting or the limitless interval shooting, it is as below.
    # set withDownload to True if want to save the videos immediatly after posting the request
    # returns the response from the stopCapture command.
    def stop_capture(self, withDownload=False):
        body = json.dumps({"name": "camera.stopCapture"})
        request = self.post_request("/osc/commands/execute", body)
        json_response = request.json()
        print(pretty_response(json_response))
        if withDownload:
            self.download_files(json_response["results"]["fileUrls"])

        return json_response

    # Downloads the files from the links specified in files
    # files is list with download urls from Ricoh Storage
    def download_files(self, files):
        file_count = 0
        for a_file in files:
            response = requests.get(a_file, stream=True)
            # reponse returns file path link, the name of the file starts from char 67
            file_name = a_file[67:]
            with open(file_name, 'wb') as f:
                # write the contents of the response (r.content)
                # to a new file in binary mode.
                f.write(response.content)
            file_count+=1
            print("[INFO] Downloading file {}complete...".format("a_file[67:] "))
        print("[INFO] All files downloaded...")
            


# Utils
def pretty_response(response):
    return json.dumps(response, indent=2, sort_keys=True)
###


if __name__ == "__main__":

    new_cam = 'THETAYL00248307'
    new_cam_pass = '00248307'
    slw_cam = 'THETAYL00160236'
    slw_cam_pass = '00160236'

    ricoh = RicohTheta(slw_cam, slw_cam_pass)
    device_info = ricoh.get_device_info()
    print(pretty_response(device_info))

    ricoh_options = ricoh.get_device_options()
    print(pretty_response(ricoh_options))

    #ricoh.set_device_videoMode()
    ricoh_options = ricoh.get_device_options()
    print(pretty_response(ricoh_options))
    response_start = ricoh.start_capture()
    print(pretty_response(response_start))
    time.sleep(5)
    response_stop = ricoh.stop_capture(True)
    print(pretty_response(response_stop))
