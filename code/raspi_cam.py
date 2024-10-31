import cv2
import numpy as np
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def get_videocapture():

    return cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

def read_camera_matrix_from_file(file_path='./data/cameraMatrix.txt'):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    camera_matrix = np.zeros((3,3), dtype = float)

    for i, line in enumerate(lines):
        values = line.strip().split(',')
        camera_matrix[i] = [float(value) for value in values]

    return camera_matrix

def read_distortion_from_file(file_path='./data/cameraDistortion.txt'):
    with open(file_path, 'r') as file:
        line = file.readline()
    values = line.strip().split(',')
    distortion = np.array([float(value) for value in values])
    return distortion
