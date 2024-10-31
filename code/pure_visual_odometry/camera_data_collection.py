# MIT License
# Copyright (c) 2019-2022 JetsonHacks extended by Robin Sternberg

import cv2
import sys
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


def show_camera():
    headless = False
    if len(sys.argv) >1:
        for arg in sys.argv[1:]:
            if arg == '-headless':
                headless = True
            else:
                print(f"Unknown argument: {arg}")

    window_title = "Raspi Camera V2"
    img_id = 0
    record = False
    last_key = ord(' ')
    record_dir = './data/images/series1/'


    print(gstreamer_pipeline())
    video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    
    print("width, height")
    print(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    print(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    if video_capture.isOpened():   
        try:
            if not headless:
                window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()

                if not headless:
                    # Check to see if the user closed the window
                    # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                    # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                    if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                        cv2.imshow(window_title, frame)
                        if record:
                            grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                            cv2.imwrite(record_dir + f'{img_id:04d}.png', grayscale)
                            img_id +=1
                    else:
                        break 
                    keyCode = cv2.waitKey(10) & 0xFF
                    
                    if keyCode==ord('r') and last_key != ord('r'): # Flankengesteuert
                        if record:
                            print(f"Starting Recording. Output in {record_dir}")
                            record = True
                        else:
                            print(f"Stopping Recording")
                            record = False
                    if keyCode == 27 or keyCode == ord('q'):
                        break
                    last_key = keyCode
                elif ret_val:
                    if img_id == 100:
                        print("Started Recording")
                    if img_id > 100 and img_id < 300:
                        print(np.array(frame).shape)
                        
                        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        cv2.imwrite(record_dir + f'{img_id:04d}.png', grayscale)
                        
                    if img_id == 300:
                        print("Ended Recording")
                        break
                    img_id +=1
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    show_camera()
