import cv2
import os
import sys
import numpy as np
import threading
import time

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


if __name__ == "__main__":
    from inertial_data_threaded import imu_process
    import queue


    def get_next_directory_name():
        items = os.listdir('data/recordings/')
        series_numbers = []
        for item in items:
            if item.startswith("series") and len(item) > 6:
                number = int(item[6:])
                series_numbers.append(number)
        next_series_number = max(series_numbers) + 1
        return f'data/recordings/series{next_series_number}/'
            

    exception_queue = exception_queue = queue.Queue()
    imu_dict = {'yaw': 0.0, 'get_pos': False, 'delta_pos': 0.0, 'stop' : False}
    l1 = threading.Lock()
    l2 = threading.Lock()
    c1 = threading.Condition()

    imu_thread = threading.Thread(target=imu_process, args=[imu_dict, l1, l2, c1, exception_queue])
    imu_thread.start()

    
    imu_timestamps = []
    video_timestamps = []


    try:
        img_id = 0

        print(get_next_directory_name())
        record_dir = get_next_directory_name()
        os.makedirs(record_dir, exist_ok = True)

        print(gstreamer_pipeline())
        video_capture = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        
        print("width, height")
        print(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        print(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        
        l2.acquire_lock() # wait for end of calibration
        if video_capture.isOpened():   
            try:
                while True:

                    # CAMERA FEED
                    ret_val, frame = video_capture.read()
                    video_timestamps.append(time.time())
                    if ret_val:
                    
                        
                        print(np.array(frame).shape)
                        
                        grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        cv2.imwrite(record_dir + f'{img_id:04d}.png', grayscale)
                            
                        if img_id == 300:
                            print("Ended Recording")
                            break
                        img_id +=1

                        # IMU DATA
                        with c1:
                            try:
                                l1.acquire()
                                imu_dict['get_pos'] = True
                            except Exception as e:
                                print("An error occurred:", e)  
                            finally:
                                l1.release()
                            c1.wait() # wait until imu thread wrote data into dict
                            #print("delta position " + str(imu_dict['delta_pos']))
                            #print("yaw " + str(imu_dict['yaw']))
                            imu_timestamps.append([time.time(), imu_dict['delta_pos'], imu_dict['yaw']])
                            print(imu_dict)

                            if not exception_queue.empty(): # raise exceptions thrown in other thread
                                exception = exception_queue.get()
                                raise exception
            except Exception as e:
                print('An Error occurred: ',e)
            finally:
                video_capture.release()
                cv2.destroyAllWindows()
        else:
            print("Error: Unable to open camera")
        
            
    finally:
        imu_dict['stop'] = True
        imu_thread.join()
        imu_path = record_dir +'imu_data.txt'
        with open(imu_path, 'w') as f:
            for timestamp in imu_timestamps:
                f.write(f"{timestamp[0]}, {timestamp[1]}, {timestamp[2]}\n")

        with open(record_dir + 'image_timestamps.txt', 'w') as f:
            for timestamp in video_timestamps:
                f.write(f"{timestamp}\n")

    
