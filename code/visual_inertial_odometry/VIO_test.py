import os
import numpy as np 
np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)
import cv2
import sys
from pathlib import Path


parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir)) # add to sys path for local raspi cam import
from raspi_cam import  read_camera_matrix_from_file, read_distortion_from_file

from visual_inertial_odometry import PinholeCamera, VisualInertialOdometry

cam_matrix = read_camera_matrix_from_file()
distortion = read_distortion_from_file()

cam = PinholeCamera( 
    width=1280.0,
    height=720.0,
    fx=cam_matrix[0,0],
    fy=cam_matrix[1,1],
    cx=cam_matrix[0,2],
    cy=cam_matrix[1,2],
    k1=distortion[0],
    k2=distortion[1],
    p1=distortion[2],
    p2=distortion[3],
    k3=distortion[4])


# parameters:
# change weight_imu to 1 for only yaw
# change y_mov_penalty_weight to 1 if you do not want side translation to be penalized.
vio = VisualInertialOdometry(cam, weight_imu_heading= 0.8, y_mov_penalty_weight=0.3)

traj = np.zeros((600,600,3), dtype=np.uint8)


#Change Paths as needed and try all the options
data_path = './data/recordings/series26/' # Rechtskurve 
data_path = './data/recordings/series27/' # erst Schlenker nach Links, dann Rechtskurve
data_path = './data/recordings/series28/' # Linkskurve
data_path = './data/recordings/series29/' # Vorwärts, dann 90 Grad Links dann etwas Fahren, dann wieder 90 Grad Links
data_path = './data/recordings/series28/'

imu_data = []
with open(data_path + 'imu_data.txt', 'r') as f:
    for line in f:
        parts = line.strip().split(',')
        time_part = float(parts[0])
        delta_pos_part = float(parts[1])
        yaw_imu = float(parts[2])
        imu_data.append([time_part, delta_pos_part, yaw_imu])
        
imu_distances = np.array(imu_data)[:,1]
imu_yaws = np.array(imu_data)[:,2]

# moving average over imu_distances to smoothe false readings. Overall distance traveled stays the same.
window_length = 4
imu_distances = np.pad(imu_distances, (window_length,0), 'constant', constant_values = (0,0)) # prepend window_length zeros for moving avg
moving_average = np.convolve(imu_distances, np.ones(window_length)/window_length, mode='valid')


img_filenames = [filename for filename in os.listdir(data_path) if filename.endswith(".png")]
img_filenames = sorted(img_filenames, key= lambda fn: int(fn[:-4]))

imu_timing_offset = 12 # from timing notebook

real_life_scale = 1 
draw_scale = 3
for img_id, filename in enumerate(img_filenames):

    # Reading Image
    img_path = data_path + filename
    img = cv2.imread(img_path, 0)
    
    # actual Update with imu-offset from timing notebook
    vio.update(img, moving_average[max(0,img_id -imu_timing_offset)], imu_yaws[max(0,img_id-imu_timing_offset)])

    #################
    # Visualization
    #################

    cur_t = vio.cur_t
    if(img_id > 2):
        x, y =  cur_t[0]*draw_scale, cur_t[1]*draw_scale
    else:
        x, y = 0., 0.
    
    draw_x, draw_y = int(y)+290, int(x)+290
    cv2.circle(traj, (draw_x,draw_y), 1, (img_id*255/len(img_filenames),255-img_id*255/len(img_filenames),0), 1)
    cv2.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
    text = f"Coordinates: (x= {cur_t[0]*real_life_scale:06.2f}cm y= {cur_t[1]*real_life_scale:06.2f}cm),"
    text += f" Yaw: {vio.yaw_filter.old_yaw:06.2f}deg"
    #text += f" Yaw: {vio.yaw_handler.old_yaw:06.2f}deg"
    cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

    cv2.putText(img, f"IMU-delta_pos = {moving_average[max(0,img_id -imu_timing_offset)]}",  (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)
    cv2.imshow('Front Camera', img)
    cv2.imshow('Trajectory', traj)
    cv2.waitKey(1)
    print(filename)

cv2.imwrite('vio_output.png', traj)

