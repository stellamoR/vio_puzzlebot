import os
import numpy as np 
import cv2
import sys
from pathlib import Path

parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir)) # add to sys path for local raspi cam import

from raspi_cam import read_camera_matrix_from_file, read_distortion_from_file
np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

from visual_odometry import PinholeCamera, VisualOdometry

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

vo = VisualOdometry(cam)

traj = np.zeros((600,600,3), dtype=np.uint8)

data_path = './data/recordings/series29/'
img_filenames = [filename for filename in os.listdir(data_path) if filename.endswith(".png")]
img_filenames = sorted(img_filenames, key= lambda fn: int(fn[:-4]))

draw_scale = 6
for img_id, filename in enumerate(img_filenames[:]):

	img_path = data_path + filename
	print(img_path)
	img = cv2.imread(img_path, 0)
	

	vo.update(img, img_id)

	cur_t = vo.cur_t
	if(img_id > 2):
		x, y, z = cur_t[0]*draw_scale, cur_t[1]*draw_scale, cur_t[2]*draw_scale
	else:
		x, y, z = 0., 0., 0.
	draw_x, draw_y = int(x)+290, int(z)+290
	

	cv2.circle(traj, (draw_x,draw_y), 1, (img_id*255/len(img_filenames),255-img_id*255/len(img_filenames),0), 1)
	cv2.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
	text = "Coordinates: x=%2f y=%2f z=%2f"%(x,y,z)
	cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

	cv2.imshow('Front Camera', img)
	cv2.imshow('Trajectory', traj)
	cv2.waitKey(1)

cv2.imwrite('mymap.png', traj)
