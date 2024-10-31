import numpy as np 
import cv2
from scipy.spatial.transform import Rotation 

kMinNumFeature = 1500

lk_params = dict(winSize  = (21, 21), 
                #maxLevel = 3,
                 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

def featureTracking(image_ref, image_cur, px_ref):
    kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)  #shape: [k,2] [k,1] [k,1]

    st = st.reshape(st.shape[0])
    kp1 = px_ref[st == 1]
    kp2 = kp2[st == 1]

    return kp1, kp2

class PinholeCamera:
    def __init__(self, width, height, fx, fy, cx, cy, 
                k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.distortion = (abs(k1) > 0.0000001)
        self.d = [k1, k2, p1, p2, k3]

def normalize_angle(angle):
    angle = angle % 360
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    return angle


class YawFusionComplementary():
    def __init__(self, weight_imu):
        self.old_yaw = 0.0
        self.weight_imu = weight_imu

    def set_offset(self, offset):
        self.offset = offset
    
    def step_deg(self, yaw_imu, rot_mat_vo):

        rot = Rotation.from_matrix(rot_mat_vo)
        euler_angles = rot.as_euler('zyx')
        euler_angles *= (180/np.pi)**2

        yaw_change_vo = euler_angles[0]
        new_yaw_vo = self.old_yaw + yaw_change_vo
        new_yaw_vo = normalize_angle(new_yaw_vo)

        yaw_imu = normalize_angle(yaw_imu - self.offset)
        diff = normalize_angle(yaw_imu - new_yaw_vo)
        fused_yaw_deg = normalize_angle(new_yaw_vo + self.weight_imu * diff)

        self.old_yaw = fused_yaw_deg
        
        return fused_yaw_deg

        
def new_position(cur_t, relative_t_update, theta, delta_p):
    # yaw from degrees to radians
    theta_rad = np.radians(theta)
    
    # rotation matrix
    R = np.array([
        [np.cos(theta_rad), -np.sin(theta_rad)],
        [np.sin(theta_rad),  np.cos(theta_rad)]
    ])

    # rotate relative translation into global translation
    global_translation = R @ relative_t_update

    
    # calculate the new position in global coordinates
    new_pos = cur_t + delta_p * global_translation
    return new_pos



class VisualInertialOdometry:
    def __init__(self, cam, weight_imu_heading, y_mov_penalty_weight):
        self.cam = cam
        self.new_frame = None
        self.last_frame = None
        self.cur_t = np.zeros(2)
        self.px_ref = None
        self.px_cur = None
        self.focal = cam.fx
        self.pp = (cam.cx, cam.cy)
        self.trueX, self.trueY, self.trueZ = 0, 0, 0
        self.detector = cv2.FastFeatureDetector_create(threshold=12, nonmaxSuppression=True)
        self.y_penalty_weight = y_mov_penalty_weight
        self.yaw_filter = YawFusionComplementary(weight_imu_heading)
        self.frame_num = 0


    def processFirstFrame(self, yaw_imu):
        self.px_ref = self.detector.detect(self.new_frame)
        self.px_ref = np.array([x.pt for x in self.px_ref], dtype=np.float32)
        self.cur_t = np.zeros(2)
        self.yaw_filter.set_offset(yaw_imu)
        self.frame_num +=1


    def processFrame(self, delta_p, yaw_imu):
        self.px_ref, self.px_cur = featureTracking(self.last_frame, self.new_frame, self.px_ref)
        E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, focal=self.focal, pp=self.pp, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        _, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, focal=self.focal, pp = self.pp)

        # t is a unit direction vector in yzx (has no scale) (that it is yzx and not xyz took longer to figure out than I would have liked
        t[1] = 0 # we are not flying, so vertical change is guaranteed to be noise
        t[0] *= self.y_penalty_weight # differential drive means that we probably move more forwards (or backward) than directly to the side (we can turn but that is more rotation and not translation directly to the side)
        t = t/np.linalg.norm(t) # scale back to unit vector
        t_2dim = np.array([t[2][0], t[0][0]])

        filtered_yaw = self.yaw_filter.step_deg(yaw_imu, R)
        if delta_p > 0.06:
            self.cur_t = new_position(self.cur_t, t_2dim, filtered_yaw, delta_p)

        if(self.px_ref.shape[0] < kMinNumFeature):
            self.px_cur = self.detector.detect(self.new_frame)
            self.px_cur = np.array([x.pt for x in self.px_cur], dtype=np.float32)
        self.px_ref = self.px_cur 
        self.frame_num +=1

    def update(self, img, delta_p, yaw_imu):
        self.new_frame = img
        if self.frame_num == 0:
            self.processFirstFrame(yaw_imu)
        else:
            self.processFrame(delta_p, yaw_imu)
        self.last_frame = self.new_frame

