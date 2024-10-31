import numpy as np
import serial
import struct
import time
from scipy.spatial.transform import Rotation as Rot
import os
from collections import deque
import threading


# stop numpy from using scientific notation for print statements
np.set_printoptions(precision=2)
np.set_printoptions(suppress=True)


def filter_accel (reading):

    threshold = 0.01
    # zero out each axis individually if its movement is below the threshold
    reading[0] = 0 if abs(reading[0]) < threshold else reading[0]
    reading[1] = 0 if abs(reading[1]) < threshold else reading[1]
    reading[2] = 0 if abs(reading[2] - 1) < threshold else reading[2] - 1  # assuming acc[2] is adjusted for gravity

    return reading

class VelocityFilter:
    def __init__(self, window_size=10, threshold=0.0001): # Todo
        self.window_size = window_size
        self.threshold = threshold
        self.buffer = np.zeros((window_size, 2))
        self.index = 0
        self.count = 0

    def update(self, vel, dv):

        self.buffer[self.index] = dv[:2] # do not look at gravity direction

        self.index = (self.index + 1) % self.window_size
        self.count = min(self.count + 1, self.window_size)

        avg_dv = np.mean(self.buffer[:self.count], axis=0)

        if np.all(np.abs(avg_dv) < self.threshold):
            return np.zeros(3)
        else:
            return vel

class YawKalmanFilter:
    def __init__(self, dt, A, B, H, R, Q, P):
        self.dt = dt
        self.A = A
        self.B = B
        self.H = H
        self.R = R
        self.Q = Q
        self.P = P
        self.x_est = np.zeros((2,))  # Initial state estimate

    def update_matrices(self, dt):
        self.dt = dt
        self.A = np.array([[1, dt], [0, 1]])  # state transition matrix
        self.B = np.array([0, dt])            # control input matrix

    def kalman_filter_cutoff(self, yaw_am, gyro_z):
        # ignore angle wraparound
        if abs(yaw_am) > 165:
            return np.array([yaw_am, gyro_z]), self.P

        self.update_matrices(self.dt)

        x_pred = np.dot(self.A, self.x_est) + self.B * gyro_z
        P_pred = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        # kalman gain
        S = np.dot(np.dot(self.H, P_pred), self.H.T) + self.R
        K = np.dot(np.dot(P_pred, self.H.T), np.linalg.inv(S))

        # update
        y = yaw_am - np.dot(self.H, x_pred)  # measurement residual

        self.x_est = x_pred + np.dot(K, y)
        self.P = np.dot((np.eye(2) - np.dot(K, self.H)), P_pred)

        return self.x_est, self.P

def imu_process(imu_dict, l1, l2, c1, exception_queue):
    try:
        l2.acquire() 
        
        to_cm = 9.81*100
        dt = 0.001
        A = np.array([[1, dt], [0, 1]])
        B = np.array([0, dt])
        H = np.array([[1, 0]])
        R = np.array([[10]])
        Q = np.eye(2) * 0.01
        P = np.eye(2)
        x_est = np.array([0, 0])
        list_for_cal = []
        list_for_avg = deque(maxlen=10) 


        vel = np.array([0.0,0.0,0.0])
        last_position = np.array([0.0,0.0])
       
        vel_filter = VelocityFilter()
        yaw_kalman = YawKalmanFilter(dt=0.001,
                  A=np.array([[1, 0.001], [0, 1]]),
                  B=np.array([0, 0.001]),
                  H=np.array([[1, 0]]),
                  R=np.array([[10]]),
                  Q=np.eye(2) * 0.01,
                  P=np.eye(2))

        serial_port = 'COM7' # windows
        serial_port = '/dev/ttyACM0' # linux on Puzzlebot
        with serial.Serial(serial_port, 9600) as arduino_ser:

            print(f"Starting Calibration. Do not Move the IMU.")
            for _ in range(500):
                data = arduino_ser.read(36)
                curr_reading = np.array(struct.unpack('fffffffff', data))
                list_for_cal.append(curr_reading)

                # also start kalman filter so that yaw doesn't start at 0
                m = curr_reading[6:9]
                m_unit = m / np.linalg.norm(m)
                D = np.array([0, 0, -1]) # assume upright position always (just gravity vector as accel)
                E = np.cross(D, m_unit)
                N = np.cross(E, D)
                N /= np.linalg.norm(N)
        
                C = np.vstack((N, E, D))
                rot = Rot.from_matrix(C)
                euler = rot.as_euler('zyx', degrees=True)
                yaw_am = euler[0]
                gyro_z = curr_reading[5]
                
                yaw_kalman.update_matrices(0.001)
                x_est, P = yaw_kalman.kalman_filter_cutoff(yaw_am, gyro_z)
                time.sleep(0.001)


            mean_calib = np.zeros(9, dtype = float)
            mean_calib[:6] = np.mean(list_for_cal, axis = 0)[:6]
            mean_calib[2] -=1 # subtract gravity 
            mean_calib
            print(f"End of Calibration.")
            l2.release()

            for _ in range(10):
                data = arduino_ser.read(36)
                curr_reading = np.array(struct.unpack('fffffffff', data))
                list_for_avg.append(curr_reading)
        
                mov_avg = np.mean(list_for_avg, axis=0)
            
            last_time = time.time()
            ticks = 0

            print("Starting IMU loop")
            while True:
                
                data = arduino_ser.read(36)
                curr_reading = np.array(struct.unpack('fffffffff', data))
                curr_reading -= mean_calib

                curr_time = time.time()
                delta_t = curr_time -last_time
                last_time = curr_time

                A[0,1] = delta_t
                B[1] = delta_t

                old_reading = list_for_avg.popleft()
                list_for_avg.append(curr_reading)
                mov_avg += (curr_reading - old_reading) / 10 # neat formula for mean filter I got from the fwpf "technical computing"
                

                #################
                # YAW
                #################
                # yaw calculation using moving average data and cross product with unit gravity vector (known at all times because puzzlebot is upright)
                m = mov_avg[6:9]
                m_unit = m / np.linalg.norm(m)
                D = np.array([0, 0, -1]) # assume upright position always (just gravity vector as accel)
                E = np.cross(D, m_unit)
                N = np.cross(E, D)
                N /= np.linalg.norm(N)
                
                C = np.vstack((N, E, D))
                rot = Rot.from_matrix(C)
                euler = rot.as_euler('zyx', degrees=True)
                yaw_am = euler[0]
                gyro_z = mov_avg[5]
                
                yaw_kalman.update_matrices(delta_t)
                x_est, P = yaw_kalman.kalman_filter_cutoff(yaw_am, gyro_z)
                
                
                ###################
                # Delta Position
                ###################

                acc = mov_avg[:3].copy()
                #acc[2] -=1
                #acc = acc_filter.update(acc)

                acc = filter_accel(acc)

                delta_vel = acc *delta_t
                vel += delta_vel


                vel = vel_filter.update(vel, delta_vel)
                

                position = last_position + vel[:2] * delta_t
                last_position = position.copy()

                # access shared dictionary
                try:
                    l1.acquire()
                    imu_dict['yaw'] = x_est[0]
                    if imu_dict['get_pos']:
                        with c1:
                            last_position = 0.0,0.0 # reset for next measurement
                            delta_pos = np.linalg.norm(position*to_cm) # get absolute position change, regardless of direction
                            imu_dict['get_pos'] = False
                            imu_dict['pos'] = position*to_cm
                            imu_dict['delta_pos'] = delta_pos
                            c1.notify()

                except Exception as e:
                    raise e
                finally:
                    l1.release()
                if imu_dict['stop']:
                    return
                
                ticks +=1
                time.sleep(0.001)

    except Exception as e:
        print(e)
        exception_queue.put(e)


if __name__ == "__main__":
    import queue
    exception_queue = exception_queue = queue.Queue()
    imu_dict = {'yaw': 0.0, 'get_pos': False, 'delta_pos': 0.0, 'stop' : False}
    l1 = threading.Lock()
    l2 = threading.Lock()
    c1 = threading.Condition()

    imu_thread = threading.Thread(target=imu_process, args=[imu_dict, l1, l2, c1, exception_queue])
    imu_thread.start()
    time.sleep(1)
    l2.acquire_lock() # wait for end of calibration

    timestamps = []

    try:
        for i in range(1000):
            with c1:
                time.sleep(1/30)
                l1.acquire()
                try:
                    imu_dict['get_pos'] = True
                except Exception as e:
                    print("An error occurred:", e)  
                finally:
                    l1.release()
                c1.wait() # wait until imu thread wrote delta pos into dict
                timestamps.append([imu_dict['delta_pos'], imu_dict['yaw'], imu_dict['pos']])
                #print(imu_dict)
                print(imu_dict['delta_pos'])

                if not exception_queue.empty(): # raise exceptions thrown in other thread
                    exception = exception_queue.get()
                    raise exception

            
    finally:
        imu_dict['stop'] = True
        imu_thread.join()

        imu_dict['stop'] = True
        imu_thread.join()
        path = 'data/timestamps/test1.txt'
        with open('timestamps.txt', 'w') as f:
            for timestamp in timestamps:
                f.write(f"{timestamp[0]}, {timestamp[1]}, {timestamp[2][0]}, {timestamp[2][1]}\n")

