
import pyrealsense2 as rs
import numpy as np
import time
import math


def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()

    conf.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    conf.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    prof = p.start(conf)
    return p


p = initialize_camera()
accel_last = None
gyro_last = None

frame_count = 0
start_time = time.time()
frame_time = start_time

dist = 0

norm_avg = 0
norm_avgs = []

firstAccel = True

x = 0
y = 0
z = 0
newX = 0
newY = 0
newZ = 0

try:
    while True:

        imu_frames = p.wait_for_frames()

        accel = imu_frames.first_or_default(
            rs.stream.accel, rs.format.motion_xyz32f).as_motion_frame().get_motion_data()
        gyro = imu_frames.first_or_default(
            rs.stream.gyro, rs.format.motion_xyz32f).as_motion_frame().get_motion_data()

        last_time = frame_time
        frame_time = time.time()
        frame_count += 1

        norm_accel = np.sqrt(np.power(
            accel.x, 2) + np.power(accel.y, 2)+np.power(accel.z, 2))  # || gives norm
        norm_gyro = np.sqrt(
            np.power(gyro.x, 2) + np.power(gyro.y, 2)+np.power(gyro.z, 2))  # || gives norm

        # add gyro somehow...

        if len(norm_avgs) == 6000:
            norm_avg = sum(norm_avgs) / len(norm_avgs)
        if frame_count <= 6000:
            norm_avgs.append(norm_accel)
            print(len(norm_avgs))

        elif frame_count > 6000:
            # what is this math...
            newX = math.acos(accel.x / norm_accel)
            newY = math.acos(accel.y / norm_accel)
            newZ = math.acos(accel.z / norm_accel)

            # first loop
            if firstAccel:
                firstAccel = False
                x = newX
                y = newY
                z = newZ
            else:
                # update values
                last_z = z
                x = x * 0.98 + newX * 0.002
                y = y * 0.98 + newY * 0.002
                z = z * 0.98 + newZ * 0.002
                delta_t = frame_time - last_time

            if norm_avg != 0:
                curr_accel = abs(norm_accel - norm_avg)
                if curr_accel >= 0.1:
                    # when changed to * 0.25 which is 250 fps / 1000 which is 0.25 frames per ms distance on X was relatively ok, when moving only on Z axis otherwise it gets noisy
                    #dist += last_z + (z * delta_t * 0.25)/2
                    # delta_t is the actual time frame difference
                    dist += last_z + (z * delta_t * delta_t)/2
                print("Dist {} || Current acceleration: {} || Norm acceleration {} || Norm_avg acceleration {}".format(
                    dist, curr_accel, norm_accel, norm_avg))

            # print("imu frame {} in {} seconds: \n\taccel = {} norm =  {}, \n\tgyro = {} norm={}".format(str(frame_count),
            #                                                                          str(frame_time - last_time),
            #                                                                          str(accel), str(norm_accel),
            #                                                                          str(gyro), str(norm_gyro)))

finally:
    p.stop()
