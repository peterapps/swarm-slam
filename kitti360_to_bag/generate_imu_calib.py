import os
import numpy as np

def parse_transform(line):
    T = line.split(' ')
    T = [float(x) for x in T]
    T = np.array(T).reshape((3, 4))
    return T

def print_transform(arr):
    line = arr.flatten()
    line = [str(x) for x in line]
    line = ' '.join(line)
    return line

folder = '2013_05_28'

with open(os.path.join(folder, 'calib_cam_to_pose.txt'), 'r') as f:
    line = f.readline().strip()
    line = line.replace('image_00: ', '')
    cam_to_pose = parse_transform(line)

with open(os.path.join(folder, 'calib_cam_to_velo.txt'), 'r') as f:
    line = f.readline().strip()
    cam_to_velo = parse_transform(line)

print('cam_to_pose')
print(cam_to_pose)
print('cam_to_velo')
print(cam_to_velo)

cam_to_velo = np.vstack((cam_to_velo, [0, 0, 0, 1]))
cam_to_imu = np.vstack((cam_to_pose, [0, 0, 0, 1]))
imu_to_cam = np.linalg.inv(cam_to_imu)

imu_to_velo = imu_to_cam @ cam_to_velo
print('imu_to_velo')
print(imu_to_velo)

R = imu_to_velo[:3, :3]
T = imu_to_velo[:3, 3]

with open(os.path.join(folder, 'calib_imu_to_velo.txt'), 'w') as f:
    f.write('calib_time: 28-May-2013 08:46:02\n')
    f.write('R: {}\n'.format(print_transform(R)))
    f.write('T: {}\n'.format(print_transform(T)))

velo_to_cam = np.linalg.inv(cam_to_velo)
print('velo_to_cam')
print(velo_to_cam)

R = velo_to_cam[:3, :3]
T = velo_to_cam[:3, 3]

with open(os.path.join(folder, 'calib_velo_to_cam.txt'), 'w') as f:
    f.write('calib_time: 28-May-2013 08:46:02\n')
    f.write('R: {}\n'.format(print_transform(R)))
    f.write('T: {}\n'.format(print_transform(T)))
    f.write('delta_f: 0.000000e+00 0.000000e+00\n')
    f.write('delta_c: 0.000000e+00 0.000000e+00\n')