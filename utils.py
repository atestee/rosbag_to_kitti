import math

import rospy
from tf2_py import ExtrapolationException, LookupException
import numpy as np
from ros_numpy import numpify

from dim import DIM

# Subt artifacts: https://www.subtchallenge.com/resources/SubT_Cave_Artifacts_Specification.pdf
import dim
artifacts = ['backpack', 'rescue_randy', 'helmet', 'extinguisher', 'drill']


def lookup_transforms_to_artifacts(msg, tf_buffer):
    transforms = []
    for artifact in artifacts:
        for i in range(1, 5):
            if tf_buffer.can_transform_core(msg.header.frame_id, artifact  + '_' + str(i), msg.header.stamp):
                try:
                    transforms.append(tf_buffer.lookup_transform_core(msg.header.frame_id, artifact + '_' + str(i), msg.header.stamp))
                except ExtrapolationException:
                    # print("Extrapolation exception between " + str(msg.header.frame_id) + " and " + artifact + '_' + str(i))
                    pass
                except LookupException:
                    # print("Lookup exception between " + str(msg.header.frame_id) + " and " + artifact + '_' + str(i))
                    pass

    return transforms


def save_transforms(transforms, name):
    file = open(name, 'w+')
    for tf in transforms:
        for line in str(tf).split('\n'):
            file.write(line + '\n')
        file.write('\n')
    file.close()


def save_artifact_data(name, tf_buffer):
    transforms = []

    for artifact in artifacts:
        for i in range(1, 5):
            if tf_buffer._frameExists(artifact + '_' + str(i)):
                transforms.append(tf_buffer.lookup_transform_core(artifact + '_' + str(i), 'world', rospy.Time(0)))

    file = open(name, 'w')
    for tf in transforms:
        for line in str(tf).split('\n'):
            file.write(line + '\n')
        file.write('\n')
    file.close()


def save_calib_file(name, calibration_matrices, p0, tf_buffer, stamp):
    file = open(name, "w+")
    frames = get_camera_frames()

    K1 = calibration_matrices[0]
    K2 = calibration_matrices[1]
    K3 = calibration_matrices[2]

    source = frames[0] # camera_0 is source camera
    lidar_frame = "X1/laser/laser"
    transform0 = tf_buffer.lookup_transform_core(source, source, stamp)
    transform1 = tf_buffer.lookup_transform_core(source, frames[1], stamp)
    transform2 = tf_buffer.lookup_transform_core(source, frames[2], stamp)
    transform3 = tf_buffer.lookup_transform_core(source, frames[3], stamp)

    T0 = numpify(transform0.transform)
    T1 = numpify(transform1.transform)
    T2 = numpify(transform2.transform)
    T3 = numpify(transform3.transform)

    K1 = np.array(K1).reshape(3,3)
    K2 = np.array(K2).reshape(3,3)
    K3 = np.array(K3).reshape(3,3)

    p1 = np.dot(K1, T1[:3, :])
    p2 = np.dot(K2, T2[:3, :])
    p3 = np.dot(K3, T3[:3, :])

    p0_str = str(p0)[1:-1]
    p1_str = get_matrix_as_string(p1)
    p2_str = get_matrix_as_string(p2)
    p3_str = get_matrix_as_string(p3)

    R0 = T0[:3, :3]
    r0_str = get_matrix_as_string(R0)

    velo_to_cam_transform = tf_buffer.lookup_transform_core(lidar_frame, source, rospy.Time(0))
    Tr_velo_to_cam = numpify(velo_to_cam_transform.transform)[:3, :]
    Tr_velo_to_cam_str = get_matrix_as_string(Tr_velo_to_cam)

    file.write('P0: ' + p0_str + '\n')
    file.write('P1: ' + p1_str + '\n')
    file.write('P2: ' + p2_str + '\n')
    file.write('P3: ' + p3_str + '\n')
    file.write('R0: ' + r0_str + '\n')
    file.write('Tr_velo_to_cam: ' + Tr_velo_to_cam_str + '\n')

    file.close()


def get_camera_frames():
    camera_0_frame = "X1/camera_0/camera_0_optical"
    camera_1_frame = "X1/camera_1/camera_1_optical"
    camera_2_frame = "X1/camera_2/camera_2_optical"
    camera_3_frame = "X1/camera_3/camera_3_optical"
    return [camera_0_frame, camera_1_frame, camera_2_frame, camera_3_frame]


def get_matrix_as_string(mat):
    string = ''
    m, n = np.shape(mat)
    for i in range (m):
        for j in range (n):
            string = string + str(mat[i, j]) + ' '

    return string


def get_alpha(tf):
    # get rotation-z difference between transforms from quaternion
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    q = numpify(tf.transform.rotation)
    alpha = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2]**2 + q[3]**2))
    if alpha > np.pi: alpha -= 2 * np.pi
    elif alpha < -np.pi: alpha += 2 * np.pi
    return alpha


def artifacts_in_pointcloud(pts, transforms):
    # element of bboxes is [class, alpha, height, width, depth, x, y, z]
    bboxes = []
    for tf in transforms:
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z

        max_x = np.max(pts[0])
        min_x = np.min(pts[0])

        max_y = np.max(pts[1])
        min_y = np.min(pts[1])

        max_z = np.max(pts[2])
        min_z = np.min(pts[2])

        if x <= max_x and y <= max_y and z <= max_z and x >= min_x and y >= min_y and z >= min_z:
            # print("artifact " + tf.child_frame_id + " in lidar scan!")
            alpha = get_alpha(tf)
            artifact_class = tf.child_frame_id[:-2]
            try:
                dimensions = DIM.get(artifact_class)
                bbox = (artifact_class, alpha, dimensions[0], dimensions[1], dimensions[2],
                        x + dimensions[3], y + dimensions[4], z + dimensions[5])
                bboxes.append(bbox)
            except KeyError:
                print("Artifact class unknown!")

    return bboxes


def save_bbox_data(bboxes, name):
    file = open(name, 'w+')
    for bbox in bboxes:
        file.write(bbox[0] + ' 1 1 ' + str(bbox[1]) + ' 1 1 1 1 ' + str(bbox[2]) + ' ' + str(bbox[3]) + ' ' +
        str(bbox[4]) + ' ' + str(bbox[5]) + ' ' + str(bbox[6]) + ' ' + str(bbox[7]) + ' 1\n')
    file.close()