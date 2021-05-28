import numpy as np
from ros_numpy import numpify
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage
import tf2_ros
import cv2
import os
import logging
import sys

import utils

def slots(msg):
    """Return message attributes (slots) as list."""
    return [getattr(msg, var) for var in msg.__slots__]

def main(bag_path, out_path):
    """Converts rosbag files into KITTI format files.

    :param bag_path: directory, where bag files are stored
    :param out_path: exported data will be in directory: out_path/kitti
    :return: nothing
    """

    # BufferCore documentation in C++
    # https://docs.ros.org/en/jade/api/tf2/html/classtf2_1_1BufferCore.html#a8f3900f749ab24dd824b14c0e453d1a6
    tf_buffer = tf2_ros.BufferCore(rospy.Duration.from_sec(3600.0))

    clouds_per_bag_filename = 'clouds_per_bag.txt'
    clouds_per_bag_file = open(clouds_per_bag_filename, 'w+')
    clouds_per_bag_file.write('numer of bag, last scanned pointcloud in this bag\n')

    i_cloud = 0
    i_image = 0
    i_bag = 0
    robot_name = 'X1'
    camera_frames = utils.get_camera_frames()
    cam_info_0_saved = False
    cam_info_1_saved = False
    cam_info_2_saved = False
    cam_info_3_saved = False
    p0 = None

    # go through bag files in given directory one-by-one
    for bag_file in os.listdir(bag_path):
        bag_file = bag_path + bag_file

        logging.info("processing file: " + bag_file)

        with rosbag.Bag(bag_file, 'r') as bag:
            info = bag.get_type_and_topic_info()

            # Get tf_buffer
            for topic, msg, t in bag:
                # Use rosbag info to determine msg type as
                # type(msg) gives a rosbag-specific helper type.
                dtype = info[1][topic][0]

                if dtype == 'tf2_msgs/TFMessage':
                    logging.info('Got transform at %s' % topic)
                    msg = TFMessage(*slots(msg))
                    for tf in msg.transforms:
                        # Robot pose has to be stored as tf not tf_static
                        if tf.child_frame_id == robot_name:
                            robot_parent_name = tf.header.frame_id
                            tf_buffer.set_transform(tf, 'default_authority')
                            # logging.info('%s -> %s set' % (tf.header.frame_id, tf.child_frame_id))
                        elif topic == '/tf':
                            tf_buffer.set_transform(tf, 'default_authority')
                            # logging.info('%s -> %s set' % (tf.header.frame_id, tf.child_frame_id))
                        elif topic == '/tf_static':
                            tf_buffer.set_transform_static(tf, 'default_authority')
                            # logging.info('static %s -> %s set' % (tf.header.frame_id, tf.child_frame_id))
                elif dtype == 'sensor_msgs/CameraInfo':
                    msg = CameraInfo(*slots(msg))
                    if msg.header.frame_id == camera_frames[0] and not cam_info_0_saved:
                        p0 = msg.P
                        R0 = msg.R
                        cam_info_0_saved = True
                    elif (msg.header.frame_id == camera_frames[1]) and not cam_info_1_saved:
                        K1 = msg.K
                        cam_info_1_saved = True
                    elif (msg.header.frame_id == camera_frames[2]) and not cam_info_2_saved:
                        K2 = msg.K
                        cam_info_2_saved = True
                    elif (msg.header.frame_id == camera_frames[3]) and not cam_info_3_saved:
                        K3 = msg.K
                        cam_info_3_saved = True
            calibration_matrices = [K1, K2, K3]

        with rosbag.Bag(bag_file, 'r') as bag:
            info = bag.get_type_and_topic_info()

            # Get Image and PointCloud2 data
            for topic, msg, t in bag:
                dtype = info[1][topic][0]
                if dtype == 'sensor_msgs/PointCloud2':
                    logging.info('Got cloud at %s' % topic)
                    # Create proper ROS msg type for ros_numpy.
                    msg = PointCloud2(*slots(msg))
                    # Convert to structured numpy array.
                    cloud = numpify(msg)
                    # Convert to 3xN array.
                    pts = np.stack([cloud[f] for f in ('x', 'y', 'z')])
                    # Add some default reflectance (assuming it is not provided, otherwise use it).
                    refl = np.zeros((1, cloud.shape[0], cloud.shape[1]), pts.dtype)
                    pts = np.concatenate((pts, refl))
                    # Save transform
                    transforms = utils.lookup_transforms_to_artifacts(msg, tf_buffer)
                    if len(transforms) != 0:
                        # Save transforms
                        # Save bounding boxes
                        bboxs = utils.artifacts_in_pointcloud(pts, transforms)
                        label_filename = os.path.join(out_path, 'KITTI', 'label_2', '%06i.txt' % i_cloud)
                        utils.save_bbox_data(bboxs, label_filename)
                        # Save calibration file
                        calib_filename = os.path.join(out_path, 'KITTI', 'calib', '%06i.txt' % i_cloud)
                        utils.save_calib_file(calib_filename, calibration_matrices, p0, tf_buffer, msg.header.stamp)
                        # Save pointcloud
                        cloud_filename = os.path.join(out_path, 'KITTI', 'velodyne', '%06i.bin' % i_cloud)
                        with open(cloud_filename, 'wb') as file:
                            pts.T.tofile(file)
                        i_cloud += 1
                elif dtype == 'sensor_msgs/Image':
                    logging.info('Got image at %s' % topic)
                    # Create proper ROS msg type for ros_numpy.
                    msg = Image(*slots(msg))
                    if (msg.header.frame_id != "X1/camera_0/camera_0_optical"):
                        continue
                    # Convert to structured numpy array.
                    img = numpify(msg)
                    transforms = utils.lookup_transforms_to_artifacts(msg, tf_buffer)
                    if len(transforms) != 0:
                        # Save image
                        image_filename = os.path.join(out_path, 'KITTI', 'image_2', '%06i.png' % i_image)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        cv2.imwrite(image_filename, img)
                        i_image += 1

            tf_buffer.clear()
            i_bag += 1
            clouds_per_bag_file.write(bag_file + ", " + str(i_cloud) + '\n')

    clouds_per_bag_file.close()


if __name__ == '__main__':
    log_filename = 'debug.log'
    log_file = open(log_filename, 'w')
    logging.basicConfig(filename=log_filename, level=logging.INFO, format='%(asctime)s %(message)s')

    # First argument: directory with rosbags
    # Second argument: directory for outputted data
    # must have format:
    # KITTI
    #  |- velodyne
    #  |- label_2
    #  |- image_2
    #  |- calib

    assert(len(sys.argv) == 3)
    input_dir = sys.argv[1]
    output_dir = sys.argv[2]
    main(input_dir, output_dir)

    log_file.close()

