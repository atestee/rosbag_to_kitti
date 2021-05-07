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

from tf2_py import ExtrapolationException, LookupException

import data_handler
import data_handler as dh

LOGGING = False

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

    i_cloud = 0
    i_image = 0
    i_bag = 0
    robot_name = 'X1'

    # go through bag files in given directory one-by-one
    for bag_file in os.listdir(bag_path):
        bag_file = bag_path + bag_file

        # if (LOGGING): print("processing file: " + bag_file)
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

        with rosbag.Bag(bag_file, 'r') as bag:
            info = bag.get_type_and_topic_info()

            # Storing timestamp information
            image_transforms = []
            points_transforms = []

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
                    transforms = data_handler.lookup_transforms_to_artifacts(msg, tf_buffer)
                    if len(transforms) != 0:
                        # Save transforms
                        image_transforms_filename = os.path.join(out_path, 'kitti', 'object_transforms', '%06i.txt' % i_cloud   )
                        dh.save_transforms(transforms, image_transforms_filename)
                        # Save pointcloud
                        cloud_filename = os.path.join(out_path, 'kitti', 'object', '%06i.bin' % i_cloud)
                        with open(cloud_filename, 'wb') as file:
                            pts.T.tofile(file)
                        i_cloud += 1
                elif dtype == 'sensor_msgs/Image':
                    logging.info('Got image at %s' % topic)
                    # Create proper ROS msg type for ros_numpy.
                    msg = Image(*slots(msg))
                    # Convert to structured numpy array.
                    img = numpify(msg)
                    transforms = data_handler.lookup_transforms_to_artifacts(msg, tf_buffer)
                    if len(transforms) != 0:
                        # Save transforms
                        image_transforms_filename = os.path.join(out_path, 'kitti', 'image_transforms', '%06i.txt' % i_image)
                        dh.save_transforms(transforms, image_transforms_filename)
                        # Save image
                        image_filename = os.path.join(out_path, 'kitti', 'image', '%06i.png' % i_image)
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        cv2.imwrite(image_filename, img)
                        i_image += 1
                # elif dtype == 'sensor_msgs/CameraInfo':
                #     logging.info("Got calib info at %s" % topic)
                #     msg = CameraInfo(*slots(msg))
                #     calib_info = numpify(msg)

            # Save transforms into files
            # dh.save_transforms(points_transforms, out_path + '/kitti/image_transforms/%02i.txt' % i_bag)
            # dh.save_transforms(image_transforms, out_path + '/kitti/laser_transforms/%02i.txt' % i_bag)
            tf_buffer.clear()
            i_bag += 1


if __name__ == '__main__':
    log_filename = 'debug.log'
    log_file = open(log_filename, 'w')
    logging.basicConfig(filename=log_filename, level=logging.INFO, format='%(asctime)s %(message)s')

    main('/home/atestee/rosbag/', '/home/atestee/rosdata/')

    log_file.close()