import numpy as np
from ros_numpy import numpify
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
import tf2_ros
import cv2
import os
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

    # go through bag files in given directory one-by-one
    for bag_file in os.listdir(bag_path):
        bag_file = bag_path + bag_file

        if (LOGGING): print("processing file: " + bag_file)

        with rosbag.Bag(bag_file, 'r') as bag:
            info = bag.get_type_and_topic_info()

            i = 0
            max_i = 100

            # Storing timestamp information
            image_secs = []
            image_nsecs = []
            points_secs = []
            points_nsecs = []

            robot_name = 'X1'
            robot_parent_name = ''

            for topic, msg, t in bag:
                # Use rosbag info to determine msg type as
                # type(msg) gives a rosbag-specific helper type.
                dtype = info[1][topic][0]

                if dtype == 'tf2_msgs/TFMessage':
                    if (LOGGING): print('got transform at %s' % topic)
                    msg = TFMessage(*slots(msg))
                    for tf in msg.transforms:
                        # Robot pose has to be stored as tf not tf_static
                        if tf.child_frame_id == 'X1':
                            robot_parent_name = tf.header.frame_id
                            tf_buffer.set_transform(tf, 'default_authority')
                            if (LOGGING): print('%s -> %s set' % (tf.header.frame_id, tf.child_frame_id))
                        elif topic == '/tf':
                            tf_buffer.set_transform(tf, 'default_authority')
                            if (LOGGING): print('%s -> %s set' % (tf.header.frame_id, tf.child_frame_id))
                        elif topic == '/tf_static':
                            tf_buffer.set_transform_static(tf, 'default_authority')
                            if (LOGGING): print('static %s -> %s set' % (tf.header.frame_id, tf.child_frame_id))
                elif dtype == 'sensor_msgs/PointCloud2':
                    if (LOGGING): print('got cloud at %s' % topic)
                    # Create proper ROS msg type for ros_numpy.
                    msg = PointCloud2(*slots(msg))
                    # Convert to structured numpy array.
                    cloud = numpify(msg)
                    # Convert to 3xN array.
                    pts = np.stack([cloud[f] for f in ('x', 'y', 'z')])
                    # Add some default reflectance (assuming it is not provided, otherwise use it).
                    refl = np.zeros((1, cloud.shape[0], cloud.shape[1]), pts.dtype)
                    pts = np.concatenate((pts, refl))
                    # Save timestamp
                    points_secs.append(msg.header.stamp.secs)
                    points_nsecs.append(msg.header.stamp.nsecs)
                    cloud_path = out_path +'/kitti/object/%06i.bin' % i_cloud
                    with open(cloud_path, 'wb') as file:
                        pts.T.tofile(file)
                    i_cloud += 1
                elif dtype == 'sensor_msgs/Image':
                    if (LOGGING): print('got image at %s' % topic)
                    # Create proper ROS msg type for ros_numpy.
                    msg = Image(*slots(msg))
                    # Convert to structured numpy array.
                    img = numpify(msg)
                    # Save timestamp
                    image_secs = msg.header.stamp.secs
                    image_nsecs = msg.header.stamp.nsecs
                    image_path = out_path +'/kitti/image/%06i.png' % i_image
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    cv2.imwrite(image_path, img)

                    i_image += 1
                i += 1
                if i == max_i:
                    # save artifact poses
                    name = out_path + '/kitti/tf_coordinates/artifacts/%02i.txt' % i_bag
                    dh.save_artifact_data(name, tf_buffer)
                    # TODO: save robot poses in time
                    # print("bag file: " + bag_file)
                    # print("image secs: " + str(image_secs))
                    # print("image nsecs: " + str(image_nsecs))
                    # print("points secs: " + str(points_secs))
                    # print("points nsecs: " + str(points_nsecs))
                    print(tf_buffer.lookup_transform_core('backpack_1', 'X1/laser', rospy.Time(0)))
                    # dh.save_robot_data(name, tf_buffer, child_frame=robot_name, parent_frame=robot_parent_name, )
                    # note: rospy.Time(secs=~, nsecs=~)
                    # TODO: save sensor poses in time
                    # TODO: create git repository
                    tf_buffer.clear()
                    i_bag += 1
                    break


if __name__ == '__main__':
    main('/home/atestee/rosbag/', '/home/atestee/rosdata/')