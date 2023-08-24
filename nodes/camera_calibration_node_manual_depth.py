#!/usr/bin/env python3
import sys
import os
import copy
import rospy
from math import pi
import numpy as np
import tf.transformations as tft
import tf


import moveit_commander
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import Image
import moveit_msgs.msg
from geometry_msgs.msg import Pose

import datetime

import yaml

from cv_bridge import CvBridge
import cv2

import sys

SECONDS_WAITING_BEFORE_STARTING = 1.0
SECONDS_WAITING_BEFORE_TAKING_PICTURE = 3.0


def image_callback(data):
    global last_image
    last_image = copy.deepcopy(data)


def get_time_str(form):
    return str(datetime.datetime.now().strftime(form))


def capture_images():

    pos_to_take = open(positions_file,'r')
    calib_data_dir = dir_path+get_time_str("%d-%m-%Y-%H:%M:%S")
    rospy.loginfo("CALIBRATION: Creating directory " + calib_data_dir)
    os.mkdir(calib_data_dir)

    #arm.set_max_velocity_scaling_factor(max_speed)
    #arm.set_end_effector_link(eef_frame)

    capture_info = {
        'time_of_start': get_time_str("%d-%m-%Y-%H:%M:%S"),
        'used_positions_file': positions_file,
    }
    rospy.sleep(SECONDS_WAITING_BEFORE_STARTING)
    pose_nb = -1
    while True:
        pose_nb += 1
        if input("press (enter) to take picture or (enter + q) to exit") != "":
            break
        # Wait before capturing the image, in case the trajectory execution is not completely finished
        print(f'Wait for {SECONDS_WAITING_BEFORE_TAKING_PICTURE} before taking picture')
        rospy.sleep(SECONDS_WAITING_BEFORE_TAKING_PICTURE)

        ## Capture image
        rospy.loginfo("CALIBRATION: Capturing eye in hand image")
        # global last_image
        # cap_image = bridge.imgmsg_to_cv2(last_image, 'bgr8')

        color_img_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=0.5)
        color_img = bridge.imgmsg_to_cv2(color_img_msg, 'bgr8')

        depth_img_msg = rospy.wait_for_message('/camera/depth/image_rect_raw', Image, timeout=0.5)
        depth_img = bridge.imgmsg_to_cv2(depth_img_msg, 'passthrough')

        ## Save image eye
        date_time = get_time_str("%d-%m-%Y-%H:%M:%S")
        image_name = f'{image_name_prefix}_{pose_nb:03d}'
        depth_image_name = f'depth_{image_name_prefix}_{pose_nb:03d}'
        color_image_file_name = f'{calib_data_dir}/{image_name}.png'
        depth_image_file_name = f'{calib_data_dir}/{depth_image_name}.png'
        rospy.loginfo("CALIBRATION: Saving files " + f'{color_image_file_name}, {depth_image_file_name}')
        cv2.imwrite(color_image_file_name, color_img)
        cv2.imwrite(depth_image_file_name, depth_img)


        if rospy.get_param('/camera_calibration/use_ext_camera'):
            try:
                rospy.loginfo("CALIBRATION: Capturing external camera image")
                ext_image = rospy.wait_for_message(rospy.get_param('/camera_calibration/ext_cam_topic'), Image, timeout=0.5)
                ext_cap_image = bridge.imgmsg_to_cv2(ext_image, 'bgr8')
            except:
                rospy.logfatal("CALIBRATION ERROR: No image captured. Saving black screen!")
                ext_cap_image = np.zeros((100,100))
            ## Save image ext
            date_time = get_time_str("%d-%m-%Y-%H:%M:%S")
            if use_time:
                image_name_ext = image_name_prefix + '_ext_' + date_time
            else:
                image_name_ext = image_name_prefix + '_ext_' + str(pose_nb)
            file_name_ext = calib_data_dir + '/' + image_name_ext + '.jpg'
            rospy.loginfo("CALIBRATION: Saving file " + file_name)
            cv2.imwrite(file_name_ext, ext_cap_image)


        rospy.loginfo("CALIBRATION: Images captured and saved")


        ## Create log file

        # pose = arm.get_current_pose()
        listener.waitForTransform(base_frame, eef_frame, last_image.header.stamp, timeout=rospy.Duration(1))
        trans, rot = listener.lookupTransform(base_frame, eef_frame, last_image.header.stamp)

        # sample_count = 50
        # sum_joint_values = np.array(arm.get_current_joint_values())
        # for x in range(sample_count - 1):
        #     sum_joint_values += np.array(arm.get_current_joint_values())
        # joint_values = (sum_joint_values/sample_count).tolist()
        joint_values = arm.get_current_joint_values()
        log_info = {
            'directory': calib_data_dir,
            'image_name': image_name,
            'depth_name': depth_image_name,
            'time': date_time,
            't_vec': copy.deepcopy(trans),
            'r_mtx': tft.quaternion_matrix(rot).tolist(),
            'joint_values': joint_values,
            'image_shape': list(color_img.shape),
            'base_frame': base_frame,
            'eef_frame': eef_frame,
        }

        file_name = calib_data_dir + '/' + image_name + '.yaml'
        rospy.loginfo("CALIBRATION: Saving file " + file_name)
        with open(file_name, 'w') as f:
            yaml.dump(log_info, f)

    rospy.loginfo("CALIBRATION: All images captured!")

    capture_info['time_of_end'] = get_time_str("%d-%m-%Y-%H:%M:%S")
    capture_info['note'] = ''
    image_info_path = os.path.join(calib_data_dir,'image_capture_info.yaml')
    with open(image_info_path, 'w') as f:
        yaml.dump(capture_info, f)
    rospy.loginfo("CALIBRATION: Exiting!")
    return True


if __name__=="__main__":
    rospy.init_node('camera_calibration_node', anonymous=True)

    dir_path = rospy.get_param('camera_calibration/directory')
    group_name = rospy.get_param('camera_calibration/group')
    use_time = rospy.get_param('camera_calibration/use_time_in_image_names')
    image_name_prefix = rospy.get_param('camera_calibration/image_name_prefix')
    positions_file = rospy.get_param('camera_calibration/positions_file')
    max_speed = rospy.get_param('camera_calibration/max_speed')
    image_topic = rospy.get_param('camera_calibration/image_topic')
    base_frame = rospy.get_param('camera_calibration/base_frame')
    eef_frame = rospy.get_param('camera_calibration/eef_frame')
    moveit_commander.roscpp_initialize(sys.argv)

    listener = tf.TransformListener()
    rospy.sleep(1)

    bridge = CvBridge()


    #last_image = np.zeros((100,100))

    last_image = Image()
    last_image.height = 100
    last_image.width = 100
    last_image.data = np.zeros((100,100))
    colour_image_subs = rospy.Subscriber(image_topic, Image, image_callback)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)


    capture_images()
