#!/usr/bin/env python

# this script emulates the tf output of a tracking system in a hand-eye calibration setup, given the ground-truth
# calibration and arbitrary transforms. the output is consists of the transform between the camera and the marker,
# plus noise
#
# if we are not calibrating but publishing the calibration, but demoing the result, we compute the result according to
# the specified ground truth frame but publish it with respect to the actual result of the calibration (so that the
# outcome of a faulty calibration is faithfully reproduced)
#
# it is assumed that a MoveIt! instance is running, so that the tf frames for a (simulated) robot are present
#
# the calibration transform is the one which is the unknown for the calibration process; the arbitrary transformation
# is the collateral transformation which is ignored during the calibration process, but it is necessary to compute the
# tracking output here

# TODO: output tracking information only when marker within field of view of tracking system to make it more realistic
# (in prism for optical tracking, within radius for EM, ...)

# TODO: more noise models

from __future__ import division

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

rospy.init_node('tracking_simulator')

tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)
tfBroadcaster = tf2_ros.TransformBroadcaster()
tfStaticBroadcaster = tf2_ros.StaticTransformBroadcaster()

is_calibration = rospy.get_param('~is_calibration')
is_eye_on_hand = rospy.get_param('~eye_on_hand')

robot_base_frame = rospy.get_param('~robot_base_frame')
robot_effector_frame = rospy.get_param('~robot_effector_frame')
tracking_base_frame = rospy.get_param('~tracking_base_frame')
tracking_marker_frame = rospy.get_param('~tracking_marker_frame')
CAMERA_DUMMY_FRAME = tracking_base_frame + '_gt'
MARKER_DUMMY_FRAME = tracking_marker_frame + '_gt'

frequency = rospy.get_param('~frequency')
transl_noise = rospy.get_param('~translation_noise_stdev')
rot_noise = rospy.get_param('~rotation_noise_stdev')


def parse_transformation(t_str):
    tx, ty, tz, rx, ry, rz, rw = [float(t) for t in t_str.split(' ')[0:7]]
    return np.array([tx, ty, tz]), np.array([rx, ry, rz, rw])


def fuzzy_transformation(translation, quaternion):
    noisy_translation = np.random.normal(translation, transl_noise)
    noisy_quaternion = np.random.normal(quaternion, rot_noise)  # TODO: better noise for rotation
    noisy_quaternion /= np.linalg.norm(noisy_quaternion)
    return noisy_translation, noisy_quaternion


ground_truth_calibration_transformation = parse_transformation(
    rospy.get_param('~ground_truth_calibration_transformation'))
arbitrary_marker_placement_transformation = parse_transformation(
    rospy.get_param('~arbitrary_marker_placement_transformation'))

if is_eye_on_hand:
    calibration_origin_frame = robot_effector_frame
    marker_placement_origin_frame = robot_base_frame
else:
    calibration_origin_frame = robot_base_frame
    marker_placement_origin_frame = robot_effector_frame

# set the marker placement in the buffer, so that we can measure the tracking output
# but the transform doesn't show up in tf (more realistic to the viewer, and no loops in the DAG)
arbitrary_transform_msg_stmpd = TransformStamped(
    header=Header(frame_id=marker_placement_origin_frame, stamp=rospy.Time.now()), child_frame_id=MARKER_DUMMY_FRAME,
    transform=Transform(translation=Vector3(*arbitrary_marker_placement_transformation[0]),
                        rotation=Quaternion(*arbitrary_marker_placement_transformation[1])))
tfBuffer.set_transform_static(arbitrary_transform_msg_stmpd, 'default_authority')

if is_calibration:
    # publish a dummy calibration for visualization purposes
    calib_gt_msg_stmpd = TransformStamped(header=Header(frame_id=calibration_origin_frame),
                                          child_frame_id=tracking_base_frame,
                                          transform=Transform(
                                              translation=Vector3(*ground_truth_calibration_transformation[0]),
                                              rotation=Quaternion(*ground_truth_calibration_transformation[1])))
    tfStaticBroadcaster.sendTransform(calib_gt_msg_stmpd)

# set the ground truth calibration; during demo this allows us to compute the correct tracking output even if the calibration failed
calib_gt_msg_stmpd = TransformStamped(header=Header(frame_id=calibration_origin_frame),
                                      child_frame_id=CAMERA_DUMMY_FRAME,
                                      transform=Transform(
                                          translation=Vector3(*ground_truth_calibration_transformation[0]),
                                          rotation=Quaternion(*ground_truth_calibration_transformation[1])))
tfBuffer.set_transform_static(calib_gt_msg_stmpd, 'default_authority')

# in the loop:
# - compute the tracking transform by closing the loop
# - add noise
# - publish the tracking transform

tracking_transform_msg_stmpd = TransformStamped(header=Header(frame_id=tracking_base_frame),
                                                child_frame_id=tracking_marker_frame,
                                                transform=Transform(translation=Vector3(),
                                                                    rotation=Quaternion()))

rate = rospy.Rate(frequency)

while not rospy.is_shutdown():

    if not tfBuffer.can_transform(CAMERA_DUMMY_FRAME, MARKER_DUMMY_FRAME, rospy.Time(0)):
        rospy.loginfo('Waiting for tf tree to be connected...')
        rospy.sleep(1)
        continue

    # measure tracking transform
    measured_tracking_transformation_msg_stmpd = tfBuffer.lookup_transform(CAMERA_DUMMY_FRAME, MARKER_DUMMY_FRAME,
                                                                           rospy.Time(0))

    # add noise
    t = measured_tracking_transformation_msg_stmpd.transform.translation
    r = measured_tracking_transformation_msg_stmpd.transform.rotation
    fuzzy_translation, fuzzy_quaternion = fuzzy_transformation(
        np.array((t.x, t.y, t.z)),
        np.array((r.x, r.y, r.z, r.w)))

    # publish tracking transform
    tracking_transform_msg_stmpd.header.stamp = rospy.Time.now() - rospy.Duration(0.1)
    tracking_transform_msg_stmpd.transform.translation = Vector3(*fuzzy_translation)
    tracking_transform_msg_stmpd.transform.rotation = Quaternion(*fuzzy_quaternion)

    tfBroadcaster.sendTransform(tracking_transform_msg_stmpd)

    rate.sleep()
