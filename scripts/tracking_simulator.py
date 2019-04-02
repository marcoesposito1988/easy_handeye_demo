#!/usr/bin/env python2

# this script emulates the tf output of a tracking system in a hand-eye calibration setup
# it is assumed that a MoveIt! instance is running, so that the tf frames for a (simulated) robot are present
# the tracking output is computed from the given "calibration" and "arbitrary" transforms
# the calibration transform is the one which is the unknown for the calibration process; the arbitrary transformation
# is the collateral transformation which is ignored during the calibration process, but it is necessary to compute the
# tracking output here
# both calibration and arbitrary transforms can be passed as parameters, otherwise (hopefully sane) defaults are used

# TODO: output tracking information only when marker within field of view of tracking system to make it more realistic
# (in prism for optical tracking, within radius for EM, ...)

# TODO: more noise models

from __future__ import division

import rospy
import tf2_ros
import transforms3d as tfs
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

rospy.init_node('tracking_simulator')

is_eye_on_hand = rospy.get_param('~eye_on_hand')
robot_base_frame = rospy.get_param('~robot_base_frame')
robot_effector_frame = rospy.get_param('~robot_effector_frame')
tracking_base_frame = rospy.get_param('~tracking_base_frame')
tracking_marker_frame = rospy.get_param('~tracking_marker_frame')


def parse_transformation(t_str):
    tx, ty, tz, rx, ry, rz, rw = [float(t) for t in t_str.split(' ')[0:7]]
    return np.array([tx, ty, tz]), np.array([rx, ry, rz, rw])


if rospy.has_param('~calibration_transformation'):
    calibration_transformation = parse_transformation(rospy.get_param('~calibration_transformation'))
else:
    if is_eye_on_hand:
        calibration_transformation = parse_transformation('0 0 0.05 0 0 0 1')
    else:
        calibration_transformation = parse_transformation('2 0 1 0 0 0 1')

if rospy.has_param('~arbitrary_transformation'):
    arbitrary_transformation = parse_transformation(rospy.get_param('~arbitrary_transformation'))
else:
    if is_eye_on_hand:
        arbitrary_transformation = parse_transformation('2 0 1 0 0 0 1')
    else:
        arbitrary_transformation = parse_transformation('0 0 0.05 0 0 0 1')


if is_eye_on_hand:
    calibration_frame_robot = robot_effector_frame
    calibration_frame_tracking = tracking_marker_frame
    arbitrary_frame_robot = robot_base_frame
    arbitrary_frame_tracking = tracking_base_frame
else:
    calibration_frame_robot = robot_base_frame
    calibration_frame_tracking = tracking_base_frame
    arbitrary_frame_robot = robot_effector_frame
    arbitrary_frame_tracking = tracking_marker_frame


frequency = rospy.get_param('~frequency')
transl_noise = rospy.get_param('~translation_noise_stdev')
rot_noise = rospy.get_param('~rotation_noise_stdev')

rate = rospy.Rate(frequency)


def fuzzy_transformation(translation, quaternion):
    noisy_translation = np.random.normal(translation, transl_noise)
    noisy_quaternion = np.random.normal(quaternion, rot_noise)  # TODO: better noise for rotation
    noisy_quaternion /= np.linalg.norm(noisy_quaternion)
    return noisy_translation, noisy_quaternion


# we publish the tracking transform reversed for keeping the tf chain direction
tracking_transform_msg_stmpd = TransformStamped(header=Header(frame_id=tracking_marker_frame), child_frame_id=tracking_base_frame,
                                                   transform=Transform(translation=Vector3(),
                                                             rotation=Quaternion()))


arbitrary_transform_msg_stmpd = TransformStamped(header=Header(frame_id=arbitrary_frame_robot), child_frame_id=tracking_marker_frame,
                                                 transform=Transform(translation=Vector3(*arbitrary_transformation[0]),
                                                             rotation=Quaternion(*arbitrary_transformation[1])))


# dummy calibration so that we can just measure the tracking transformation from tf;
# we just put this into the buffer without broadcasting
dummy_calibration_transform_msg_stmpd = TransformStamped(header=Header(frame_id=arbitrary_frame_robot), child_frame_id=tracking_base_frame + '_dummy',
                                                         transform=Transform(translation=Vector3(*calibration_transformation[0]),
                                                             rotation=Quaternion(*calibration_transformation[1])))


tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)
tfBroadcaster = tf2_ros.TransformBroadcaster()


tfBuffer.set_transform_static(arbitrary_transform_msg_stmpd, 'default_authority')
tfBuffer.set_transform_static(dummy_calibration_transform_msg_stmpd, 'default_authority')

# in the loop:
# - compute the tracking transform by closing the loop
# (if eye on base: marker -> hand -> robot base -> tracking base; that is arbitrary -> robot hand to base -> calibration)
# (if eye on hand: tracking base -> hand -> robot base -> tracking marker; that is calibraton -> robot hand to base -> arbitrary)
# - publish the tracking transform
# - optionally publish the calibration transform with dummy frame for debugging

while not rospy.is_shutdown():
    # publish arbitrary transform
    arbitrary_transform_msg_stmpd.header.stamp = rospy.Time.now() - rospy.Duration(0.1)
    tfBroadcaster.sendTransform(arbitrary_transform_msg_stmpd)

    # measure tracking transform
    measured_tracking_transformation_msg_stmpd = tfBuffer.lookup_transform(tracking_marker_frame, tracking_base_frame+'_dummy', rospy.Time(0))

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
