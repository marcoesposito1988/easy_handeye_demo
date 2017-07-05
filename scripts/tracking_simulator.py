#!/usr/bin/env python2

from __future__ import division

import rospy
import tf
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

rospy.init_node('tracking_simulator')


def parse_transformation_string(t_str):
    tx, ty, tz, rx, ry, rz, rw = [float(t) for t in t_str.split(' ')[0:7]]
    base_frame, dest_frame = t_str.split(' ')[7:9]
    return (np.array([tx, ty, tz]), np.array([rx, ry, rz, rw])), base_frame, dest_frame

cal_transformation, cal_origin_frame, cal_dest_frame = parse_transformation_string(rospy.get_param('~calibration_transformation'))
aux_transformation, aux_origin_frame, aux_dest_frame = parse_transformation_string(rospy.get_param('~auxiliary_transformation'))

frequency = rospy.get_param('~frequency')
transl_noise = rospy.get_param('~translation_noise_stdev')
rot_noise = rospy.get_param('~rotation_noise_stdev')

rate = rospy.Rate(frequency)
broadcaster = tf.TransformBroadcaster()
listener = tf.TransformListener()


def fuzzy_transformation(transform):
    transl, rot = transform
    noisy_translation = np.random.normal(transl, transl_noise)
    noisy_rotation = np.random.normal(rot, rot_noise)
    noisy_rotation /= np.linalg.norm(noisy_rotation)
    return noisy_translation, noisy_rotation

cal_transform_stamped = TransformStamped(header=Header(frame_id=cal_origin_frame), child_frame_id=cal_dest_frame+'_dummy',
                                         transform=Transform(translation=Vector3(*cal_transformation[0]),
                                                             rotation=Quaternion(*cal_transformation[1])))
aux_transform_stamped = TransformStamped(header=Header(frame_id=aux_origin_frame), child_frame_id=aux_dest_frame,
                                         transform=Transform(translation=Vector3(*aux_transformation[0]),
                                                             rotation=Quaternion(*aux_transformation[1])))


while not rospy.is_shutdown():
    cal_transform_stamped.header.stamp = rospy.Time.now() - rospy.Duration(0.1)
    listener.setTransform(cal_transform_stamped)

    aux_transform_stamped.header.stamp = rospy.Time.now() - rospy.Duration(0.1)
    fuzzy_aux_transform = fuzzy_transformation(aux_transformation)
    aux_transform_stamped.transform.translation = Vector3(*fuzzy_aux_transform[0])
    aux_transform_stamped.transform.rotation = Quaternion(*fuzzy_aux_transform[1])
    listener.setTransform(aux_transform_stamped)
    broadcaster.sendTransform(fuzzy_aux_transform[0], fuzzy_aux_transform[1],
                              aux_transform_stamped.header.stamp,
                              aux_transform_stamped.child_frame_id, aux_transform_stamped.header.frame_id)

    if listener.canTransform(cal_dest_frame+'_dummy', aux_dest_frame, rospy.Time(0)):
        tracking_transform = listener.lookupTransform(cal_dest_frame+'_dummy', aux_dest_frame, rospy.Time(0))
        broadcaster.sendTransform(tracking_transform[0], tracking_transform[1], rospy.Time.now(), cal_dest_frame, aux_dest_frame)
    else:
        print('skipping: cannot transform between {} and  {}'.format(cal_dest_frame, aux_dest_frame))
    rate.sleep()
