#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from tf import transformations as trans
from geometry_msgs.msg import Point, Quaternion, PoseStamped
import numpy as np

class TransformationUtils():
    @classmethod
    def matrixFromPosition(cls, pos):
        return trans.translation_matrix((pos.x, pos.y, pos.z))

    @classmethod
    def matrixFromRotation(cls, rot):
        return trans.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))

    @classmethod
    def matrixFromPose(cls, pose):
        return np.dot(cls.matrixFromPosition(pose.position), cls.matrixFromRotation(pose.orientation))

    @classmethod
    def matrixFromTransform(cls, t):
        return np.dot(cls.matrixFromPosition(t.translation), cls.matrixFromRotation(t.rotation))

    @classmethod
    def pointFromMatrix(cls, m):
        return Point(*trans.translation_from_matrix(m))

    @classmethod
    def quaternionFromMatrix(cls, m):
        return Quaternion(*trans.quaternion_from_matrix(m))

    @classmethod
    def transformPoseWithTransformStamped(cls, p, ts):
        m = np.dot(cls.matrixFromTransform(ts.transform), cls.matrixFromPose(p))
        pos = cls.pointFromMatrix(m)
        quat = cls.quaternionFromMatrix(m)
        ps = PoseStamped()
        ps.header.frame_id = ts.header.frame_id
        ps.header.stamp = ts.header.stamp
        ps.pose.position = pos
        ps.pose.orientation = quat
        return ps
