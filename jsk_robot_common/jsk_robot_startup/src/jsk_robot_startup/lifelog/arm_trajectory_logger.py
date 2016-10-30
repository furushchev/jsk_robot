#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy

from mongodb_store.message_store import MessageStoreProxy


class ArmTrajectoryLogger(object):
    def __init__(self):
        self.db_name = rospy.get_param('robot/database','jsk_robot_lifelog')
        try:
            self.col_name = rospy.get_param('robot/name')
        except KeyError as e:
            rospy.logerr("please specify param \"robot/name\" (e.g. pr1012, olive)")
            exit(1)
