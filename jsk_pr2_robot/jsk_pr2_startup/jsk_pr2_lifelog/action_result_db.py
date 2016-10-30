#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from jsk_robot_startup.lifelog import ActionResultLogger


if __name__ == "__main__":
    rospy.init_node('action_result_logger')
    ActionResultDB().run()
