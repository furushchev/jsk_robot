#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from mongodb_store.message_store import MessageStoreProxy
from posedetection_msgs.msg import Object6DPose
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped
from datetime import datetime, timedelta
import tf

from transform_utils import TransformationUtils as T
from visualization_utils import VisualizationUtils as V

class DBPlay(object):
    def __init__(self):
        self.db_name = rospy.get_param('~db_name','jsk_pr2_lifelog')
        self.col_name = rospy.get_param('~col_name', 'pr1012')
        self.duration = rospy.get_param('~duration', 30) # days
        self.msg_store = MessageStoreProxy(database=self.db_name,
                                           collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))
        self.pub = rospy.Publisher('/move_base_marker_array', MarkerArray)
        self.marker_count = 0
        
        while not rospy.is_shutdown():
            trans = self.msg_store.query(type=TransformStamped._type,
                                         meta_query={"inserted_at": {
                                             "$gt": datetime.now() - timedelta(days=self.duration)
                                         }},
                                         sort_query=[("$natural", -1)])
            m_arr = MarkerArray()
            m_arr.markers = V.transformStampedArrayToLabeledLineStripMarker(trans, label_downsample=10, discrete=True)
            self.pub.publish(m_arr)
            rospy.sleep(1.0)
            rospy.logdebug("publishing move_base_marker_array")


if __name__ == '__main__':
    rospy.init_node('visualize_move_base', log_level=rospy.DEBUG)
    o = DBPlay()
