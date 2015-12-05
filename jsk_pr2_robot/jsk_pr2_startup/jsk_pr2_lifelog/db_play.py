#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from mongodb_store.message_store import MessageStoreProxy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from posedetection_msgs.msg import Object6DPose
from geometry_msgs.msg import TransformStamped
from datetime import datetime, timedelta
import tf

from transform_utils import TransformationUtils as tu

class DBPlay(object):
    def __init__(self):
        self.db_name = rospy.get_param('~db_name','jsk_pr2_lifelog')
        self.col_name = rospy.get_param('~col_name', 'pr1012')
        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))
        self.pub = rospy.Publisher('/marker_array', MarkerArray)

        objs = self.msg_store.query(type=Object6DPose._type,
                                    meta_query={"inserted_at": {
                                        "$gt": datetime.now() - timedelta(days=60)
                                    }},
                                    sort_query=[("$natural", -1)])

        first_obj_meta = objs[0][1]

        trans = [tuple(self.msg_store.query(type=TransformStamped._type,
                                            meta_query={"inserted_at": {
                                              "$lt": first_obj_meta["inserted_at"]
                                            }},
                                            sort_query=[("$natural", -1)],
                                            single=True))]

        trans += self.msg_store.query(type=TransformStamped._type,
                                      meta_query={"inserted_at": {
                                        "$gt": first_obj_meta["inserted_at"]
                                      }},
                                      sort_query=[("$natural", -1)])


        j = 0
        m_arr = MarkerArray()
        for i in range(len(objs)):
            o,o_meta = objs[i]
            t,t_meta = trans[j]
            if o_meta["inserted_at"] < t_meta["inserted_at"]:
                j += 1
                t,t_meta = trans[j]

            ps = tu.transformPoseWithTransformStamped(o.pose, t)

            h = Header()
            h.stamp = rospy.Time(0)
            h.frame_id = "map"#ps.header.frame_id
            m = Marker(type=Marker.CUBE_LIST,
                       action=Marker.ADD,
                       header=h,
                       id=i)
            m.scale.x = 10
            m.scale.y = 10
            m.scale.z = 10

            m.color = ColorRGBA(1.0,0,0,1.0)
            m.text = o.type
            m.ns = "play"
            m.lifetime = rospy.Time(300)
            m_arr.markers.append(m)

        while not rospy.is_shutdown():
            self.pub.publish(m_arr)
            rospy.sleep(1.0)
            print "publishing"


if __name__ == '__main__':
    rospy.init_node('play')
    o = DBPlay()
