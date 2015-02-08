#!/usr/bin/python
#
# Store the ObjectDetection message
#

try:
    import roslib; roslib.load_manifest('jsk_pr2_startup')
except:
    pass

import rospy
import tf
from threading import Timer

from geometry_msgs.msg import PoseStamped, TransformStamped
from posedetection_msgs.msg import ObjectDetection
from sensor_msgs.msg import Image

from mongodb_store.message_store import MessageStoreProxy


class ObjectDetectionDBSettings(object):
    OBJECT_DETECTION = "posedetection_msgs/ObjectDetection"
    IMAGE_SUFFIX = "/image_rect_color_throttled"
    DEBUG_IMAGE_SUFFIX = "/debug_image"

    def deplayed(sec):
        def decorator(f):
            def wrapper(*args, **kargs):
                t = Timer(sec, f, args, kargs)
                t.start()
            return wrapper
        return decorator

class ObjectDetectionDB(ObjectDetectionDBSettings):
    def __init__(self):
        self.db_name = rospy.get_param('~db_name','jsk_pr2_lifelog')
        self.col_name = rospy.get_param('~col_name', 'objectdetection_db')
        self.update_cycle = rospy.get_param('update_cycle', 1)
        self.map_frame = rospy.get_param('~map_frmae', 'map')
        self.robot_frame = rospy.get_param('~robot_frame','base_footprint')
        self.image_savetime_torelance = rospy.get_param('~image_savetime_torelance', 1.0)
        self.tf_listener = tf.TransformListener()
        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))
        rospy.loginfo("map->robot: %s -> %s" % (self.map_frame, self.robot_frame))

        self.subscribers = {}

        self.images = {}

    # DB Insertion function
    @delayed(self.image_savetime_torelance)
    def _insert_object_detection_to_db(self, map_to_robot, robot_to_obj, topic_name):
        try:
            self.msg_store.insert(map_to_robot)
            self.msg_store.insert(robot_to_obj)
            if self.images.has_key(topic_name):
                img_cnt = len(self.images[topic_name])
                [self.msg_store.insert(m) for m in self.images[topic_name]]
                self.images[topic_name].clear()
                rospy.loginfo('inserted map2robot: %s, robot2obj: %s with %d imgs' % (map_to_robot, robot_to_obj, img_cnt))
            else:
                rospy.loginfo('inserted map2robot: %s, robot2obj: %s' % (map_to_robot, robot_to_obj))
        except Exception as e:
            rospy.logwarn('failed to insert to db' + e)

    def _lookup_transform(self, target_frame, source_frame, time=rospy.Time(0), timeout=rospy.Duration(0.0)):
        self.tf_listener.waitForTransform(target_frame, source_frame, time, timeout)
        res = self.tf_listener.lookupTransform(target_frame, source_frame, time)
        ret = TransformStamped()
        ret.header.frame_id = target_frame
        ret.header.stamp = time
        ret.child_frame_id = source_frame
        ret.transform.translation.x = res[0][0]
        ret.transform.translation.y = res[0][1]
        ret.transform.translation.z = res[0][2]
        ret.transform.rotation.x = res[1][0]
        ret.transform.rotation.y = res[1][1]
        ret.transform.rotation.z = res[1][2]
        ret.transform.rotation.w = res[1][3]
        return ret

    def _objectdetection_cb(self, msg, topic_name):
        try:
            self.tf_listener.waitForTransform(self.robot_frame, self.map_frame, msg.header.stamp, rospy.Duration(1.0))
            map_to_robot = self._lookup_transform(self.robot_frame, self.map_frame, msg.header.stamp)
        except Exception as e:
            rospy.logwarn("failed to lookup tf: %s", e)
            return

        try:
            for obj in msg.objects:
                spose = PoseStamped(header=msg.header,pose=obj.pose)
                tpose = self.tf_listener.transformPose(self.robot_frame, spose)
                obj.pose = tpose.pose
                self._insert_object_detection_to_db(map_to_robot, obj, topic_name)
        except Exception as e:
            rospy.logwarn("failed to object pose transform: %s", e)

    def _image_cb(self, msg, od_topic_name):
        for i in self.images[od_topic_name]:
            if abs((msg.header.stamp - i.header.stamp).to_sec) > 2.0 * self.image_savetime_torelance:
                # allow to save images before/after <image_savetime_torelance> seconds.
                self.images[od_topic_name].pop(i)
            else:
                break
        self.images[od_topic_name] += [msg]

    def update_subscribers(self):
        all_topics = rospy.client.get_published_topics()
        targets = [x for x in all_topics if x[1]==OBJECT_DETECTION and ('_agg' in x[0])]
        sensor_ns = "/".join(x[0].split("/")[:-1])

        for group, sub_dict in {g:s for g, s in self.subscribers if s[OBJECT_DETECTION].get_num_connections() == 0}:
            for t, sub in sub_dict:
                sub.unregister()
                rospy.loginfo('unsubscribe (%s)', sub)
            self.subscribers.pop(group)

        for topic_info in [t for t in targets if not t[0] in [s[OBJECT_DETECTION].name for g,s in self.subscribers]]:
            od_sub = rospy.Subscriber(topic_info[0], ObjectDetection,
                                      lambda msg: self._objectdetection_cb(msg, topic_info[0]))
            self.subscribers[topic_info[0]] = [od_sub]
            rospy.loginfo('start subscribe (%s)', od_sub.name)

            image_topic_name = sensor_ns + IMAGE_SUFFIX
            if image_topic_name in [t[0] for t in all_topics]:
                img_sub = rospy.Subscriber(image_topic_name, Image,
                                           lambda msg: self._image_cb(msg, topic_info[0]))
                self.images[topic_info[0]] = []
                self.subscribers[topic_info[0]] += [img_sub]
                rospy.loginfo('start subscribe(%s)', img_sub.name)

    def sleep_one_cycle(self):
        rospy.sleep(self.update_cycle)

if __name__ == "__main__":
    rospy.init_node('objectdetecton_db')
    obj = ObjectDetectionDB()
    while not rospy.is_shutdown():
        obj.update_subscribers()
        obj.sleep_one_cycle()
