#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from mongodb_store.message_store import MessageStoreProxy
from multiprocessing import Process, Queue
import os
import psutil
import rospy


class MongoLogger(object):
    '''
    This is base class for mongodb logger
    Inserting message to database is executed non-block as possible.

    In an inherited class, message can be inserted to database by insert method.
    '''

    def __init__(self, db_name=None, col_name=None):
        if db_name is None:
            self.db_name = rospy.get_param('robot/database','jsk_robot_lifelog')
        else:
            self.db_name = db_name
        self.col_name = rospy.get_param('robot/name', col_name)
        if self.col_name is None:
            rospy.logerr("please specify param \"robot/name\" (e.g. pr1012, olive)")
            exit(1)

        self.msg_store = MessageStoreProxy(database=self.db_name, collection=self.col_name)
        rospy.loginfo("connected to %s.%s" % (self.db_name, self.col_name))

        self.__queue = Queue()
        self.__process = None

        rospy.on_shutdown(self.__kill_insert_proc)

    def __kill_insert_proc(self):
        if self.__process is not None and self.__process.is_alive():
            rospy.loginfo("waiting child process shutdown for 10 seconds...")
            self.__process.join(timeout=10)
            if self.__process.is_alive():
                parent = psutil.Process(os.getpid())
                for child in parent.get_children(recursive=True):
                    rospy.logwarn("killing child process [%s: %s]" % (child.pid, child.name))
                    child.kill()

    def __insert_proc(self):
        while not rospy.is_shutdown():
            msg, meta = self.queue.get() # wait
            try:
                res = self.msg_store.insert(msg, meta=meta)
            except Exception as e:
                rospy.logerr("Error: %s" % str(e))

    def insert(self, msg, meta={}, block=False):
        '''
        Insert message to mongodb server.
        Inserting is executed as a non-blocking process.
        If ``block`` is set to ``True``, wait the result of insertion.
        '''
        if block:
            return self.msg_store.insert(msg, meta=meta)
        else:
            self.queue.put((msg, meta))

    def run(self):
        '''run child process to insert message to mongodb server'''
        self.__process = Process(target=MongoLogger.__insert, args=[self])
        self.__process.start()


if __name__ == '__main__':
    rospy.init_node("mongo_logger")
    l = MongoLogger("test", "test")
    l.run()
