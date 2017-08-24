#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from datetime import datetime
import csv
import json
import os
import requests
import rospy
from diagnostic_msgs.msg import DiagnosticArray
import traceback


class BatteryLogger(object):
    def __init__(self, period=120):
        self.period = rospy.Duration(period)
        self.last_update = rospy.Time(0)

    def write(self, date, info):
        raise NotImplementedError()

class FileLogger(BatteryLogger):
    def __init__(self, out_dir):
        super(FileLogger, self).__init__(period=600)
        self.out_dir = out_dir
        if not os.path.exists(self.out_dir):
            os.makedirs(self.out_dir)

    def write(self, date, info):
        filename = os.path.join(self.out_dir,
                                datetime.now().strftime("battery_%Y-%m-%d.log"))
        lines = []
        index = ["HardwareID", "CycleCount", "FullCapacity", "RemainingCapacity"]
        if not os.path.exists(filename):
            # write index
            lines.append(["Date", "Name"] + index)

        for name, batt in info.items():
            values = []
            for k in index:
                if k in batt:
                    values.append(str(batt[k]))
            lines.append([date.secs, name] + values)

        with open(filename, "a") as f:
            writer = csv.writer(f, lineterminator=os.linesep)
            writer.writerows(lines)

class DweetLogger(BatteryLogger):
    def __init__(self, uid):
        super(DweetLogger, self).__init__(period=10)
        self.uid = uid

    def write(self, date, info):
        info["date"] = date.secs
        res = requests.post("http://dweet.io/dweet/for/{uid}".format(uid=self.uid),
                            json=info)
        assert res.ok and json.loads(res.content)["this"] == "succeeded"


class BatteryInfoAggregator(object):
    def __init__(self):
        robot_name = rospy.get_param("robot/name")
        self.loggers = [
            FileLogger("/var/log/ros/battery"),
            DweetLogger(robot_name + "-battery"),
        ]

        self.sub_diag = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diag_callback)

    def diag_callback(self, msg):
        # aggregate
        results = {}
        for s in msg.status:
            if s.name.startswith("/Power System/Smart Battery"):
                if s.name not in results:
                    results[s.name] = { "HardwareID": s.hardware_id }
                for kv in s.values:
                    if(kv.key.startswith("Full Charge Capacity (mAh)")):
                        results[s.name]["FullCapacity"] = int(kv.value)
                    elif kv.key.startswith("Remaining Capacity"):
                        results[s.name]["RemainingCapacity"] = int(kv.value)
                    elif kv.key.startswith("Batery Status"):
                        results[s.name]["Status"] = int(kv.value)
                    elif kv.key.startswith("Cycle Count"):
                        results[s.name]["CycleCount"] = int(kv.value)
                    elif kv.key.startswith("Manufacture Date"):
                        results[s.name]["ManufactureDate"] = kv.value

        # log
        for logger in self.loggers:
            if logger.last_update + logger.period < msg.header.stamp:
                try:
                    logger.write(msg.header.stamp, results)
                    logger.last_update = msg.header.stamp
                except Exception as e:
                    rospy.logerr("[%s] Failed to write to logger: %s", type(logger).__name__, str(e))
                    rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    rospy.init_node("battery_logger")
    agg = BatteryInfoAggregator()
    rospy.spin()
