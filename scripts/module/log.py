#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospkg

import datetime
import os
import rospy
from std_msgs.msg import String


class Log:
    def __init__(self):
        rospy.init_node("sound_system_log")
        rospy.Subscriber("/sound_system/log/spoke", String, self.spoke_callback)
        rospy.Subscriber("/sound_system/log/heard", String, self.heard_callback)
        
        self.log_path = "{}/{}/".format(rospkg.RosPack().get_path('sound_system'), "log")
        # logフォルダ生成
        if not os.path.exists(self.log_path):
            os.mkdir(self.log_path)
        
        self.log_file_name = "log{}.txt".format(datetime.datetime.now())
    
    def spoke_callback(self, msg):
        # type:(String)->None
        with open(self.log_path + self.log_file_name, "a") as f:
            f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + msg.data + "\n")
    
    def heard_callback(self, msg):
        # type:(String)->None
        with open(self.log_path + self.log_file_name, "a") as f:
            f.write(str(datetime.datetime.now()) + "\t" + "robot heard:" + msg.data + "\n")


if __name__ == '__main__':
    Log()
    rospy.spin()
