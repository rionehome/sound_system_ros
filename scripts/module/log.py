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
        
        self.path = os.path.dirname(os.path.abspath(__file__))
        self.log_path = "{}/{}".format(rospkg.RosPack().get_path('sound_system'), "log")
        self.log_file_name = "log{}.txt".format(datetime.datetime.now())
    
    def spoke_callback(self, msg):
        with open(self.log_path + self.log_file_name, "a") as f:
            f.write(str(datetime.datetime.now()) + "\t" + "robot spoke:" + msg + "\n")
    
    def heard_callback(self, msg):
        with open(self.log_path + self.log_file_name, "a") as f:
            f.write(str(datetime.datetime.now()) + "\t" + "robot heard:" + msg + "\n")


if __name__ == '__main__':
    Log()
    rospy.spin()
