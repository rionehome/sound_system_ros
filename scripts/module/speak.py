#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal
import rospy
from sound_system.srv import *
from std_msgs.msg import String

from svox import Svox


class Main:

    def __init__(self):
        self.svox = Svox()

        signal.signal(signal.SIGINT, signal.SIG_DFL)

        rospy.init_node("speak", anonymous=False)

        self.log_spoke_pub = rospy.Publisher("/sound_system/log/spoke", String, queue_size=10)

        rospy.Service("/sound_system/speak", StringService, self.speak_callback)

    def speak_callback(self, message):
        # type: (StringServiceRequest) -> StringServiceResponse
        """
        ROSサービスサーバー関数
        テキストデータを投げたら、読み上げる
        :param message: テキストデータ
        :return: なし
        """
        text = message.request
        if text:
            text = text.replace("_", " ")
            self.log_spoke_pub.publish(text)
            self.svox.speak(text)
        return StringServiceResponse()


if __name__ == "__main__":
    Main()
    rospy.spin()
