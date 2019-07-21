#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal
import threading

import rospy
import rospkg
from sound_system.srv import *

from module.svox import Svox


class Main:

    def __init__(self):
        self.svox = Svox()

        rospy.init_node("speak", anonymous=False)
        rospy.Service("/sound_system/speak",
                      StringService, self.speak_callback)

        def signal_handler(signal, frame):
            self.thread_stop.set()
            self.thread_stop.wait()
            sys.exit()

        signal.signal(signal.SIGINT, signal_handler)

        rospy.spin()

    def speak_callback(self, message):
        # type: (StringServiceRequest) -> StringServiceResponse
        """
        ROSサービスサーバー関数
        テキストデータを投げたら、読み上げる
        :param message: 中身なし
        :return: なし
        """
        text = message.request
        if text:
            self.svox.speak(text)
        return StringServiceResponse()


if __name__ == "__main__":
    Main()
