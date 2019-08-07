#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sound_system.srv import *

from module.svox import Svox


class Speak:
    
    def __init__(self):
        self.svox = Svox()
        
        rospy.init_node("speak", anonymous=False)
        rospy.Service("/sound_system/speak", StringService, self.speak_callback)
    
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
    Speak()
    rospy.spin()
