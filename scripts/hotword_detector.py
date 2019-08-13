#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from module.hotword import snowboydecoder
from sound_system.srv import *
from module.se import SE


class HotwordDetector:
    
    def __init__(self):
        self.interrupted = False
        self.model = rospkg.RosPack().get_path('sound_system') + "/etc/model/Hey_Ducker.pmdl"
        self.detector = snowboydecoder.HotwordDetector(self.model, sensitivity=0.5)
        self.se = SE()
        
        rospy.init_node('hotword', anonymous=False)
        rospy.Service("/sound_system/hotword", HotwordService, self.hotword_detect)
    
    def hotword_detect(self, request):
        """
        Hotwordの検出を行う
        :param request: HotwordRequest型だが中身は何もなし
        :return: HotwordResponse型だが中身は何もなし
        """
        print("Hotword 待機...")
        is_detect = self.detector.start()
        self.detector.terminate()
        if is_detect:
            self.se.play(self.se.WAKEUP)
            print("Hotword 検出")
        return HotwordServiceResponse()


if __name__ == '__main__':
    HotwordDetector()
    rospy.spin()
