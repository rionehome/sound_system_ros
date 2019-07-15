#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import signal
import snowboydecoder
from sound_system.srv import *


class HotwordDetector:

    def __init__(self):
        self.model = rospkg.RosPack().get_path('sound_system') + "/model/Hey_Ducker.pmdl"
        self.detector = snowboydecoder.HotwordDetector(self.model, sensitivity=0.5)
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        rospy.init_node('sound_system', anonymous=False)

        rospy.Service("/hotword/detect", HotwordService, self.hotword_detect)

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
            print("Hotword 検出")
        return HotwordServiceResponse()


if __name__ == '__main__':
    detector = HotwordDetector()
    rospy.spin()