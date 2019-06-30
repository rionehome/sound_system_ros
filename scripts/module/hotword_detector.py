#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import signal
import snowboydecoder
from sound_system.srv import *
from se import SE


class HotwordDetector:

    def __init__(self):
        self.interrupted = False
        self.model = rospkg.RosPack().get_path('sound_system') + "/model/Hey_Ducker.pmdl"
        self.detector = snowboydecoder.HotwordDetector(self.model, sensitivity=0.5)
        self.se = SE()
        self.init_ros()

    def init_ros(self):
        rospy.init_node('sound_system', anonymous=False)

        def signal_handler(signal, frame):
            self.detector.terminate()
            sys.exit()

        signal.signal(signal.SIGINT, signal_handler)

        def hotword_detect(request):
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

        rospy.Service("/hotword/detect", HotwordService, hotword_detect)
        rospy.spin()


if __name__ == '__main__':
    HotwordDetector()
