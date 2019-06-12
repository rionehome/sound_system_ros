#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal
import threading

import rospy
from sound_system.srv import *

from module.julius import Julius
from module.svox import Svox
from std_msgs.msg import String


class Main:

    def __init__(self):
        self.julius = Julius(port=10500, config="navigation_nlp.jconf", is_debug=False)
        self.svox = Svox()

        rospy.init_node("sound_system", anonymous=False)
        self.publisher = rospy.Publisher("/sound_system/recognition", String, queue_size=10)
        rospy.Service("/sound_system/speak", StringService, self.to_speak)

        # マルチスレッドで動かす
        self.thread_stop = threading.Event()
        self.thread = threading.Thread(target=self.multi_thread)
        self.thread.start()

        def signal_handler(signal, frame):
            self.thread_stop.set()
            self.thread_stop.wait()
            sys.exit()

        signal.signal(signal.SIGINT, signal_handler)
        rospy.spin()

    def multi_thread(self):
        while not self.thread_stop.is_set():
            hotword_topic = "/hotword/detect"
            nlp_topic = "/sound_system/nlp"
            speak_topic = "/sound_system/speak"
            try:
                # Hotword 検出処理
                rospy.wait_for_service(hotword_topic, timeout=1)
                print("Hotword 待機...")
                rospy.ServiceProxy(hotword_topic, HotwordService)()
                print("Hotword 受信")

                # Juliusで音声認識処理
                self.julius.resume()
                text = self.julius.recognition()
                self.julius.pause()
                print(text)
                # 認識したテキストデータを自然言語処理に投げる
                rospy.wait_for_service(nlp_topic, timeout=1)
                nlp_result = rospy.ServiceProxy(nlp_topic, NLPService)(text)
                print("response: {}".format(nlp_result.response))
                speak_text = nlp_result.response
                if speak_text:
                    rospy.wait_for_service(speak_topic, timeout=1)
                    rospy.ServiceProxy(speak_topic, StringService)(speak_text)
                # 全体に投げる用
                self.publisher.publish(text)
                print(text)
            except rospy.ROSException:
                print("接続エラー")

    def to_speak(self, message):
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


if __name__ == '__main__':
    Main()
