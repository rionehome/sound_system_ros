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
        self.julius = Julius(port=10500, config="navigation_nlp.jconf")
        self.svox = Svox()

        rospy.init_node("sound_system", anonymous=False)
        self.publisher = rospy.Publisher("/sound_system/recognition", String, queue_size=10)

        def multi_thread():
            while not thread_stop.is_set():
                hotword_topic = "/hotword/detect"
                nlp_topic = "/sound_system/nlp"
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

                    # 認識したテキストデータを自然言語処理に投げる
                    rospy.wait_for_service(nlp_topic, timeout=1)
                    nlp_result = rospy.ServiceProxy(nlp_topic, NLPService)(text)
                    print("response: {}".format(nlp_result.response))
                    if nlp_result.response:
                        self.svox.speak(nlp_result.response)

                    # 全体に投げる用
                    self.publisher.publish(text)
                    print(text)
                except rospy.ROSException:
                    print("接続エラー")

        # マルチスレッドで動かす
        thread_stop = threading.Event()
        thread = threading.Thread(target=multi_thread)
        thread.start()

        def signal_handler(signal, frame):
            thread_stop.set()
            thread_stop.wait()
            sys.exit()

        signal.signal(signal.SIGINT, signal_handler)

        rospy.spin()


if __name__ == '__main__':
    Main()
