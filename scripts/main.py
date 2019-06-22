#!/usr/bin/env python
# -*- coding: utf-8 -*-

import signal
import threading

import rospy
import rospkg
from sound_system.srv import *

from module.julius import Julius
from module.sphinx import Sphinx
from module.svox import Svox
from std_msgs.msg import String
import os


class Main:

    def __init__(self):
        # self.julius = Julius(port=10500, config="navigation_nlp.jconf", is_debug=False)

        # self.sphinx = Sphinx(rospy.get_param("/sound_system/sphinx_config"))
        self.svox = Svox()

        log_file = "{}/{}".format(rospkg.RosPack().get_path("sound_system"), "logger.log")
        if os.path.exists(log_file):
            os.remove(log_file)
        self.log = open(log_file, "a")

        rospy.init_node("sound_system", anonymous=False)
        rospy.Service("/sound_system/speak", StringService, self.to_speak)

        # トピック名を設定
        self.hotword_topic = "/hotword/detect"
        self.nlp_topic = "/sound_system/nlp"
        self.speak_topic = "/sound_system/speak"
        self.recognition_topic = "/sound_system/recognition"

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
            try:
                # Hotword 検出処理
                rospy.wait_for_service(self.hotword_topic, timeout=1)
                print("Hotword 待機...")
                rospy.ServiceProxy(self.hotword_topic, HotwordService)()
                print("Hotword 受信")
                print("音声認識 開始")
                rospy.wait_for_service(self.recognition_topic, timeout=1)
                text = rospy.ServiceProxy(self.recognition_topic, StringService)().response
                print("音声認識 終了")
                # 認識内容のログ出力と表示
                print(text)
                self.output_log("sphinx: {}".format(text))

                # 認識したテキストデータを自然言語処理に投げる
                rospy.wait_for_service(self.nlp_topic, timeout=1)
                nlp_result = rospy.ServiceProxy(self.nlp_topic, NLPService)(text)
                speak_text = nlp_result.response

                # スピーク内容のログ出力と表示
                print("response: {}".format(speak_text))
                self.output_log("response: {}".format(speak_text))

                if speak_text:
                    rospy.wait_for_service(self.speak_topic, timeout=1)
                    rospy.ServiceProxy(self.speak_topic, StringService)(speak_text)

                # 全体に投げる用
            except rospy.ROSException:
                print("接続エラー")

    def output_log(self, text):
        self.log.write("{}\n".format(text))
        self.log.flush()

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


if __name__ == "__main__":
    Main()
