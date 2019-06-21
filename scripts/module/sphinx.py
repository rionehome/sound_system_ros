#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 音声認識

import rospy
from std_msgs.msg import String, Bool
import os
import sys
from pocketsphinx import LiveSpeech
from sound_system.srv import *
import signal


class Sphinx:

    def __init__(self):

        rospy.init_node("sphinx")

        self.dict = rospy.get_param("/{}/dict".format(rospy.get_name()))
        self.gram = rospy.get_param("/{}/gram".format(rospy.get_name()))

        self.model_path = "/usr/local/lib/python2.7/dist-packages/pocketsphinx/model"  # 音響モデルのディレクトリの絶対パス
        self.dictionary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dictionary")  # 辞書のディレクトリの絶対パス
        self.speech = None
        self.start = False
        self.result = None
        self.pause()

        def exit(signal, frame):
            print("\n process exit [PID=%d]" % self.process.pid)
            self.process.kill()
            sys.exit(0)

        signal.signal(signal.SIGINT, exit)

        topic = "/sound_system/recognition"
        rospy.Service(topic, StringService, self.recognition)
        self.multi_thread()
        # rospy.spin()

    def resume(self):
        print("== START RECOGNITION ==")
        self.speech = LiveSpeech(
            verbose=False, sampling_rate=8000, buffer_size=2048, no_search=False, full_utt=False,
            hmm=os.path.join(self.model_path, "en-us"),
            lm=False,
            dic=os.path.join(self.dictionary_path, self.dict),
            jsgf=os.path.join(self.dictionary_path, self.gram)
        )

    # 音声認識ストップ
    def pause(self):
        print("== STOP RECOGNITION ==")
        self.speech = LiveSpeech(no_search=True)

    # 音声認識結果の表示
    def recognition(self, message):
        # type: (StringServiceRequest) -> StringServiceResponse
        self.start = True
        while not self.result:
            pass
        result = self.result
        self.result = None
        return StringServiceResponse(result)

    def multi_thread(self):
        while True:
            if self.start:
                self.resume()
                for text in self.speech:
                    score = text.confidence()
                    print(str(text), score)
                    if score > 0.1:
                        text = str(text)
                        # self.pub.publish(text)  # 音声認識の結果をpublish
                        # self.pause()
                        self.pause()
                        self.start = False
                        self.result = text
                        break
                    else:
                        print("**noise**")


if __name__ == "__main__":
    Sphinx()
