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
import threading


class Sphinx:

    def __init__(self):
        rospy.init_node("sphinx")

        self.init_sphinx()
        self.init_ros()

        self.pause()

        signal.signal(signal.SIGINT, signal.SIG_DFL)

    def init_sphinx(self):
        self.dict = rospy.get_param("/{}/dict".format(rospy.get_name()))
        self.gram = rospy.get_param("/{}/gram".format(rospy.get_name()))

        # 音響モデルのディレクトリの絶対パス
        self.model_path = "/usr/local/lib/python2.7/dist-packages/pocketsphinx/model"
        self.dictionary_path = os.path.join(os.path.dirname(
            os.path.abspath(__file__)), "module/dictionary")  # 辞書のディレクトリの絶対パス
        self.speech = None
        self.start = False
        self.result = None
        self.noise_words = self.read_noise_word()

    def init_ros(self):
        topic = "/sound_system/recognition"
        rospy.Service(topic, StringService, self.recognition)
        service = "/sound_system/recognition_stop"
        rospy.Service(service, StringService, self.recognition_stop)

        self.pub = rospy.Publisher("/sound_system/result", String, queue_size=10)

    def read_noise_word(self):
        words = []
        with open(os.path.join(self.dictionary_path, self.gram)) as f:
            for line in f.readlines():
                if "<noise>" not in line:
                    continue
                if "<rule>" in line:
                    continue
                line = line.replace("<noise>", "").replace("=", "").replace(" ", "").replace("\n", "").replace(";", "")
                words = line.split("|")
                # print(words)
        return words

    def resume(self):
        print("== START RECOGNITION ==")
        self.speech = LiveSpeech(
            verbose=False, sampling_rate=8000, buffer_size=2048, no_search=False, full_utt=False,
            hmm=os.path.join(self.model_path, "en-us"),
            lm=False,
            dic=os.path.join(self.dictionary_path, self.dict),
            jsgf=os.path.join(self.dictionary_path, self.gram)
        )

    # 音声認識ストップ ###########################################
    def pause(self):
        print("== STOP RECOGNITION ==")
        self.speech = LiveSpeech(no_search=True)

    # 音声認識を開始 #############################################
    def recognition(self, message):
        # type: (StringServiceRequest) -> StringServiceResponse
        if message.request != '' :
            self.dict = message.request + '.dict'
            self.gram = message.request + '.gram'
            self.noise_word = self.read_noise_word()

        print "dict: ", self.dict
        #print "gram: ", self.gram

        self.start = True
        return StringServiceResponse()

    def recognition_stop(self, message):
        self.pause()
        self.start = False
        self.result = None
        return StringServiceResponse()

    ############################################################

    def multi_thread(self):
        while True:
            if self.start:
                self.resume()
                for text in self.speech:
                    score = text.confidence()
                    print(str(text), score)
                    if score > 0.1 and str(text) not in self.noise_words:
                        text = str(text)
                        self.pause()

                        self.start = False
                        self.result = text
                        self.pub.publish(self.result)  # 音声認識の結果をpublish
                        self.result = None
                        break
                    else:
                        print("**noise**")


if __name__ == "__main__":
    sphinx = Sphinx()
    sphinx.multi_thread()