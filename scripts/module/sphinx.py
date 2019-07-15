#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 音声認識

import rospy
import rospkg
from std_msgs.msg import String, Bool
import os
from pocketsphinx import LiveSpeech
from sound_system.srv import *
import signal
from se import SE
from sound_system.msg import SphinxParam


class Sphinx:

    def __init__(self):
        rospy.init_node("sphinx")

        self.se = SE()

        self.speech = None
        self.start = False
        self.result = None

        # 音響モデルのディレクトリの絶対パス と 辞書のディレクトリの絶対パス
        self.model_path = "/usr/local/lib/python2.7/dist-packages/pocketsphinx/model"
        self.dictionary_path = "{}/{}".format(rospkg.RosPack().get_path('sound_system'), "sphinx_dictionary")

        # sphinxの設定をlaunchのパラメーターから取得
        self.dict = rospy.get_param("/{}/dict".format(rospy.get_name()))
        self.gram = rospy.get_param("/{}/gram".format(rospy.get_name()))
        self.noise_words = self.read_noise_word()

        signal.signal(signal.SIGINT, signal.SIG_DFL)

        rospy.Service("/sound_system/recognition", RecognitionService, self.recognition)

        self.log_heard_pub = rospy.Publisher("/sound_system/log/heard", String, queue_size=10)

        self.multi_thread()

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
                print(words)
        return words

    def resume(self):
        """
        音声認識の再開
        :return: None
        """
        print("== START RECOGNITION ==")
        self.speech = LiveSpeech(
            verbose=False, sampling_rate=8000, buffer_size=2048, no_search=False, full_utt=False,
            hmm=os.path.join(self.model_path, "en-us"),
            lm=False,
            dic=os.path.join(self.dictionary_path, self.dict),
            jsgf=os.path.join(self.dictionary_path, self.gram)
        )

    def pause(self):
        """
        音声認識の一時停止
        :return:
        """
        print("== STOP RECOGNITION ==")
        self.speech = LiveSpeech(no_search=True)

    def recognition(self, message):
        # type: (RecognitionServiceRequest) -> RecognitionServiceResponse
        """
        音声認識の開始
        本来なら、ここでSphinxをResumeすればいいのだが、Sphinxの仕様でResumeはメインスレッドでしか出来ない
        よってここでは、フラグでマルチスレッドに処理をおこなわせ、結果が返ってくるまで待機させる
        self.resultを必ずreturn前で初期化しないと、以降の処理で同じ認識結果しか返ってこない
        :param message: dictとgram
        :return:
        """
        self.dict = message.dict
        self.gram = message.gram
        self.noise_words = self.read_noise_word()

        self.start = True
        while not self.result:
            pass
        result = self.result
        self.result = None
        return RecognitionServiceResponse(result)

    def multi_thread(self):
        """
        マルチスレッド
        メインスレッドでしかResumeできないSphinxへの対策
        self.start, self.resultでフラグ管理を行っている
        self.startが処理の開始用フラグ
        :return:
        """
        while not rospy.is_shutdown():
            # 音声認識の命令が来るまで待機
            if self.start:
                self.se.play(self.se.START)
                self.resume()
                for text in self.speech:
                    score = text.confidence()
                    print(str(text), score)
                    if score > 0.1 and str(text) not in self.noise_words:
                        text = str(text)
                        self.pause()
                        self.se.play(self.se.STOP)
                        self.start = False
                        self.result = text
                        self.log_heard_pub.publish(text)
                        break
                    else:
                        print("**noise**")


if __name__ == "__main__":
    Sphinx()
