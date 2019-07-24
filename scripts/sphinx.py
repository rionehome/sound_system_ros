#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 音声認識

import rospy
import rospkg
from std_msgs.msg import String
import os
from pocketsphinx import LiveSpeech
from sound_system.srv import *
import signal
from module.se import SE


class Sphinx:
    
    def __init__(self):
        rospy.init_node("sphinx")
        
        # launchのパラメータ読み取り
        self.dict = rospy.get_param("/{}/dict".format(rospy.get_name()))
        self.gram = rospy.get_param("/{}/gram".format(rospy.get_name()))
        
        # 音響モデルのディレクトリの絶対パス
        self.model_path = "/usr/local/lib/python2.7/dist-packages/pocketsphinx/model"
        self.dictionary_path = rospkg.RosPack().get_path('sound_system') + "/etc/dictionary"  # 辞書のディレクトリの絶対パス
        self.speech = None
        self.start = False
        self.result = None
        self.noise_words = []
        self.se = SE()
        
        rospy.Service("/sound_system/recognition", StringService, self.recognition)
        rospy.Service("/sound_system/recognition_stop", StringService, self.recognition_stop)
        
        self.log_heard_pub = rospy.Publisher("/sound_system/log/heard", String, queue_size=10)
        self.result_pub = rospy.Publisher("/sound_system/result", String, queue_size=10)
        self.pause()
        
        signal.signal(signal.SIGINT, signal.SIG_DFL)
    
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
        self.start = False
    
    # 音声認識を開始 #############################################
    def recognition(self, message):
        # type: (StringServiceRequest) -> StringServiceResponse
        if message.request != '':
            self.dict = message.request + '.dict'
            self.gram = message.request + '.gram'
            self.noise_words = self.read_noise_word()
        
        print "dict: ", self.dict
        print "gram: ", self.gram
        
        self.start = True
        return StringServiceResponse()
    
    def recognition_stop(self, message):
        self.pause()
        return StringServiceResponse()
    
    ############################################################
    
    def multi_thread(self):
        """
        マルチスレッド
        メインスレッドでしかResumeできないSphinxへの対策
        self.start, self.resultでフラグ管理を行っている
        self.startが処理の開始用フラグ
        :return:
        """
        while True:
            # 音声認識の命令が来るまで待機
            if not self.start:
                continue
            
            self.se.play(self.se.START)
            self.resume()
            for text in self.speech:
                score = text.confidence()
                print(str(text), score)
                if score > 0.1 and str(text) not in self.noise_words:
                    self.se.play(self.se.STOP)
                    self.pause()
                    text = str(text)
                    self.result = text
                    self.log_heard_pub.publish(text)
                    self.result_pub.publish(self.result)
                    break
                else:
                    print("**noise**")


if __name__ == "__main__":
    sphinx = Sphinx()
    sphinx.multi_thread()
