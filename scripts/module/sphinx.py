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


class Sphinx:
    
    def __init__(self):
        
        rospy.init_node("sphinx")
        
        # ctrl+cをキャッチ
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        
        self.se = SE()
        
        # sphinxの設定をlaunchのパラメーターから取得
        self.dict = rospy.get_param("/{}/dict".format(rospy.get_name()))
        self.gram = rospy.get_param("/{}/gram".format(rospy.get_name()))
        
        # 音響モデルのディレクトリの絶対パス
        self.model_path = "/usr/local/lib/python2.7/dist-packages/pocketsphinx/model"
        # 辞書のディレクトリの絶対パス
        self.dictionary_path = "{}/{}".format(rospkg.RosPack().get_path('sound_system'), "sphinx_dictionary")
        
        self.speech = None
        self.start = False
        self.result = None
        
        rospy.Service("/sound_system/recognition", StringService, self.recognition)
        rospy.Subscriber("/sound_system/sphinx/dict", String, self.change_dict)
        rospy.Subscriber("/sound_system/sphinx/gram", String, self.change_gram)
        self.log_heard_pub = rospy.Publisher("/sound_system/log/heard", String, queue_size=10)
        # noiseになるwordをリストアップ
        self.noise_words = self.read_noise_word()
        
        # <削除予定> <ここから> Serviceに全て移行して欲しいが不可能なのでとりあえず用意、使用は非推奨
        rospy.Subscriber("/sound_system/recognition_start", Bool, self.recognition_start)
        self.result_pub = rospy.Publisher("/sound_system/recognition_result", String, queue_size=10)
        # <削除予定> <ここまで> Serviceに全て移行して欲しいが不可能なのでとりあえず用意、使用は非推奨
        
        self.multi_thread()
    
    def read_noise_word(self):
        words = []
        with open(os.path.join(self.dictionary_path, self.gram)) as f:
            for line in f.readlines():
                if "<noise>" not in line:
                    continue
                if "<rule>" in line:
                    continue
                words = line.replace("<noise>", "").replace("=", "").replace(" ", "").replace("\n", "").replace(";",
                                                                                                                "").split(
                    "|")
                print(words)
        return words
    
    def change_dict(self, message):
        # type: (String) -> None
        """
        ROS Subscriber関数
        受け取ったテキスト名のファイルをdictに設定
        :param message: ファイル名
        :return:
        """
        self.dict = message.data
    
    def change_gram(self, message):
        # type: (String) -> None
        """
        ROS Subscriber関数
        受け取ったテキスト名のファイルをgramに設定
        :param message:
        :return:
        """
        self.gram = message.data
    
    def resume(self):
        """
        音声認識の再開
        :return: ファイル名
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
        # type: (StringServiceRequest) -> StringServiceResponse
        """
        音声認識の開始
        本来なら、ここでSphinxをResumeすればいいのだが、Sphinxの仕様でResumeはメインスレッドでしか出来ない
        よってここでは、フラグでマルチスレッドに処理をおこなわせ、結果が返ってくるまで待機させる
        self.resultを必ずreturn前で初期化しないと、以降の処理で同じ認識結果しか返ってこない
        :param message:
        :return:
        """
        self.se.play(self.se.START)  # beep
        self.start = True
        while not self.result:
            pass
        result = self.result
        # log書き込み
        self.log_heard_pub.publish(self.result)
        self.result = None
        self.se.play(self.se.STOP)  # beep
        return StringServiceResponse(result)
    
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
            if self.start:
                print "start"
                self.resume()
                print "resumed"
                for text in self.speech:
                    score = text.confidence()
                    print(str(text), score)
                    if score > 0.1 and str(text) not in self.noise_words:
                        text = str(text)
                        # self.pub.publish(text)  # 音声認識の結果をpublish
                        # self.pause()
                        self.pause()
                        self.start = False
                        self.result = text
                        self.log_heard_pub.publish(text)
                        self.result_pub.publish(self.result)  # <-削除予定
                        break
                    else:
                        print("**noise**")
    
    def recognition_start(self, message):
        # type: (Bool) -> None
        """
        <削除予定>
        使用非推奨
        recognition関数のSubscriberバージョン
        Serviceに移行が済み次第削除
        :param message: Trueで音声認識開始
        :return:
        """
        self.start = message.data


if __name__ == "__main__":
    Sphinx()
