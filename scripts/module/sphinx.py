#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Follow me, 音声認識

import rospy
from std_msgs.msg import String, Bool
import os
import sys
from pocketsphinx import LiveSpeech


class Sphinx:

    def __init__(self, file_name):
        self.speech = None
        self.model_path = '/usr/local/lib/python2.7/dist-packages/pocketsphinx/model'  # 音響モデルのディレクトリの絶対パス
        self.dictionary_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dictionary')  # 辞書のディレクトリの絶対パス
        self.file_name = file_name

    def resume(self):
        print('== START RECOGNITION ==')
        self.speech = LiveSpeech(
            verbose=False, sampling_rate=8000, buffer_size=2048, no_search=False, full_utt=False,
            hmm=os.path.join(self.model_path, 'en-us'),
            lm=False,
            dic=os.path.join(self.dictionary_path, "{}.dict".format(self.file_name)),
            jsgf=os.path.join(self.dictionary_path, "{}.gram".format(self.file_name))
        )

    # 音声認識ストップ
    def pause(self):
        print('== STOP RECOGNITION ==')
        self.speech = LiveSpeech(no_search=True)

    # 音声認識結果の表示
    def recognition(self):
        for text in self.speech:
            score = text.confidence()
            print(text, score)
            print(score)
            if score > 0.1:
                text = str(text)
                # self.pub.publish(text)  # 音声認識の結果をpublish
                # self.pause()
                return text
            else:
                print("**noise**")


if __name__ == '__main__':
    sphinx = Sphinx()
    for i in range(5):
        sphinx.resume()
        print("Result: {}".format(sphinx.recognition()))
        sphinx.pause()