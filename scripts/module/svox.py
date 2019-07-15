#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import rospkg


class Svox:

    def __init__(self):
        self.file = rospkg.RosPack().get_path('sound_system') + "/voice/voice.wav"

    def speak(self, text):
        """
        svoxに喋らせる関数
        しゃべり終わるまで、待機
        :param text: textで投げる
        """
        create_command = ["pico2wave", "-w=" + self.file, text]
        play_command = ["aplay", self.file]
        subprocess.call(create_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        subprocess.call(play_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
