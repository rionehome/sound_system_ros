#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import rospkg
import rospy
from sound_system.srv import *
from std_msgs.msg import Bool


class Svox:

    def __init__(self):
        self.file = rospkg.RosPack().get_path('sound_system') + "/etc/voice/voice.wav"

    def speak(self, text):
        """
        :param text: SVOXに喋らせる言葉をStringで投げる
        """
        create_command = ["pico2wave", "-w=" + self.file, text]
        play_command = ["aplay", self.file]
        subprocess.call(create_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        subprocess.call(play_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
