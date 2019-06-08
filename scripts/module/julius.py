#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospkg
import rospy
from time import sleep
from atexit import register
from threading import Thread
import subprocess
from socket import socket, AF_INET, SOCK_STREAM
from re import compile
from atexit import register
from std_msgs.msg import Bool, String
from sound_system.srv import *
import signal

RECOGOUT_START = "<RECOGOUT>"
RECOGOUT_END = "</RECOGOUT>"
REJECT = "<REJECTED"
WHYPO_WORD = "<WHYPO"
WORD = "WORD="
PACKAGE = rospkg.RosPack().get_path('sound_system')
DEFAULT_CONFIG = "default.jconf"


class Julius:
    def __init__(self, host="localhost", port=10500, config=None, is_debug=False):
        self.is_debug = is_debug
        self.host = host
        self.port = port
        self.client = None
        self.process = None
        self.active = True
        if config is None:
            config = DEFAULT_CONFIG
        self.boot(config)
        sleep(1)
        self.connect()

        def exit(signal, frame):
            print("\n process exit [PID=%d]" % self.process.pid)
            self.process.kill()
            sys.exit(0)

        signal.signal(signal.SIGINT, exit)

    def boot(self, config):
        # type: (str) -> None
        command = [PACKAGE + "/julius/boot.sh", config, str(self.port)]
        if self.is_debug:
            self.process = subprocess.Popen(command)
        else:
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def resume(self):
        self.client.send("RESUME\n")
        self.active = True

    def pause(self):
        self.client.send("TERMINATE\n")
        self.active = False

    def recognition(self):
        # type: ()-> str
        data = " "
        is_recogout = False
        while True:
            recv = self.client.recv(2048).decode("utf-8")
            if self.is_debug:
                print(recv)
            if not len(recv) > 0:
                continue
            if data[-1] is "\n":
                data = recv
            else:
                data += recv
            if not data[-1] == "\n":
                continue
            lines = data.split("\n")
            for line in lines:
                if not len(line) > 1:
                    continue
                sentences = line.lstrip().split()
                if not len(sentences) > 0:
                    continue
                if sentences[0] == RECOGOUT_START:
                    is_recogout = True
                if (sentences[0] == REJECT) or (is_recogout and sentences[0] == RECOGOUT_END):
                    continue
                    # return None
                if sentences[0] == WHYPO_WORD:
                    for sentence in sentences:
                        if WORD in sentence:
                            return sentence.split('"')[1].replace("_", " ")

    def connect(self):
        try:
            self.client = socket(AF_INET, SOCK_STREAM)
            self.client.connect((self.host, self.port))
            print("Connect Successfully")
            self.pause()
        except IOError:
            sys.exit(1)

    @staticmethod
    def start():
        rospy.spin()
