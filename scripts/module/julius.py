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

    def __init__(self):
        
        rospy.init_node("julius")

        self.config = rospy.get_param("/{}/config".format(rospy.get_name()))
        self.host = rospy.get_param("/{}/host".format(rospy.get_name()))
        self.port = rospy.get_param("/{}/port".format(rospy.get_name()))
        self.is_debug = rospy.get_param("/{}/debug".format(rospy.get_name()))
        self.client = None
        self.process = None
        self.active = True
        if self.config is None:
            self.config = DEFAULT_CONFIG
        print(self.config)
        self.boot(self.config)
        sleep(1)
        self.connect()

        def exit(signal, frame):
            print("\n process exit [PID=%d]" % self.process.pid)
            self.process.kill()
            sys.exit(0)

        signal.signal(signal.SIGINT, exit)

        topic = "/sound_system/recognition"
        rospy.Service(topic, StringService, self.recognition)
        rospy.spin()

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

    def recognition(self, message):
        # type: (StringServiceRequest)-> StringServiceResponse
        self.resume()
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
                            self.pause()
                            return StringServiceResponse(sentence.split('"')[1].replace("_", " "))

    def connect(self):
        try:
            self.client = socket(AF_INET, SOCK_STREAM)
            self.client.connect((self.host, self.port))
            print("Connect Successfully")
            self.pause()
        except IOError:
            sys.exit(1)


if __name__ == '__main__':
    Julius()
