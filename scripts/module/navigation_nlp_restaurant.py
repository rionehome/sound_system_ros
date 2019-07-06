#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sound_system.srv import *
from location.srv import *
from location.msg import *
from std_msgs.msg import String

DISTANCE_RANGE = 1.0


class NavigationNLP:

    def __init__(self):
        rospy.init_node("nlp")
        rospy.Service("/sound_system/nlp", NLPService, self.analyze)
        self.move_command_publisher_kitchen = rospy.Publisher("/navigation/move_command", Location, queue_size=10)
        self.move_command_publisher_table = rospy.Publisher("/navigation_human_detect/move_command", Location, queue_size=10)
        self.follow_command_publisher = rospy.Publisher("/follow_me/control", String, queue_size=10)

        self.register_topic = "/navigation/register_current_location"
        self.request_location_topic = "/navigation/request_location"
        self.request_current_topic = "/navigation/request_current_location"
        self.request_location_list_topic = "/navigation/request_location_list"

        rospy.spin()

    def analyze(self, message):
        # type: (NLPServiceRequest) -> NLPServiceResponse
        """
        ROSサービスサーバー関数
        音声認識で認識した言語の処理を行う
        :param message: 認識したテキスト
        :return: 発話文章 (発話する必要がないなら文は空でいい)
        """

        # Juliusだとスペースが使えないのでアンダーバーで代用しているのでその部分を再変換
        text = message.request.replace("_", " ")
        answer = ""

        print(text)

        if "Please go to" in text:
            # 指定場所への移動命令
            location_name = text.replace("Please go to ", "")
            if location_name is not None:
                try:
                    print location_name
                    rospy.wait_for_service(self.request_location_topic, timeout=1)
                    request_location = rospy.ServiceProxy(self.request_location_topic, RequestLocation)(
                        location_name).location
                    if request_location.name == location_name:
                        answer = "I go to the {}.".format(location_name)
                        if location.name=="kitchen":
                            self.move_command_publisher_kitchen.publish(request_location)
                        elif location.name=="table":
                            self.move_command_publisher_table.publish(request_location)
                    else:
                        answer = "Sorry, I don't know where {} is.".format(location_name)
                except rospy.ROSException:
                    answer = "Error, not find location node."
            return NLPServiceResponse(answer, False)

        return NLPServiceResponse("", False)

    def answer_current_location(self):
        # type: () -> str
        """
        現在位置をテキストにする関数
        半径1m以内に登録座標がある      IN
        1m以内にない場合は最寄りの場所   NEAREST
        場所が1つも登録されていない      DON'T KNOW
        :return: 現在の場所に関するテキストデータ
        """
        # 現在位置を取得
        try:
            rospy.wait_for_service(self.request_current_topic, timeout=1)
            request_current_location = rospy.ServiceProxy(self.request_current_topic, RequestCurrentLocation)()
            current_location = request_current_location.location  # type: Location
        except rospy.ROSException:
            return "Error, not find location node."

        # 現在登録されている場所情報をすべて取得
        try:
            rospy.wait_for_service(self.request_current_topic)
            request_locations = rospy.ServiceProxy(self.request_location_list_topic, RequestLocationList)()
            locations = request_locations.locations  # type: list
        except rospy.ROSException:
            return "Error, not find location node."

        # 現在どこにいるかの判定
        answer = None
        if locations:
            nearest = None
            distance = 99999
            for location in locations:  # type: Location
                calc_distance = (pow(current_location.x - location.x, 2)
                                 + pow(current_location.y - location.y, 2)
                                 + pow(current_location.z - location.z, 2))
                if calc_distance < distance:
                    distance = calc_distance
                    nearest = location
            if math.sqrt(distance) < DISTANCE_RANGE:
                # 指定範囲以内なのでIN
                answer = "I am in the {}.".format(nearest.name)
            else:
                # 指定範囲より遠いので NEAREST
                answer = "Sorry, I only know {} is the nearest.".format(nearest.name)
        else:
            # 何も情報がないので DON'T KNOW
            answer = "Sorry, I don't know where I am."
        return answer


if __name__ == '__main__':
    NavigationNLP()