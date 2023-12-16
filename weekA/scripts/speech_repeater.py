#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool

class Repeater:
    def __init__(self):
        rospy.init_node('speech_repeater', anonymous=True)
        self.stt_sub_node = "/speech_recognition/final_result"
        self.tts_pub_node = "/tts/phrase"
        self.tts_status_node = "/tts/status"
        self.speech_sub = rospy.Subscriber(self.stt_sub_node, String, self.speech_callback)
        self.tts_pub = rospy.Publisher(self.tts_pub_node, String, queue_size=10)
        self.tts_status_pub = rospy.Publisher(self.tts_status_node, Bool, queue_size=10)
        self.tts_status_pub.publish(False)

    def speech_callback(self, data):
        rospy.loginfo("Custom speech: %s", data.data)
        self.tts_pub.publish(data.data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    speech_repeater = Repeater()
    speech_repeater.run()
