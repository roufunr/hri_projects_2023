#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool

class Questionnaire:
    def __init__(self):
        rospy.init_node('speech_repeater', anonymous=True)
        self.stt_sub_node = "/speech_recognition/final_result"
        self.questions = [
            "do you like data structure?",
            "do you like algorithm?",
            "do you know about time complexity?"
        ]
        self.question_state = 0
        self.user_feedback = []
        self.speech_sub = rospy.Subscriber(self.stt_sub_node, String, self.speech_callback)
        self.ask_a_question()
        
    def handle_response(self): 
        yesCount = 0
        for fb in self.user_feedback:
            if "yes" in fb:
                yesCount += 1
        
        self.user_feedback.clear()
        self.question_state = 0

        if yesCount == 0: 
            rospy.loginfo("I believe, you don't like computer science!")
        elif yesCount == 3:
            rospy.loginfo("I believe, you like computer science!")
        else:
            rospy.loginfo("I believe, you partially like computer science!")
        
        rospy.loginfo("#################################################")
        rospy.loginfo("#############START AGAIN###################")
        self.ask_a_question()
        

    def ask_a_question(self):
        rospy.loginfo(self.questions[self.question_state])
        self.question_state += 1

    def speech_callback(self, data):
        respone = data.data
        rospy.loginfo("We heard: %s", respone)
        if ("yes" in respone)  or ("no" in respone):
            self.user_feedback.append(respone)
            if self.question_state < 3:
                self.ask_a_question()
            else: 
                self.handle_response()
        else: 
            rospy.loginfo("Just say 'YES/NOT'")
        
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    questionaire = Questionnaire()
    questionaire.run()