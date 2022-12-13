#!/usr/bin/env python

import rospy
import rospkg
from eric_a_navigation.srv import Speak, SpeakResponse

class EricAspeakNode:
    def __init__(self):
        rospy.Service('/play_song', Speak, self.play_sound)
        rospy.loginfo('Ready to eric_a_speak Node')
        rospy.on_shutdown(self.__del__)

    def main(self):
        rospy.spin()

    def play_sound(self,req):
        rospy.loginfo("background sound :  %s"%req.sequence)
        return SpeakResponse('finish')

    def __del__(self):
        print("[info]terminating eric_a_speak_node")



if __name__ == '__main__':
    rospy.init_node('eirc_a_speak_node', anonymous=True)
    node= EricAspeakNode()
    node.main()