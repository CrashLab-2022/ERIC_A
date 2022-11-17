#!/usr/bin/env python3

import rospy
import rospkg
from move_base_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from eric_a_navigation.srv import ClassNumber, ClassNumberResponse
from std_srvs.srv import Empty, EmptyRequest
import actionlib
import os
from eric_a_classnumber import classnumber

# class fileRoot(object):
#     pkg_path = rospkg.RosPack()
#     speak_path = pkg_path.get_path('eric_a_speak')
    # web = pkg_path.get_path('hongdo_ros_web')



# class SoundStart:
#     _voice_path=None
#     _sound_path=None

#     def __init__(self, vocie_path = None, sound_path = None):
#         self._voice_path=vocie_path
#         self._sound_path=sound_path

#     def play_voice(self, req):
#         rospy.loginfo("voice :  %s"%req.sequence)
#         os.system('play ' + self._voice_path +'/%s.mp3'%req.sequence)
#         return PlaySongResponse('finish')

#     def play_sound(self,req):
#         rospy.loginfo("background sound :  %s"%req.sequence)
#         os.system('play ' + self._sound_path +'/%s.mp3'%req.sequence)
#         return PlaySongResponse('finish')

    # def __del__(self):
    #     self._voice_path=None
    #     self._sound_path=None

class EricAStartNode:
    def __init__(self):
        #sound
        
        # voice_path = fileRoot.webconnect + '/speak/voice'
        # sound_path = fileRoot.webconnect + '/speak/sound'

        #storaging
        # self.sound_start=SoundStart(voice_path, sound_path)
        

        ##service
        rospy.Service('classnumber',ClassNumber , self.purpose_class_set)
        
        # rospy.Service('/play_voice', PlaySong, self.sound_start.play_voice)
        
        #action
        self.goalclient = actionlib.ActionClient('move_base/goal', MoveBaseAction)
        
        #timer
        # self.move_base_goal.header.stamp = rospy.Time.now()

        # self.goalclient.wait_for_server()
        # self.opencvStart()
        rospy.loginfo('Ready to eric_a_speak Node')
        rospy.on_shutdown(self.__del__)

    def purpose_class_set(self, req):
        ## web to robot
        # self.classnum = classnumber.class
        # self.classnum=[]
        if req.number ==101:
            self.classnum = classnumber.class101
            
        elif req.number == 102:
            self.classnum = classnumber.class102
        elif req.number == 103:
            self.classnum = classnumber.class103

        self.update_nav_goal()
        return ClassNumberResponse('finish')

    def update_nav_goal(self):
        id_number = 0
        x = 0 #self.classnum[0]
        y = 0# self.classnum[1]
        thetha = 0 #self.classnum[2]
        purpose = Point(x,y,0)
        goal_orientation_quat = quaternion_from_euler(0,0, thetha)
        
        
        move_base_goal = MoveBaseActionGoal()

        move_base_goal.header.stamp = rospy.Time.now()
        move_base_goal.goal_id.id = id_number
        move_base_goal.goal.target_pose.pose = Pose(purpose,Quaternion(*goal_orientation_quat))
        self.goalclient.send_goal(move_base_goal)
        # self.goalclient.wait_for_result()
        print('hijj')
        action_result = self.goalclient.
        if action_result == 3:
            self.costmap_clear()
        else: 
            pass
        return self.goalclient.get_result()
        


    def costmap_clear(self):
        try:
            start_clear = rospy.ServiceProxy('clear_costmaps', Empty)
            return start_clear()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def main(self):
        rospy.loginfo("hi")
        rospy.spin()

    def __del__(self):
        print("[info]terminating eric_a_speak_node")



if __name__ == '__main__':
    rospy.init_node('eric_a_start_node', anonymous=True)
    node=EricAStartNode()
    node.main()


    #출발 직전 costmap clear, 목적지 goal, 출발명령받기