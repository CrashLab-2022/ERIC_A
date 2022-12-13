#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import Empty, EmptyRequest
from eric_a_navigation.srv import Speak, SpeakRequest

class Dest():
    start=(0,0,0)
    middle=(-4,2,0)
    final=(-2,1.5,0)

class MoveClient():
    def __init__(self):
        self.cnt=0
        self.srvs = [] 
        self.srvs.append(rospy.Service('start', Trigger, self.start)) 
        self.srvs.append(rospy.Service('middle', Trigger, self.middle)) 
        self.srvs.append(rospy.Service('final', Trigger, self.final)) 
        self.actionclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.timer=rospy.Timer(rospy.Duration(5), self.clear_callback)

    def euler2quat(self):
        self.qx = np.sin(0) * np.cos(0) * np.cos(self.theta) - np.cos(0) * np.sin(0) * np.sin(self.theta)
        self.qy = np.cos(0) * np.sin(0) * np.cos(self.theta) + np.sin(0) * np.cos(0) * np.sin(self.theta)
        self.qz = np.cos(0) * np.cos(0) * np.sin(self.theta) - np.sin(0) * np.sin(0) * np.cos(self.theta)
        self.qw = np.cos(0) * np.cos(0) * np.cos(self.theta) + np.sin(0) * np.sin(0) * np.sin(self.theta)
        self.movebase_client()

    def movebase_client(self):
        # self.actionclient.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = self.qx
        goal.target_pose.pose.orientation.y = self.qy
        goal.target_pose.pose.orientation.z = self.qz
        goal.target_pose.pose.orientation.w = self.qw
        self.actionclient.send_goal(goal)
        wait = self.actionclient.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        
        else:
            state= self.actionclient.get_state()

            if state == 3:
                # self.costmap_clear()
                # rospy.loginfo("self.cnt")
                # rospy.loginfo(self.cnt)

                if self.cnt==0:
                    self.cnt+=1
                    self.web_client_1()
                    
                elif self.cnt==1: 
                    self.cnt+=1
                    self.web_client_2()
                    
                elif self.cnt==2: 
                    self.web_client_3()

            # else:
            #     return self.restart()

    # def restart(self):
    #     rospy.loginfo("restart")
    #     rospy.sleep(5)
    #     self.movebase_client()

    def clear_callback(self, timer):
        self.costmap_clear()

    def web_client_1(self):
        self.speakclient(3)
        web_service=rospy.ServiceProxy('middle_arrive', Trigger)
        return web_service()
        
    def web_client_2(self):
        self.speakclient(4)
        web_service=rospy.ServiceProxy('final_arrive', Trigger)
        return web_service()

    def web_client_3(self):
        self.speakclient(5)
        web_service=rospy.ServiceProxy('start_arrive', Trigger)
        return web_service()

    def costmap_clear(self):
        try:    
            start_clear = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
            return start_clear()

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def start(self,req):
        self.x=Dest.start[0]
        self.y=Dest.start[1]
        self.theta=Dest.start[2]
        self.euler2quat()
        return TriggerResponse(True,'first finish')

    def middle(self, req):
        self.speakclient(2)
        self.x=Dest.middle[0]
        self.y=Dest.middle[1]
        self.theta=Dest.middle[2]
        self.euler2quat()
        return TriggerResponse(True,'middle finish')

    def final(self, req):
        self.x=Dest.final[0]
        self.y=Dest.final[1]
        self.theta=Dest.final[2]
        self.euler2quat()
        return TriggerResponse(True,'final finish')

    def speakclient(self,num):
        # rospy.wait_for_service('play_song')
        try:
            play=rospy.ServiceProxy('play_song', Speak)
            rospy.loginfo("play song detect")
            return play(num)

        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: ")


if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    cls_=MoveClient()
    rospy.spin()