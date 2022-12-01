#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from eric_a_navigation.srv import Destination, DestinationResponse
from std_srvs.srv import EmptyRequest, Empty

class Dest():
    start=(-3,2,0)
    middle=(-4,2,0)
    final=(-2,1.5,0)

class MoveClient():
    def __init__(self):
        self.srvs = [] 
        self.srvs.append(rospy.Service('dest', Destination, self.handle_track)) 
        self.actionclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
    def euler2quat(self):
        self.qx = np.sin(0) * np.cos(0) * np.cos(self.theta) - np.cos(0) * np.sin(0) * np.sin(self.theta)
        self.qy = np.cos(0) * np.sin(0) * np.cos(self.theta) + np.sin(0) * np.cos(0) * np.sin(self.theta)
        self.qz = np.cos(0) * np.cos(0) * np.sin(self.theta) - np.sin(0) * np.sin(0) * np.cos(self.theta)
        self.qw = np.cos(0) * np.cos(0) * np.cos(self.theta) + np.sin(0) * np.sin(0) * np.sin(self.theta)
        self.movebase_client()

    def movebase_client(self):
        self.costmap_clear()
        self.actionclient.wait_for_server()

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
            rospy.loginfo(self.actionclient.get_state())
            # self.costmap_clear()
            # if client.get_state()!=3:
            state= self.actionclient.get_state() 
            if state == 3:
                return self.finish()
            else:
                return self.restart()

    def restart(self):
        rospy.sleep(2)
        self.movebase_client()

    def finish(self):
        try:    
            start_clear = rospy.ServiceProxy(' ', Empty)
            return start_clear()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        pass

    def costmap_clear(self):
        try:    
            start_clear = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
            return start_clear()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def handle_track(self, req):
        if req.dest=='middle':
            rospy.loginfo("middle")
            self.x=Dest.middle[0]
            self.y=Dest.middle[1]
            self.theta=Dest.middle[2] 
            self.euler2quat()           
            return DestinationResponse('middle')

        elif req.dest=='final':
            rospy.loginfo("final")
            self.x=Dest.final[0]
            self.y=Dest.final[1]
            self.theta=Dest.final[2]
            self.euler2quat()
            return DestinationResponse('final')

        elif req.dest=='start':
            rospy.loginfo("start")
            self.x=Dest.start[0]
            self.y=Dest.start[1]
            self.theta=Dest.start[2]
            self.euler2quat()
            return DestinationResponse('start')

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    node=MoveClient()
    node.main()