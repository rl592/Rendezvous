#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
import numpy as np

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool,Float32MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        param_name = rospy.search_param('robot_name')
        self.robot_name = rospy.get_param(param_name)
        rospy.loginfo(str(self.robot_name))
        self.pub = rospy.Publisher('move_status', Bool, queue_size=1)
        self.pub_ab = rospy.Publisher('abortion', Bool, queue_size=1)


        #Create action client
        if self.robot_name == "/":
            self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        else:
            self.client = actionlib.SimpleActionClient(('/' + str(self.robot_name) + '/move_base'), MoveBaseAction)
        rospy.loginfo(self.robot_name + ": Waiting for move_base action server...")

        wait = self.client.wait_for_server(rospy.Duration(1))
        while not wait:
            wait = self.client.wait_for_server(rospy.Duration(1))
        rospy.loginfo(self.robot_name + ": Connected to move base server")
        rospy.loginfo(self.robot_name + ": Starting goals achievements ...")
        

        self.pre_point = np.array([0,0])
        self.nodeList = np.empty([0,2])
        self.nav_goal = None
        self.next_goal = MoveBaseGoal()
        self.move_state = Bool()
        self.abortion = Bool()

        self.abortion.data = True
        self.move_state.data = False
        self.pub.publish(self.move_state)

        self.nodeList = np.array([[0.5,0.5,0], [0,1,0], [0,0,0]])
        self.yaw = np.array([0.5*np.pi, np.pi, 0])
        # self.sendGoal([0.5,0.5,0], self.init1_cb, 0.5*np.pi)

        self.sendGoal(self.nodeList[0], self.done_cb, self.yaw[0])

        self.nodeList = np.delete(self.nodeList, 0, 0)
        self.yaw = np.delete(self.yaw, 0)
        self.movebase_client()
        
    def active_cb(self):
        # rospy.loginfo('Goal goes active')
        pass

    def feedback_cb(self, feedback):

        cur_x, cur_y = feedback.base_position.pose.position.x, feedback.base_position.pose.position.y
        dist = (cur_x - self.cur_goal[0]) ** 2 +  (cur_y - self.cur_goal[1]) ** 2 

        if self.nodeList.shape[0] > 0:
            if dist < 0.2 :
                self.sendGoal(self.nodeList[0], self.done_cb, self.yaw[0])
                self.nodeList = np.delete(self.nodeList, 0, 0)
                self.yaw = np.delete(self.yaw, 0)
        else:
            # if dist < 0.05  :
            #     if not self.move_state.data:
            #         rospy.loginfo(self.robot_name + ": Goal pose reached") 
            #     self.move_state.data = True
            #     self.pub.publish(self.move_state)
            pass

    
    def sub_cb(self, data):
        self.nodeList = np.array(data.data)
        self.nodeList = self.nodeList.reshape(-1,2)
        
        numOfNode = self.nodeList.shape[0]
        self.nodeList = np.insert(self.nodeList, 0, self.pre_point, axis=0)
        self.nodeList = np.insert(self.nodeList, 2, 0, axis=1)

        self.pre_point = self.nodeList[-1][:2]

        self.yaw = np.zeros(numOfNode)
        for i in range(numOfNode):
            self.yaw[i] = np.arctan2(self.nodeList[i+1][1] - self.nodeList[i][1],self.nodeList[i+1][0] - self.nodeList[i][0])

        self.nodeList = np.delete(self.nodeList, 0, 0)


        self.move_state.data = False
        self.pub.publish(self.move_state)

        self.sendGoal(self.nodeList[0], self.done_cb, self.yaw[0])

        self.nodeList = np.delete(self.nodeList, 0, 0)
        self.yaw = np.delete(self.yaw, 0)


    def done_cb(self, status, result):
        

        if status == 2:
            rospy.loginfo(self.robot_name + ": received a cancel request after it started executing, completed execution!")

        if status == 3:
            if self.nodeList.shape[0] > 0:
                # self.sendGoal(self.nodeList[0], self.done_cb, self.yaw[0])
                # self.nodeList = np.delete(self.nodeList, 0, 0)
                # self.yaw = np.delete(self.yaw, 0)
                pass
            else:
                self.move_state.data = True
                self.pub.publish(self.move_state)
                rospy.loginfo(self.robot_name + ": Goal pose reached") 
                pass
    

        if status == 4:
            self.pub_ab.publish(self.abortion)
            rospy.loginfo(self.robot_name + ": Goal pose was aborted by the Action Server")
            # rospy.signal_shutdown(self.robot_name + ": Goal pose aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo(self.robot_name + ": Goal pose has been rejected by the Action Server")
            # rospy.signal_shutdown(self.robot_name + ": Goal pose rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo(self.robot_name + ": Goal pose  received a cancel request before it started executing, successfully cancelled!")

    def sendGoal(self, point, cb, yaw):
        self.cur_goal = point[:2]
        quat_yaw = Quaternion(*(quaternion_from_euler(0, 0, yaw, axes='sxyz')))
        self.nav_goal = Pose(Point(*point),quat_yaw)
        self.next_goal.target_pose.pose = self.nav_goal
        self.nav_goal = None
        if self.robot_name == "/":
            self.next_goal.target_pose.header.frame_id = "map"
        else:
            self.next_goal.target_pose.header.frame_id = str(self.robot_name) + "/map"
        self.next_goal.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(self.next_goal, cb, self.active_cb, self.feedback_cb) 



    def myhook(self):
        self.client.cancel_all_goals()
        rospy.signal_shutdown(self.robot_name + " shutdown!")

    def movebase_client(self):
        self.sub = rospy.Subscriber('nav_goal', Float32MultiArray, self.sub_cb)
        rospy.on_shutdown(self.myhook)
        rospy.spin()


if __name__ == '__main__':
    try:
        MoveBaseSeq()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

