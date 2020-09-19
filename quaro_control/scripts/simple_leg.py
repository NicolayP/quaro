#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
import yaml


class SimpleWalker(object):
    def __init__(self, goal=None, goal_file=None):
        rospy.init_node('SimpleWalker')
        self.pub_list = []
        self.goals = []
        self.nb_act = 13
        self.error = np.zeros(self.nb_act)

        self.rate = rospy.Rate(10)
        for i in range(4):
            self.pub_list.append(rospy.Publisher('/quaro/ankle{}_position_controller/command'.format(i), Float64, queue_size=10))
        self.pub_list.append(rospy.Publisher('/quaro/chest_position_controller/command', Float64, queue_size=10))
        for i in range(4):
            self.pub_list.append(rospy.Publisher('/quaro/hip{}_position_controller/command'.format(i), Float64, queue_size=10))
        for i in range(4):
            self.pub_list.append(rospy.Publisher('/quaro/knee{}_position_controller/command'.format(i), Float64, queue_size=10))


        if goal is None and goal_file is None:
            self.goal = np.zeros(self.nb_act)
            self.goals.append(dict('point', self.goal))
        elif goal is not None:
            self.goal = goal
            self.goals.append(dict('point', self.goal))
        else:
            rospy.loginfo("load yaml file")
            self.loadGoals(goal_file)

        rospy.Subscriber("/quaro/joint_states", JointState, self.stateCB)

    def loadGoals(self, file):
        with open(file) as msg:
            self.goals = yaml.load(msg)['waypoints']
        rospy.loginfo("Loaded goals")
        rospy.loginfo(self.goals)
        self.getNextGoal()


    def getNextGoal(self):
        if self.goals:
            self.goal = self.goals.pop()['point']
            rospy.loginfo("new goal: {}".format(self.goal))
            rospy.loginfo("Remaining goals: {}".format(self.goals))
        #otherwise no new goal, keep the last one

    def stateCB(self, msg):
        rospy.loginfo("Goals remaining stateCB")
        rospy.loginfo(self.goals)
        for i, val in enumerate(msg.position):
            self.error[i] = self.goal[i] - val


        if (np.abs(self.error) < 1e-1).all():
            self.getNextGoal()
        #rospy.loginfo(self.error)

    def setGoal(self, goal):
        self.goal = goal


    def run(self):
        # chest
        #if abs(self.error) < 1e-2:
        #    goal = plan.get_next()

        while not rospy.is_shutdown():
            for i in range(self.nb_act):
                self.pub_list[i].publish(Float64(self.goal[i]))
            self.rate.sleep()


def main():
    sw = SimpleWalker(goal_file="/home/pierre/Personal/workspace/quad_rob/quaro-ws/src/quaro_control/config/waypoints.yaml")
    sw.run()

if __name__ == '__main__':
    main()
