#!/usr/bin/env python
# Author: Sammy Pfeiffer

# Based on https://github.com/ros-naoqi/nao_robot/blob/master/nao_apps/scripts/test_joint_angles.py
import rospy
from rospy import Duration

import actionlib
from actionlib_msgs.msg import GoalStatus
import naoqi_msgs.msg
import trajectory_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
import std_srvs.srv


def move_head_test():
    client = actionlib.SimpleActionClient("joint_trajectory", naoqi_msgs.msg.JointTrajectoryAction)
    rospy.loginfo("Waiting for joint_trajectory server...")
    client.wait_for_server()
    rospy.loginfo("Done.")

    goal = naoqi_msgs.msg.JointTrajectoryGoal()
    # multiple joints, multiple keypoints:
    goal.trajectory.joint_names = ["HeadYaw", "HeadPitch"]
    goal.trajectory.points = []
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(0.5),
                                                    positions = [1.0, 1.0]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.0),
                                                    positions = [1.0, 0.0]))
    goal.trajectory.points.append(JointTrajectoryPoint(time_from_start = Duration(1.5),
                                                    positions = [0.0, 0.0]))

    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Getting results...")
    result = client.get_result()
    print "Result:", ', '.join([str(n) for n in result.goal_position.position])

if __name__ == '__main__':
    try:
        rospy.init_node('joint_angle_client')
        move_head_test()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
