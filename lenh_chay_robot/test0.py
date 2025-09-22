#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('move_robot_dsr')

# Action client tới controller thật
client = actionlib.SimpleActionClient(
    '/dsr01m1013/dsr_joint_trajectory_controller/follow_joint_trajectory',
    FollowJointTrajectoryAction
)
client.wait_for_server()

# Các điểm joint (radian)
home = [0, 0, 0, 0, 0, 0]
goal1 = [0.898, -0.835, -1.148, 0, -1.181, 0]
goal2 = [-0.732, -0.835, -1.148, 0, -1.181, 0]
sequence = [home, goal1, goal2, home]

joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']

for idx, positions in enumerate(sequence):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(3.0)  # 3 giây cho mỗi điểm
    goal.trajectory.points.append(point)
    
    rospy.loginfo(f"Di chuyển đến điểm {idx+1}: {positions}")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.sleep(0.5)

rospy.loginfo("Hoàn tất di chuyển qua tất cả các điểm.")
