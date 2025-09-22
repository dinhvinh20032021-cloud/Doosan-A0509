#!/usr/bin/env python3

import sys
import os
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from rospkg import RosPack

def add_box(scene, name, size, position, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    pose.pose.orientation.w = 1.0
    scene.add_box(name, pose, size=size)
    rospy.loginfo(f"✅ Added obstacle '{name}' at {position}")

def add_table(scene, frame_id):
    rp = RosPack()
    pkg = rp.get_path("my_table_description")
    fp  = os.path.join(pkg, "meshes", "table.stl")
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = -0.4
    pose.pose.position.y =  2.20
    pose.pose.position.z =  0.04
    pose.pose.orientation.w = 1.0
    scale = (0.2, 0.2, 0.2)
    scene.add_mesh("table", pose, fp, scale)
    rospy.loginfo("✅ Added table mesh")

def main():
    # Init ROS & MoveIt
    rospy.init_node("run_avoidance", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    # RobotCommander & MoveGroup
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm")

    # PlanningScene
    scene = PlanningSceneInterface(synchronous=True)
    rospy.sleep(1.0)
    frame = group.get_planning_frame()
    rospy.loginfo(f"Planning in frame: {frame}")

    # 1) Add table + obstacles
    add_table(scene, frame)
    boxes = [
        (f"box{i}", (0.2,0.2,0.2), pos)
        for i, pos in enumerate([
          (0.11, -0.78, 0.13),(0.60, -0.34, 0.73),(-0.86, -0.89, 0.10),
          (0.66,  0.31, 1.08),(0.12, -0.79, 0.45),(-0.84,  0.00, 0.11),
          (-0.84,  0.00, 0.41),(-0.68,  0.00, 1.20),(0.00,  0.94, 0.12),
          (-0.92,  0.76, 0.11),(-0.96,  0.79, 0.43),(-0.70, -0.76, 0.41),
          (-0.27,  0.72, 0.54),(0.00, -0.52, 1.02),(-0.11,  0.76, 1.09),
          (0.84,  0.51, 0.12),(0.72, -0.27, 0.09),(0.61,  0.10, 0.52),
          (0.00,  0.09, 1.67)
        ], start=1)
    ]
    for name, size, pos in boxes:
        add_box(scene, name, size, pos, frame)

    # Wait tới khi tất cả objects được thêm vào
    expected = {n for n,_,_ in boxes} | {"table"}
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < 5.0:
        if expected.issubset(set(scene.get_known_object_names())):
            rospy.loginfo("✅ All objects in the scene")
            break
        rospy.sleep(0.1)

    # 2) Cấu hình planning
    group.set_goal_joint_tolerance(0.01)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_planning_time(10.0)
    group.set_planner_id("RRTkConfigDefault")

    # 3) Chuỗi pose (home → goal1 → goal2 → home)
    home  = [0,0,0,0,0,0]
    goal1 = [-2.805,0.619,1.374,-0.004,1.191,-3.252]
    goal2 = [-3.850,0.619,1.374,-0.004,1.191,-3.252]
    seq   = [home, goal1, goal2, home]

    rospy.loginfo("🚀 Executing avoidance trajectory")
    for i, tgt in enumerate(seq, start=1):
        group.set_joint_value_target(tgt)
        plan = group.plan()
        if plan and plan[0]:
            rospy.loginfo(f"➡️ Move {i}")
            group.execute(plan[1], wait=True)
            group.stop()
            group.clear_pose_targets()
            rospy.sleep(1.0)
        else:
            rospy.logwarn(f"⚠️ Plan failed at step {i}")
            break

    rospy.loginfo("✅ Done.")
    moveit_commander.roscpp_shutdown()

if __name__=="__main__":
    main()
