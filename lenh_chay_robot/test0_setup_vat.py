#!/usr/bin/env python3
import sys, rospy, moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def add_obstacle_box(scene, name, size, position, rpy, frame):
    box = PoseStamped()
    box.header.frame_id = frame
    box.pose.position.x, box.pose.position.y, box.pose.position.z = position
    q = quaternion_from_euler(*rpy)
    box.pose.orientation.x, box.pose.orientation.y = q[0], q[1]
    box.pose.orientation.z, box.pose.orientation.w = q[2], q[3]
    scene.add_box(name, box, size=size)
    rospy.loginfo(f"✅ Added box '{name}' size={size} at {position} in frame {frame}")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_sequence_with_obstacle", anonymous=True)

    # Khởi RobotCommander, PlanningScene và MoveGroup
    robot = moveit_commander.RobotCommander()
    scene = PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    # Đợi scene load
    rospy.sleep(1)
    planning_frame = group.get_planning_frame()
    rospy.loginfo(f"Planning frame: {planning_frame}")

    # Thêm hộp chặn đường đi thẳng giữa home->goal1
    add_obstacle_box(
        scene,
        name="block1",
        size=(0.2, 0.4, 0.3),
        position=(0.4, 0.05, 0.35),
        rpy=(0, 0, 0),
        frame=planning_frame
    )

    # Chờ MoveIt cập nhật scene
    start = rospy.get_time()
    while (rospy.get_time() - start < 5.0) and not rospy.is_shutdown():
        if "block1" in scene.get_known_object_names():
            rospy.loginfo("✅ block1 is now known in the scene")
            break
        rospy.sleep(0.1)
    else:
        rospy.logwarn("⚠️ block1 không xuất hiện sau 5s")

    # Cấu hình planner
    group.set_goal_joint_tolerance(0.01)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_planning_time(5.0)
    group.set_planner_id("RRTConnectkConfigDefault")

    # Chuỗi joint targets (6 joints cho M1013)
    home  = [0, 0, 0, 0, 0, 0]
    goal1 = [-2.805, 0.619, 1.374, -0.004, 1.191, -3.252]
    goal2 = [-3.850, 0.619, 1.374, -0.004, 1.191, -3.252]
    sequence = [home, goal1, goal2, home]

    rospy.loginfo("🚀 Bắt đầu chuỗi di chuyển né vật cản")
    for idx, pose in enumerate(sequence):
        group.set_joint_value_target(pose)
        plan = group.plan()
        if plan and plan[0]:
            rospy.loginfo(f"➡️ Move to point {idx+1}")
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            rospy.sleep(1)
        else:
            rospy.logwarn(f"⚠️ Plan thất bại tại point {idx+1}")
            break

    rospy.loginfo("✅ Hoàn tất chuỗi di chuyển.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
