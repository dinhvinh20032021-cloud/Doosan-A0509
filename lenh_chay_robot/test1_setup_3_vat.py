#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def add_obstacle_box(scene, name, size, position, rpy, frame):
    """
    Thêm một hộp obstacle vào Planning Scene.
      - scene: PlanningSceneInterface
      - name: tên object (chuỗi)
      - size: (x, y, z) kích thước hộp
      - position: (x, y, z) tọa độ center
      - rpy: (roll, pitch, yaw) góc xoay
      - frame: frame_id để gắn object
    """
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
    rospy.init_node("move_sequence_with_obstacles", anonymous=True)

    # Khởi RobotCommander, Scene, MoveGroup
    robot = moveit_commander.RobotCommander()
    scene = PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    # Đợi scene load xong
    rospy.sleep(1.0)
    planning_frame = group.get_planning_frame()
    rospy.loginfo(f"Planning frame: {planning_frame}")

    # 1) block1 chặn home→goal1
    add_obstacle_box(
        scene, "block1",
        size=(0.2, 0.4, 0.3),
        position=(0.4,  0.05, 0.35),
        rpy=(0, 0, 0),
        frame=planning_frame
    )

    # 2) block2 chặn goal1→goal2
    add_obstacle_box(
        scene, "block2",
        size=(0.3, 0.2, 0.3),
        position=(0.1,  0.4, 0.35),
        rpy=(0, 0, 0),
        frame=planning_frame
    )

    # 3) block3 chặn goal2→home
    add_obstacle_box(
        scene, "block3",
        size=(0.3, 0.2, 0.3),
        position=(0.1, -0.4, 0.35),
        rpy=(0, 0, 0),
        frame=planning_frame
    )

    # Chờ tất cả object được đăng ký trong Planning Scene
    obstacles = {"block1", "block2", "block3"}
    start = rospy.get_time()
    while (rospy.get_time() - start < 5.0) and not rospy.is_shutdown():
        known = set(scene.get_known_object_names())
        if obstacles.issubset(known):
            rospy.loginfo("✅ All obstacles are now known in the scene")
            break
        rospy.sleep(0.1)
    else:
        rospy.logwarn("⚠️ Some obstacles did not appear in time")

    # Cấu hình planner RRTConnect
    group.set_goal_joint_tolerance(0.01)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_planning_time(5.0)
    group.set_planner_id("RRTConnectkConfigDefault")

    # Chuỗi joint targets (6 joints)
    home  = [0, 0, 0, 0, 0, 0]
    goal1 = [-2.805, 0.619, 1.374, -0.004, 1.191, -3.252]
    goal2 = [-3.850, 0.619, 1.374, -0.004, 1.191, -3.252]
    sequence = [home, goal1, goal2, home]

    rospy.loginfo("🚀 Bắt đầu chuỗi di chuyển né nhiều vật cản")
    for idx, target in enumerate(sequence):
        group.set_joint_value_target(target)
        plan = group.plan()
        if plan and plan[0]:
            rospy.loginfo(f"➡️ Moving to point {idx+1}")
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            rospy.sleep(1.0)
        else:
            rospy.logwarn(f"⚠️ Plan failed at point {idx+1}")
            break

    rospy.loginfo("✅ Hoàn tất chuỗi di chuyển.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
