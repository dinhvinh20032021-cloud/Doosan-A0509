#!/usr/bin/env python3
import sys
import rospy
import random
import moveit_commander
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
    rospy.loginfo(f"✅ Added '{name}' size={size} at {position}")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_sequence_with_random_obstacles", anonymous=True)

    # Khởi RobotCommander, PlanningScene và MoveGroup
    robot = moveit_commander.RobotCommander()
    scene = PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    # Đợi scene load
    rospy.loginfo("⏳ Waiting for planning scene to initialize...")
    rospy.sleep(1.0)
    planning_frame = group.get_planning_frame()
    rospy.loginfo(f"Planning frame: {planning_frame}")

    # Sinh ngẫu nhiên 10 hộp xung quanh robot
    random.seed()  # hoặc cố định seed để lặp lại kết quả
    obstacles = []
    for i in range(10):
        # Vị trí ngẫu nhiên trong ô vuông [-0.5, 0.5] trên x,y, z cố định 0.2
        x = random.uniform(-0.5, 0.5)
        y = random.uniform(-0.5, 0.5)
        z = 0.2
        size = (0.05, 0.05, 0.2)  # hộp nhỏ
        name = f"rand_box_{i+1}"
        obstacles.append({
            "name": name,
            "size": size,
            "position": (x, y, z),
            "rpy": (0, 0, 0),
        })

    # Thêm tất cả hộp vào Planning Scene
    for obs in obstacles:
        add_obstacle_box(
            scene,
            obs["name"],
            obs["size"],
            obs["position"],
            obs["rpy"],
            planning_frame
        )

    # Chờ tất cả obstacles xuất hiện
    expected = {obs["name"] for obs in obstacles}
    start = rospy.get_time()
    while not rospy.is_shutdown() and (rospy.get_time() - start < 5.0):
        known = set(scene.get_known_object_names())
        if expected.issubset(known):
            rospy.loginfo("✅ All random obstacles are now in the scene")
            break
        rospy.sleep(0.1)
    else:
        missing = expected - known
        rospy.logwarn(f"⚠️ Missing obstacles: {missing}")

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

    rospy.loginfo("🚀 Bắt đầu chuỗi di chuyển né 10 hộp ngẫu nhiên")
    for idx, target in enumerate(sequence):
        group.set_joint_value_target(target)
        plan = group.plan()
        if plan and plan[0]:
            rospy.loginfo(f"➡️ Move to point {idx+1}")
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            rospy.sleep(1.0)
        else:
            rospy.logwarn(f"⚠️ Planner failed at point {idx+1}")
            break

    rospy.loginfo("✅ Hoàn tất chuỗi di chuyển.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
