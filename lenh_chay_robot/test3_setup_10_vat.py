#!/usr/bin/env python3
import sys
import rospy
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
    rospy.loginfo(f"✅ Added '{name}' size={size} at {position} in {frame}")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_with_selective_obstacles", anonymous=True)

    # Khởi RobotCommander, Scene và MoveGroup
    robot = moveit_commander.RobotCommander()
    scene = PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    # Đợi Planning Scene khởi tạo
    rospy.sleep(1.0)
    planning_frame = group.get_planning_frame()
    rospy.loginfo(f"Planning frame: {planning_frame}")

    # Định nghĩa 10 hộp, nhưng chỉ 2 hộp block1/block2 nằm trên quỹ đạo chính
    cube_size = (0.2, 0.2, 0.2)
    obstacles = [
        # Hai hộp “chính” buộc robot phải né
        { "name": "block1",
          "position": (0.4,  0.05, 0.30),
          "rpy": (0, 0, 0) },
        { "name": "block2",
          "position": (-0.2, 0.30, 0.30),
          "rpy": (0, 0, 0) },

        # Tám hộp phụ đặt xa hoặc cao, không chặn quỹ đạo thẳng
        { "name": "block3",  "position": (1.2,  1.0, 0.20), "rpy": (0, 0, 0) },
        { "name": "block4",  "position": (-1.2, -1.0, 0.20), "rpy": (0, 0, 0) },
        { "name": "block5",  "position": (1.2, -1.0, 0.20), "rpy": (0, 0, 0) },
        { "name": "block6",  "position": (-1.2, 1.0, 0.20), "rpy": (0, 0, 0) },
        { "name": "block7",  "position": (0.0,  1.5, 0.50), "rpy": (0, 0, 0) },
        { "name": "block8",  "position": (0.0, -1.5, 0.50), "rpy": (0, 0, 0) },
        { "name": "block9",  "position": (1.5,  0.0, 0.50), "rpy": (0, 0, 0) },
        { "name": "block10", "position": (-1.5, 0.0, 0.50), "rpy": (0, 0, 0) },
    ]

    # Thêm 10 hộp vào Planning Scene
    for obs in obstacles:
        add_obstacle_box(
            scene,
            obs["name"],
            cube_size,
            obs["position"],
            obs["rpy"],
            planning_frame
        )

    # Chờ tất cả hộp được load
    names = {o["name"] for o in obstacles}
    start = rospy.get_time()
    while not rospy.is_shutdown() and (rospy.get_time() - start < 5.0):
        if names.issubset(set(scene.get_known_object_names())):
            rospy.loginfo("✅ All obstacles are now known")
            break
        rospy.sleep(0.1)
    else:
        missing = names - set(scene.get_known_object_names())
        rospy.logwarn(f"⚠️ Missing obstacles: {missing}")

    # Cấu hình planner
    group.set_goal_joint_tolerance(0.01)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_planning_time(5.0)
    group.set_planner_id("RRTConnectkConfigDefault")

    # Chuỗi joint targets
    home  = [0, 0, 0, 0, 0, 0]
    goal1 = [-2.805, 0.619, 1.374, -0.004, 1.191, -3.252]
    goal2 = [-3.850, 0.619, 1.374, -0.004, 1.191, -3.252]
    sequence = [home, goal1, goal2, home]

    rospy.loginfo("🚀 Executing motion avoiding selected obstacles")
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

    rospy.loginfo("✅ Motion complete.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
