#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def add_box(scene, name, size, position, rpy, frame_id):
    """
    Thêm một hộp va chạm vào Planning Scene.
      - scene: PlanningSceneInterface
      - name: tên object
      - size: (x, y, z) kích thước hộp
      - position: (x, y, z) tọa độ center
      - rpy: (roll, pitch, yaw)
      - frame_id: frame để gắn object
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    q = quaternion_from_euler(*rpy)
    pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
    pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
    scene.add_box(name, pose, size=size)
    rospy.loginfo(f"✅ Added '{name}' size={size} at {position}")

def main():
    rospy.init_node("move_sequence_with_boxes", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    # Khởi RobotCommander và MoveGroup với namespace và robot_description của robot thật
    robot = moveit_commander.RobotCommander(
        robot_description="/dsr01m1013/robot_description",
        ns="/dsr01m1013"
    )
    group = moveit_commander.MoveGroupCommander(
        "arm",
        robot_description="/dsr01m1013/robot_description",
        ns="/dsr01m1013"
    )

    # Khởi PlanningSceneInterface cùng namespace
    scene = PlanningSceneInterface(ns="/dsr01m1013")

    # Cho scene 1s để đồng bộ
    rospy.sleep(1.0)
    frame_id = group.get_planning_frame()
    rospy.loginfo(f"Planning frame: {frame_id}")

    # Danh sách 19 hộp (size, position, rotation)
    boxes = [
        ("box1",  (0.2,0.2,0.2), ( 0.11, -0.78, 0.13), (0,0,0)),
        ("box2",  (0.2,0.2,0.2), ( 0.60, -0.34, 0.73), (0,0,0)),
        ("box3",  (0.2,0.2,0.2), (-0.86, -0.89, 0.10), (0,0,0)),
        ("box4",  (0.2,0.2,0.2), ( 0.66,  0.31, 1.08), (0,0,0)),
        ("box5",  (0.2,0.2,0.2), ( 0.12, -0.79, 0.45), (0,0,0)),
        ("box6",  (0.2,0.2,0.2), (-0.84,  0.00, 0.11), (0,0,0)),
        ("box7",  (0.2,0.2,0.2), (-0.84,  0.00, 0.41), (0,0,0)),
        ("box8",  (0.2,0.2,0.2), (-0.68,  0.00, 1.20), (0,0,0)),
        ("box9",  (0.2,0.2,0.2), ( 0.00,  0.94, 0.12), (0,0,0)),
        ("box10", (0.2,0.2,0.2), (-0.92,  0.76, 0.11), (0,0,0)),
        ("box11", (0.2,0.2,0.2), (-0.96,  0.79, 0.43), (0,0,0)),
        ("box12", (0.2,0.2,0.2), (-0.70, -0.76, 0.41), (0,0,0)),
        ("box13", (0.2,0.2,0.2), (-0.27,  0.72, 0.54), (0,0,0)),
        ("box14", (0.2,0.2,0.2), ( 0.00, -0.52, 1.02), (0,0,0)),
        ("box15", (0.2,0.2,0.2), (-0.11,  0.76, 1.09), (0,0,0)),
        ("box16", (0.2,0.2,0.2), ( 0.84,  0.51, 0.12), (0,0,0)),
        ("box17", (0.2,0.2,0.2), ( 0.72, -0.27, 0.09), (0,0,0)),
        ("box18", (0.2,0.2,0.2), ( 0.61,  0.10, 0.52), (0,0,0)),
        ("box19", (0.2,0.2,0.2), ( 0.00,  0.09, 1.67), (0,0,0)),
    ]

    # Thêm từng hộp vào scene
    for name, size, pos, rpy in boxes:
        add_box(scene, name, size, pos, rpy, frame_id)

    # Chờ tối đa 5s cho các object xuất hiện
    expected = {name for name, *_ in boxes}
    start = rospy.get_time()
    while not rospy.is_shutdown() and (rospy.get_time() - start) < 5.0:
        known = set(scene.get_known_object_names())
        if expected.issubset(known):
            rospy.loginfo("✅ All boxes are now in the scene")
            break
        rospy.sleep(0.1)

    # Cấu hình MoveGroup cho robot thật
    group.set_goal_joint_tolerance(0.01)
    group.set_max_velocity_scaling_factor(0.2)       # 20% max speed
    group.set_max_acceleration_scaling_factor(0.2)   # 20% max accel
    group.set_planning_time(5.0)
    group.set_planner_id("RRTConnectkConfigDefault")

    # Chuỗi chuyển động
    home  = [0, 0, 0, 0, 0, 0]
    goal1 = [-2.805, 0.619, 1.374, -0.004, 1.191, -3.252]
    goal2 = [-3.850, 0.619, 1.374, -0.004, 1.191, -3.252]
    sequence = [home, goal1, goal2, home]

    rospy.loginfo("🚀 Executing motion avoiding 19 boxes")
    for idx, target in enumerate(sequence):
        group.set_joint_value_target(target)
        plan = group.plan()
        if plan and plan[0]:
            rospy.loginfo(f"➡️ Moving to point {idx+1}")
            # Dùng execute để robot thật đi theo đúng trajectory
            group.execute(plan[1], wait=True)
            group.stop()
            group.clear_pose_targets()
            rospy.sleep(1.0)
        else:
            rospy.logwarn(f"⚠️ Planning failed at point {idx+1}")
            break

    rospy.loginfo("✅ Motion complete.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
