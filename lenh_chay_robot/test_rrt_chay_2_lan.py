#!/usr/bin/env python3
import rospy
import moveit_commander

def main():
    rospy.init_node("move_sequence_node")
    moveit_commander.roscpp_initialize([])

    # Khởi tạo RobotCommander và MoveGroupCommander với namespace đúng
    robot = moveit_commander.RobotCommander(robot_description="/dsr01m1013/robot_description")
    group = moveit_commander.MoveGroupCommander(
        "arm",
        robot_description="/dsr01m1013/robot_description",
        ns="/dsr01m1013"
    )

    # Cấu hình chuyển động mượt và dùng thuật toán RRTConnect
    group.set_goal_joint_tolerance(0.01)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_planning_time(5.0)
    group.set_planner_id("RRTConnectkConfigDefault")  # Sử dụng thuật toán RRTConnect

    # Các vị trí joint (radian)
    home = [0, 0, 0, 0, 0, 0]
    goal1 = [-2.805, 0.619, 1.374, -0.004, 1.191, -3.252]
    goal2 = [-3.850 ,0.619, 1.374, -0.004, 1.191, -3.252]
    sequence = [home, goal1, goal2, home]

    # Thực hiện 2 vòng chuyển động
    for i in range(2):
        rospy.loginfo(f"🔁 Bắt đầu vòng {i+1}")
        for idx, pose in enumerate(sequence):
            group.set_joint_value_target(pose)
            plan = group.plan()
            if plan and plan[0]:
                rospy.loginfo(f"➡️ Di chuyển đến điểm {idx+1} bằng RRTConnect")
                success = group.go(wait=True)
                group.stop()
                group.clear_pose_targets()
                rospy.sleep(1)
                if not success:
                    rospy.logwarn(f"⚠️ Robot không hoàn thành chuyển động tại vòng {i+1}, điểm {idx+1}")
                    break
            else:
                rospy.logwarn(f"⚠️ Không lập được kế hoạch đến điểm {idx+1}")
                break

    rospy.loginfo("✅ Hoàn tất 2 vòng chuyển động bằng RRTConnect.")

if __name__ == "__main__":
    main()
