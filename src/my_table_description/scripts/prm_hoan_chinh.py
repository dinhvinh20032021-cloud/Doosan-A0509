#!/usr/bin/env python3

"""
// lenh build sau khi tao 1 file moi 
chmod +x rrt_hoan_chinh.py
cd ~/catkin_ws
catkin_make
source devel/setup.bash
"""
"""
This script performs the following steps in one file:
 1. Initializes ROS, MoveIt and the Planning Scene.
 2. Adds a mesh table at a fixed pose and scales it.
 3. Publishes a gray color for the table on the proper namespaced topic.
 4. Adds 19 box obstacles at specified positions.
 5. Waits until all objects appear in the scene.
 6. Configures and runs an RRT-based motion sequence (home → goal1 → goal2 → home).
"""

import sys
import os
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import PlanningScene, ObjectColor
from tf.transformations import quaternion_from_euler
from rospkg import RosPack

def add_box(scene, name, size, position, rpy, frame_id):
    """
    Add a box obstacle to the planning scene.
    
    :param scene: PlanningSceneInterface instance
    :param name: unique string ID for the box
    :param size: tuple (x, y, z) dimensions in meters
    :param position: tuple (x, y, z) position in the planning frame
    :param rpy: tuple (roll, pitch, yaw) orientation in radians
    :param frame_id: the reference frame for all poses (e.g., "/world" or robot base)
    """
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    # set position
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    # compute quaternion from roll-pitch-yaw
    q = quaternion_from_euler(*rpy)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    # add the box into the scene
    scene.add_box(name, pose, size=size)
    rospy.loginfo(f"✅ Added obstacle '{name}' at {position}")

def add_table(scene, frame_id, ns):
    """
    Add a mesh table to the planning scene and paint it gray.

    :param scene: PlanningSceneInterface instance
    :param frame_id: reference frame for the table pose
    :param ns: namespace of the MoveIt instance (e.g., "/dsr01m1013")
    """
    # 1) Locate the table.stl in the package
    rp = RosPack()
    pkg_path = rp.get_path("my_table_description")
    mesh_fp  = os.path.join(pkg_path, "meshes", "table.stl")

    # 2) Define the pose of the table (position + identity orientation)
    table_pose = PoseStamped()
    table_pose.header.frame_id = frame_id
    table_pose.pose.position.x = -0.4
    table_pose.pose.position.y =  2.20
    table_pose.pose.position.z =  0.04
    table_pose.pose.orientation.w = 1.0  # no rotation

    # 3) Scale the mesh (twice the original size)
    mesh_scale = (0.2, 0.2, 0.2)

    # 4) Add the mesh to the scene
    scene.add_mesh("table", table_pose, mesh_fp, mesh_scale)
    rospy.loginfo("✅ Added table mesh at (-0.4, 2.20, 0.04) with scale (0.2,0.2,0.2)")

    # 5) Prepare and publish a PlanningScene diff to set the table color to gray
    #    We must publish to the namespaced topic so MoveIt picks it up.
    obj_color = ObjectColor()
    obj_color.id = "table"
    obj_color.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)

    ps = PlanningScene()
    ps.is_diff = True
    ps.object_colors.append(obj_color)

    # Name-spaced planning scene topic for MoveIt
    planning_scene_topic = f"{ns}/planning_scene"
    pub = rospy.Publisher(planning_scene_topic, PlanningScene, queue_size=1)
    rospy.sleep(0.5)  # allow publisher handshake
    pub.publish(ps)
    rospy.loginfo(f"🎨 Published gray color for 'table' on topic '{planning_scene_topic}'")

def main():
    # 1) Initialize ROS node and MoveIt commander
    rospy.init_node("move_sequence_rrt", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    # 2) Create RobotCommander and MoveGroupCommander with namespace
    ns = "/dsr01a0509"
    robot = RobotCommander(robot_description=f"{ns}/robot_description", ns=ns)
    group = MoveGroupCommander("arm", robot_description=f"{ns}/robot_description", ns=ns)

    # 3) Create PlanningSceneInterface in the same namespace
    scene = PlanningSceneInterface(ns=ns, synchronous=True)

    # Give time for scene to initialize
    rospy.sleep(1.0)
    frame_id = group.get_planning_frame()
    rospy.loginfo(f"Planning frame: {frame_id}")

    # 4) Add the table mesh and paint it gray
    add_table(scene, frame_id, ns)

    # 5) Add 19 box obstacles around the robot
    boxes = [
        ("box1",  (0.2,0.2,0.2), ( 0.11, -0.61, 0.13)),
        ("box2",  (0.2,0.2,0.2), ( -0.55, 0.65, 0.11)),
        ("box3",  (0.2,0.2,0.2), (-0.78,  0.60, 0.43)),
        ("box4",  (0.2,0.2,0.2), (-0.61, -0.64, 0.41)),
        ("box5",  (0.2,0.2,0.2), (-0.27,  0.68, 0.54)),
        ("box6",  (0.2,0.2,0.2), ( 0.00, -0.46, 0.95)),
        ("box7",  (0.2,0.2,0.2), (-0.23,  0.66, 1.09)),
        ("box8",  (0.2,0.2,0.2), ( 0.69,  0.51, 0.12)),
        ("box9",  (0.2,0.2,0.2), ( 0.64, -0.19, 0.09)),
        ("box10", (0.2,0.2,0.2), ( 0.57,  0.10, 0.52)),
        ("box11", (0.2,0.2,0.2), ( 0.00, -0.07, 1.43)),
        ("box12", (0.2,0.2,0.2), ( 0.50, -0.34, 0.73)),
        ("box13", (0.2,0.2,0.2), (-0.62, -0.67, 0.10)),
        ("box14", (0.2,0.2,0.2), ( 0.44,  0.31, 1.08)),
        ("box15", (0.2,0.2,0.2), ( 0.12, -0.68, 0.49)),
        ("box16", (0.2,0.2,0.2), (-0.56,  0.00, 0.11)),
        ("box17", (0.2,0.2,0.2), (-0.61,  0.00, 0.34)),
        ("box18", (0.2,0.2,0.2), (-0.53,  0.00, 1.03)),
        ("box19", (0.2,0.2,0.2), ( 0.00,  0.67, 0.12)),
    ]
    for name, size, pos in boxes:
        add_box(scene, name, size, pos, (0,0,0), frame_id)

    # 6) Wait until all objects (table + boxes) appear in the planning scene
    expected = {n for n, *_ in boxes} | {"table"}
    t0 = rospy.get_time()
    while not rospy.is_shutdown() and rospy.get_time() - t0 < 5.0:
        if expected.issubset(set(scene.get_known_object_names())):
            rospy.loginfo("✅ All objects are now in the scene")
            break
        rospy.sleep(0.1)

    # 7) Configure RRT planner parameters
    group.set_goal_joint_tolerance(0.01)
    group.set_max_velocity_scaling_factor(0.2)
    group.set_max_acceleration_scaling_factor(0.2)
    group.set_planning_time(10.0)
    group.set_planner_id("PRMstar")

    # 8) Define and execute the joint-space sequence
    home  = [0, 0, 0, 0, 0, 0]
    goal1 = [-2.480, 0.619, 1.374, -0.004, 1.191, -3.252]
    goal2 = [-3.850, 0.619, 1.374, -0.004, 1.191, -3.252]
    sequence = [home, goal1, goal2, home]

    rospy.loginfo("🚀 Executing motion using RRT")
    for idx, target in enumerate(sequence):
        group.set_joint_value_target(target)
        plan = group.plan()
        if plan and plan[0]:
            rospy.loginfo(f"➡️ Moving to point {idx+1}")
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
