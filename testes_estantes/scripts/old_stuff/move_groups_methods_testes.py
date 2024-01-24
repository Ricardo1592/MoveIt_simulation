#! /usr/bin/env python3
import sys
import copy
import time
import math
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import moveit_commander
import geometry_msgs.msg 
import moveit_msgs.msg
import rospy  


if __name__ == '__main__':

    rospy.init_node('planning_scene_kinova', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting the Initialization')
    print("argv")
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
    scene = moveit_commander.PlanningSceneInterface('my_gen3_lite', synchronous=True)

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name, robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")

    display_trajectory_publisher = rospy.Publisher( "/move_group/display_planned_path",
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20,
                                                   )
    
    # planning parameters --> https://answers.ros.org/question/365579/moveit-takes-a-lot-of-time-for-plan-and-execute/
    #https://ompl.kavrakilab.org/planners.html
    #https://planners-benchmarking.readthedocs.io/en/latest/user_guide/2_motion_planners.html
    move_group.set_planner_id("RRTConnect")
    move_group.set_pose_reference_frame('base_link')
    move_group.allow_replanning(False)
    move_group.set_num_planning_attempts(10)
    move_group.set_planning_time(5)
    
    rospy.loginfo('Cleaning of the objects in the scene')
    try:
        scene.clear()
        # scene.remove_world_object("grasping_object")

    except Exception as e:
        print(e)


    rospy.sleep(1.0)
    #euler_from_quaternion --> https://python.hotexamples.com/pt/site/file?hash=0xddea61d221dba056f657cb35f51cee33fd02af40dd9863afc4b2d7e6ebf1abad&fullName=msc-kinova-experiments-master/src/grasp_server/moveit.py&project=m-rios/msc-kinova-experiments
    quaternion = quaternion_from_euler(1.60, 0.50, 0.00, axes='rxyz')
    # print("quaternion -- ", quaternion)
    stl_id = 'grasping_object'
    obj_pose = geometry_msgs.msg.PoseStamped()
    obj_pose.header.frame_id = 'base_link'
    obj_pose.pose.position.x =  0.35 
    obj_pose.pose.position.y = 1.265
    obj_pose.pose.position.z = -0.01
    obj_pose.pose.orientation.x = quaternion[0]
    obj_pose.pose.orientation.y = quaternion[1]
    obj_pose.pose.orientation.z = quaternion[2]
    obj_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo('Adding the structure')
    scene.add_mesh(
    stl_id, obj_pose,
    '/home/ricardo/Downloads/montagemApenasEstrutura_.stl'
    )


    time.sleep(1)

    rospy.loginfo('Adding the table object')
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "base_link"
    table_pose.pose.position.x = 0.00
    table_pose.pose.position.y = 0.00
    table_pose.pose.position.z = -0.01
    scene.add_box("table", table_pose, size=(1.90, 1.8, 0.01))
    
    time.sleep(1)

    paths_lenght = []
    planning_time_list = []

    # pose_goal = geometry_msgs.msg.PoseStamped()
    # pose_goal.pose.position.x = -0.3565
    # pose_goal.pose.position.y = 0.5051
    # pose_goal.pose.position.z = 0.0770
    # pose_goal.pose.orientation.x = 0.1815
    # pose_goal.pose.orientation.y = -0.8822
    # pose_goal.pose.orientation.z = -0.4120
    # pose_goal.pose.orientation.w = 0.1382
    # pose_goal.header.frame_id = "world"
    # print("pose goal frame_id ", pose_goal.header.frame_id)


    joints = move_group.get_current_joint_values()
    joints = [round(i, 5) for i in joints]
    print("current joints values - ", joints)
    print("type ", type(joints))

    pose = move_group.get_current_pose()

    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z
    x_orient = pose.pose.orientation.x
    y_orient = pose.pose.orientation.y
    z_orient = pose.pose.orientation.z
    w_orient = pose.pose.orientation.w

    pose_list = [x, y, z, x_orient, y_orient, z_orient, w_orient]
    pose_list = [round(i, 5) for i in pose_list]
    
    print("pose values", pose_list)

    scene_objs = scene.get_known_object_names()

    print("scenes -", scene_objs)