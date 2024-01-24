#! /usr/bin/env python3
import sys
import copy
import time
import math
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import moveit_commander
import geometry_msgs.msg 
import sensor_msgs.msg
import moveit_msgs.msg
import rospy  
moveit_msgs.msg.MoveItErrorCodes
planning_times = []
times_from_start = []
plans = []
execution_sucess_list = []
success = True

def teste1(move_group, pose_goal):
    global success
    
    move_group.set_start_state_to_current_state()
    joint_start = copy.deepcopy(move_group.get_current_joint_values())
    print("joints inicial or last position --> ", joint_start)

    ## Position1
    # joint_start[0] = math.radians(-36) 
    # joint_start[1] = math.radians(-43)
    # joint_start[2] = math.radians(51)
    # joint_start[3] = math.radians(-12)
    # joint_start[4] = math.radians(-39)
    # joint_start[5] = math.radians(7)
    
    # # planning parameters --> https://answers.ros.org/question/365579/moveit-takes-a-lot-of-time-for-plan-and-execute/
    # #https://ompl.kavrakilab.org/planners.html
    # #https://planners-benchmarking.readthedocs.io/en/latest/user_guide/2_motion_planners.html
    # move_group.set_planner_id("RRTConnect")
    # move_group.set_pose_reference_frame('base_link')
    # move_group.allow_replanning(False)

    print("planner to move to start postion id -- ", move_group.get_planner_id())
    # joints = [-150, 84, 45, -118, 74, 118]
    joints = [15, -39, 87, -59, 18, 64]
    joint_start = [math.radians(i) for i in joints]  
    # move_group.set_goal_tolerance(0.005)
    move_group.set_planner_id("RRTConnect")
    print("global success", success)
    if success:
        move_group.set_joint_value_target(joint_start)
        start_position_confirmed = move_group.go(wait=True)
        print("joints start radian -", joint_start)
        
        while not start_position_confirmed:
            start_position_confirmed = move_group.go(wait=True)
    rospy.sleep(0.5)        
    move_group.stop()
    move_group.set_start_state_to_current_state()    



    # # `go()` returns a boolean indicating whether the planning and execution was successful.
    # # plan interface --> https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_ros/planning_interface/move_group_interface/src/wrap_python_move_group.cpp
    # move_group.set_joint_value_target(joint_start)
    # move_group.go(wait=True)
    # move_group.set_start_state_to_current_state()

    # print("joints start position --> ", move_group.get_current_joint_values())

    # For goal as joints values
    # joint_goal = copy.deepcopy(joint_start)
    # joint_goal[0] = math.radians(136) 
    # joint_goal[1] = math.radians(-125)
    # joint_goal[2] = math.radians(-67)
    # joint_goal[3] = math.radians(-103)
    # joint_goal[4] = math.radians(-72)
    # joint_goal[5] = math.radians(111)
    # in collision just to verify it's working, never use goal tolarance,
    # because it will still be able to find a solution collision free, even if the position is obviously in collision
    # joint_goal[0] = math.radians(-29) 
    # joint_goal[1] = math.radians(-57)
    # joint_goal[2] = math.radians(29)
    # joint_goal[3] = math.radians(-7)
    # joint_goal[4] = math.radians(-34)
    # joint_goal[5] = math.radians(7)
    # move_group.set_joint_value_target(joint_goal)


    move_group.set_pose_target(pose_goal)
    # move_group.set_goal_tolerance(0.025)
    move_group.set_planner_id("RRTstar")
    move_group.set_pose_reference_frame('base_link')
    move_group.allow_replanning(False)
    move_group.set_num_planning_attempts(20)
    move_group.set_planning_time(5)

    print("planner query id -- ", move_group.get_planner_id())
    
    plan = move_group.plan()

    # print("planning frame -- ", move_group.get_planning_frame())
    # print("pose reference frame -- ", move_group.get_pose_reference_frame())

    i = 0
    # print("plan obj -- ", len(plan[1].joint_trajectory.points))
    # print("plan obj -- ", plan)

    print("plan success = ", plan[0])
    if plan[0]:

        print("going to position")
        # print("has joint_trajectory attribute:", hasattr(plan[1], "joint_trajectory"))
        success = move_group.execute(plan[1], wait=True)
        print("success of plan =", success)
        if success:
            execution_sucess_list.append(1)   
            rospy.sleep(0.5)
            move_group.stop()
            time_from_start = plan[1].joint_trajectory.points[-1].time_from_start.to_sec()
            times_from_start.append(time_from_start)
        else:
            execution_sucess_list.append(0)    
        pose_goal = move_group.get_current_pose()
        plans.append(plan)
        # print("pose goal is --> ", pose_goal)
        # print("pose_goal.pose is --> ", pose_goal.pose)
    else:
        print("plan failed")
        plans.append(plan)

    print("planning time: ", plan[2])
    print("error code: ", plan[3])
    return plan



# sites: https://python.hotexamples.com/pt/site/file?hash=0xd0854fb0129139503a43256944c8e2a56638996cecb0b949b325dfa4d3099a9f
# https://python.hotexamples.com/pt/site/file?hash=0xddea61d221dba056f657cb35f51cee33fd02af40dd9863afc4b2d7e6ebf1abad&fullName=msc-kinova-experiments-master/src/grasp_server/moveit.py&project=m-rios/msc-kinova-experiments


# Não funciona, mas capturado usando outro método
def check_computation_time(msg):
    # get computation time for successful plan to be found
    print("message ", msg)
    if msg.status.status == 3:
        planning_times.append(msg.result.planning_time)
    

if __name__ == '__main__':

    rospy.init_node('planning_scene_kinova', anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo('Starting the Initialization')
    print("argv -- ", sys.argv)
    joint_state_topic = ['joint_states:=/my_gen3_lite/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)

    robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
    scene = moveit_commander.PlanningSceneInterface('my_gen3_lite', synchronous=True)

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name, robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")

    display_trajectory_publisher = rospy.Publisher( "/move_group/display_planned_path",
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20,
                                                   )
    pub_work_scene = rospy.Publisher('/my_gen3_lite/planning_scene',moveit_msgs.msg.PlanningScene, queue_size=20)
    # joint_state_publisher = rospy.Publisher('/my_gen3_lite/joint_states', sensor_msgs.msg.JointState, queue_size=20)
    
    # Esse subscriber não está printando o callback - TODO - descobrir o motivo depois
    planning_time_sub = rospy.Subscriber('/move_group/result', moveit_msgs.msg.MoveGroupActionResult, check_computation_time)
    

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # # Sometimes for debugging it is useful to print the entire state of the
    # # robot:
    # print("============ Printing robot state")
    # robot_state = robot.get_current_state()
    # print(type(robot_state))
    # print("")

    ## Planner params
    #print("============ Printing planner param")
    #print(robot.get_planner_param())
    #print("")

    rospy.loginfo('Cleaning of the objects in the scene')
    try:
        scene.remove_world_object("grasping_object")

    except Exception as e:
        print(e)

    rospy.sleep(1.5)
    #euler_from_quaternion --> https://python.hotexamples.com/pt/site/file?hash=0xddea61d221dba056f657cb35f51cee33fd02af40dd9863afc4b2d7e6ebf1abad&fullName=msc-kinova-experiments-master/src/grasp_server/moveit.py&project=m-rios/msc-kinova-experiments
    quaternion = quaternion_from_euler(0, 1.60, 1.60)
    print("quaternion -- ", quaternion)
    stl_id = 'grasping_object'
    obj_pose = geometry_msgs.msg.PoseStamped()
    obj_pose.header.frame_id = 'base_link'
    obj_pose.pose.position.x = -0.95
    obj_pose.pose.position.y = 0.90
    obj_pose.pose.position.z = -0.015
    obj_pose.pose.orientation.x = 1.60
    obj_pose.pose.orientation.y = 1.60
    obj_pose.pose.orientation.z = 1.60
    obj_pose.pose.orientation.w = 1.60
    
    time.sleep(1)

    scene.add_mesh(
    stl_id, obj_pose,
    '/home/ricardo/Downloads/montagemApenasEstrutura.stl' 
    )

    rospy.loginfo('Adding the structure')
    # scene.add_mesh(
    # stl_id, obj_pose,
    # '/home/ricardo/Downloads/montagemApenasEstrutura(2).stl'
    # )

    # time.sleep(1)

    # rospy.loginfo('Adding the table object')
    # table_pose = geometry_msgs.msg.PoseStamped()
    # table_pose.header.frame_id = "base_link"
    # table_pose.pose.position.x = 0.00
    # table_pose.pose.position.y = 0.00
    # table_pose.pose.position.z = 0.00
    # scene.add_box("table", table_pose, size=(1.90, 1.8, 0.0))

    paths_lenght = []
    planning_time_list = []

    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.pose.position.x = -0.3565
    pose_goal.pose.position.y = 0.5051
    pose_goal.pose.position.z = 0.0770
    pose_goal.pose.orientation.x = 0.1815
    pose_goal.pose.orientation.y = -0.8822
    pose_goal.pose.orientation.z = -0.4120
    pose_goal.pose.orientation.w = 0.1382
    pose_goal.header.frame_id = "world"
    print("pose goal frame_id ", pose_goal.header.frame_id)


    for i in range(1, 26):
        print("running:", i)
        plan = teste1(move_group, pose_goal)
        planning_time = plan[2]
        waypoints = len(plan[1].joint_trajectory.points)
        if plan[0]:
            motion_time = plan[1].joint_trajectory.points[-1].time_from_start.to_sec()
            motion_time = round(motion_time, 4)
            planning_time_list.append(motion_time)
        paths_lenght.append(waypoints)

    print("waypoints25 =", paths_lenght)
    print("planning_times25 =", planning_time_list)
    print("succesful_execution_times =", [round(i, 4) for i in times_from_start])
    print("25 list success", execution_sucess_list)

    # for i in range(1, 51):
    #     print("running:", i)
    #     plan = teste1(move_group, pose_goal)
    #     planning_time = plan[2]
    #     waypoints = len(plan[1].joint_trajectory.points)
    #     motion_time = plan[1].joint_trajectory.points[-1].time_from_start.to_sec()
    #     motion_time = round(motion_time, 4)
    #     paths_lenght.append(waypoints)
    #     planning_time_list.append(motion_time)


    # print("waypoints50 =", paths_lenght)
    # print("planning_times50 =", planning_time_list)
    # print("succesful_execution_times =", [round(i, 4) for i in times_from_start])
    # print("50 list success", execution_sucess_list)

    # for i in range(1,51):
    #     results = teste1(move_group)
    #     paths_lenght.append(results[0])
    #     planning_time_list.append(round(results[1], 4))        
    
        
    # print("50 list", paths_lenght)
    # print("50 list", planning_time_list)

    # for i in range(1,101):
    #     results = teste1(move_group)
    #     paths_lenght.append(results[0])
    #     planning_time_list.append(round(results[1], 4))        
           
    # print("100 list", paths_lenght)
    # print("100 list", planning_time_list)

    #planners examples    

     # setup the hand group and its planner --> https://python.hotexamples.com/pt/examples/moveit_commander/MoveGroupCommander/set_planner_id/python-movegroupcommander-set_planner_id-method-examples.html#0x8820579dc010e8c78b75e0dac0bab1299242ddb28f1b1c63e3dd5d0e55a15ca9-93,,123,
    #hand = MoveGroupCommander("hand")
    #hand.set_start_state_to_current_state()
    #hand.set_planner_id("LBKPIECEkConfigDefault") 
    #hand.set_planning_time(10.0)

    

    #if plan.joint_trajectory:  # True if trajectory contains points
    #    print("traj success")
    #else:
    #    rospy.logerr("Trajectory is empty. Planning was unsuccessful.")
    

    # Calling `stop()` ensures that there is no residual movement


    # https://python.hotexamples.com/pt/examples/moveit_commander/MoveGroupCommander/set_planner_id/python-movegroupcommander-set_planner_id-method-examples.html#0x548743957d493bb7e4d8e7f2cdbb0fce61d7e96d5adfc7ed77ceba1a7f008f5c-7,,83,
    #self.gripper = MoveGroupCommander("onine_gripper")
    #self.arm = MoveGroupCommander("onine_arm")
#
    #self.arm.set_goal_tolerance(0.004)
    #self.arm.allow_replanning(True)
    ## self.arm.set_goal_position_tolerance(0.005)
    ## self.arm.set_goal_orientation_tolerance(0.1)
    #self.arm.set_num_planning_attempts(10)
    #self.arm.set_planning_time(5)
    #self.arm.set_planner_id("RRTkConfigDefault")

    
    # planner output --> https://python.hotexamples.com/pt/examples/moveit_commander/MoveGroupCommander/set_planner_id/python-movegroupcommander-set_planner_id-method-examples.html#0x5a6619d664df5aeadf27070ac69ceadb6c2fe7462967b105cf5cc6a518d93902-12,,186,

    print("planner interface ", move_group.get_interface_description().planner_ids)
    moveit_commander.roscpp_shutdown()