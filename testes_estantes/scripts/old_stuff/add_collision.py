#! /usr/bin/env python3

import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld, CollisionObject
import rospy

def wait_for_state_update(box_name, scene_interface, box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        attached_objects = scene_interface.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0
        is_known = box_name in scene_interface.get_known_object_names()
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True
        rospy.sleep(0.1)
        seconds = rospy.get_time()
        return False

if __name__ == '__main__':
    rospy.init_node('attach_object')
    joint_state_topic = ['joint_states:=/my_gen3_lite/joint_states']
    planning_service = ['get_planning_scene:=/my_gen3_lite/get_planning_scene']
    moveit_commander.roscpp_initialize(joint_state_topic)
    namespace = 'gen3_lite_gen3_lite_2f'
    pub_work_scene = rospy.Publisher('/my_gen3_lite/planning_scene',PlanningScene, queue_size=10)
    scene_interface = moveit_commander.PlanningSceneInterface('my_gen3_lite')
    box_pose = Pose()
    box_name = "box"
    primitive = SolidPrimitive()
    primitive.type = primitive.BOX
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    rospy.loginfo(wait_for_state_update(box_name, scene_interface, box_is_known=True))
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 1.0
    box_pose.pose.position.z = 0.0
    
    robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
    group = moveit_commander.MoveGroupCommander('arm', robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
    grasping_group = 'gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    # Publicando a caixa
    scene_interface.attach_box(box_pose.header.frame_id, box_name, touch_links=touch_links, pose=box_pose, size=(1.0, 1.0, 1.0)) 
    rospy.loginfo(wait_for_state_update(box_name, scene_interface, box_is_attached=True, box_is_known=False))
