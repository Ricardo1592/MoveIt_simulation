    
from pathlib import Path
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

class Setup:

    def __init__(self):
        
        self.create_node_setUp()
        
        self.create_log_setUp()
        
        self.initialize_topic()
    
    def create_node_setUp(self):    
        rospy.init_node('planning_scene_kinova', anonymous=True, log_level=rospy.DEBUG)

    def create_log_setUp(self):
        rospy.loginfo('Starting the Initialization')    

    def initialize_topic(self):
        joint_state_topic = ['joint_states:=/my_gen3_lite/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)


    def get_robot_commander(self) -> moveit_commander.RobotCommander:    
        robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
        return robot
        
    def get_scene(self) -> moveit_commander.PlanningSceneInterface:
        scene = moveit_commander.PlanningSceneInterface('my_gen3_lite', synchronous=True)
        return scene

    def get_move_group(self) -> moveit_commander.MoveGroupCommander:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name, robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
        return move_group

class Scene:

    def __init__(self, setup: Setup):
        self.setup = setup
        self.scene = self.setup.get_scene()
        self.shelf = ShelfOnly()
        self.table_obj = TableObject()

    def set_scene_publisher(self):
        self.scene_pub = rospy.Publisher('/my_gen3_lite/planning_scene',moveit_msgs.msg.PlanningScene, queue_size=20)    

    def clean_scene(self):
        rospy.loginfo('Cleaning of the objects in the scene')
        try:
            self.scene.clear()

        except Exception as e:
            print(e)

    def change_shelf_position(self):
        # TODO
        """
        Acho que tenho que publicar no tópico de PlanningSceneInterface
        """
        ...

    def add_shelf(self,
                mesh_path: str = '/home/rcmm/Documents/kinova_ros/catkin_workspace/src/testes_estantes/scripts/teste3oandarmodelo3d/montagemPlanejamento-3andares.stl'):
        """
        Adds a file mesh to a MoveIt scene.
        """
        rospy.loginfo(f'Adding the structure: {self.shelf.name}')
        
        self.scene.add_mesh(
            self.shelf.name,
            self.shelf.pose_msg,
            mesh_path
        )

    def set_shelf_pose(self, pose: list, name: str = ''):
        self.shelf.set_pose(pose, name)
        

    def add_table(self, size: tuple = (1.90, 1.8, 0.01)):
        rospy.loginfo('Adding the table object')
        self.scene.add_box(self.table_obj.name, self.table_obj.pose_msg, size=size)


class SceneObject():

    def __init__(self, name: str = ''):
        self.name = name
        self.pose_msg = geometry_msgs.msg.PoseStamped()

    def set_pose(self, pose: list):

        quaternion = quaternion_from_euler(*pose[3:], axes='rxyz')
        # print("quaternion -- ", quaternion)
        self.pose_msg.header.frame_id = 'base_link'
        self.pose_msg.pose.position.x =  pose[0] 
        self.pose_msg.pose.position.y = pose[1]
        self.pose_msg.pose.position.z = pose[2]
        self.pose_msg.pose.orientation.x = quaternion[0]
        self.pose_msg.pose.orientation.y = quaternion[1]
        self.pose_msg.pose.orientation.z = quaternion[2]
        self.pose_msg.pose.orientation.w = quaternion[3]   

        return self.pose_msg
    
class ShelfOnly(SceneObject):
    def __init__(self, name: str = 'Shelf'):
        super().__init__(name)
        self.pose_list = [0.35, 1.29, 0.01, 1.60, 0.50, 0.00]
        self.set_pose(self.pose_list)


class TableObject(SceneObject):

    def __init__(self, name: str = 'Table_box'):
        super().__init__(name)
        self.set_pose()

    def set_pose(self, pose: list = [0.00, 0.00, -0.01]):

        self.pose_msg.header.frame_id = "base_link"
        self.pose_msg.pose.position.x = pose[0]
        self.pose_msg.pose.position.y = pose[1]
        self.pose_msg.pose.position.z = pose[2]
    
class PathPlanner:
    def __init__(self):
        self.planned_path_pub: rospy.Publisher = None

    def set_path_publisher(self):
        self.planned_path_pub = rospy.Publisher( "/move_group/display_planned_path",
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20,
                                                   )

if __name__ == "__main__":
    setup = Setup()
    scene = Scene(setup)
    print(scene.table_obj.name)
    scene.clean_scene()
    # scene.add_mesh()
    # scene.add_table()