from pathlib import Path
import sys
import copy
import time
import math

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import moveit_commander
import geometry_msgs.msg
import sensor_msgs.msg
import moveit_msgs.msg
import rospy


class SetUp:

    def __init__(self):
        SetUp.show_init_log()

        SetUp.create_node()

        SetUp.initialize_topic()

    @staticmethod
    def create_node():
        rospy.init_node('planning_scene_kinova', anonymous=True, log_level=rospy.DEBUG)

    @staticmethod
    def show_init_log():
        rospy.loginfo('Starting the Scene SetUp')

    @staticmethod
    def initialize_topic():
        joint_state_topic = ['joint_states:=/my_gen3_lite/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)

    @staticmethod
    def get_robot_commander() -> moveit_commander.RobotCommander:
        robot = moveit_commander.RobotCommander(robot_description="/my_gen3_lite/robot_description", ns="my_gen3_lite")
        return robot

    @staticmethod
    def get_scene() -> moveit_commander.PlanningSceneInterface:
        scene = moveit_commander.PlanningSceneInterface('my_gen3_lite', synchronous=True)
        return scene

    @staticmethod
    def get_move_group() -> moveit_commander.MoveGroupCommander:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name,
                                                         robot_description="/my_gen3_lite/robot_description",
                                                         ns="my_gen3_lite")
        return move_group


class MoveGroup:
    def __init__(self, move_group: moveit_commander.MoveGroupCommander):
        self.move_group = move_group


class RobotCommander:
    def __init__(self, robot: moveit_commander.MoveGroupCommander):
        self.robot = robot
        self.pose_stamped: PoseStamped = PoseStamped()

    def get_pose(self) -> PoseStamped:
        return self.robot.get_current_pose()

    def get_cartesian_radian(self) -> tuple:
        quaternion = [0] * 4

        self.pose_stamped = self.get_pose()

        quaternion[0] = self.pose_stamped.pose.orientation.x
        quaternion[1] = self.pose_stamped.pose.orientation.y
        quaternion[2] = self.pose_stamped.pose.orientation.z
        quaternion[3] = self.pose_stamped.pose.orientation.w

        cartesian_pose = euler_from_quaternion(quaternion)

        return cartesian_pose

    def get_cartesian(self) -> list:
        cartesian_pose = self.get_cartesian_radian()

        # A pose do kinova tem 3 valores de posição x, y, z, e
        # 3 valores de orientação adicionadas pelo método append
        degrees = [0] * 3
        degrees[0] = round(self.pose_stamped.pose.position.x, 5)
        degrees[1] = round(self.pose_stamped.pose.position.y, 5)
        degrees[2] = round(self.pose_stamped.pose.position.z, 5)

        for radian in cartesian_pose:
            angle = round(math.degrees(radian), 5)
            degrees.append(angle)

        return degrees


class Scene:
    def __init__(self, setup: SetUp):
        self.objects_name = None
        self.scene_pub: rospy.Publisher = None

        self.setup = setup

        self.scene = self.setup.get_scene()
        self.move_group = self.setup.get_move_group()

        self.mg = MoveGroup(self.move_group)
        self.robot = RobotCommander(self.move_group)

        self.shelf_only_3s = ShelfOnly3s()
        self.table_obj = TableObject()

    def set_scene_publisher(self):
        self.scene_pub = rospy.Publisher('/my_gen3_lite/planning_scene', moveit_msgs.msg.PlanningScene, queue_size=20)

    def clean_scene(self):
        rospy.loginfo('Cleaning of the objects in the scene')
        try:
            self.scene.clear()

        except Exception as e:
            print(e)

    def change_shelf_only_3s_pose(self):
        # TODO
        """
        Change shelf position, using an inherited implementation
        of PlanningSceneInterface class and CollisionObject message class.
        PlanningSceneInterface: https://docs.ros.org/en/jade/api/moveit_commander/html/planning__scene__interface_8py_source.html
        CollisionObject MSG: https://docs.ros.org/en/api/moveit_msgs/html/msg/CollisionObject.html
        """
        ...

    def add_shelf_only_3s(self,
                          mesh_path: str = '/home/rcmm/Documents/kinova_ros/catkin_workspace/src/testes_estantes/scripts/teste3oandarmodelo3d/montagemPlanejamento-3andares.stl'):
        """
        Adds a file mesh to a MoveIt scene.
        """
        rospy.loginfo(f'Adding the structure: {self.shelf_only_3s.name}')

        self.scene.add_mesh(
            self.shelf_only_3s.name,
            self.shelf_only_3s.get_pose(),
            mesh_path
        )

    def set_shelf_only_3s_pose(self, pose: list, name: str = ''):
        self.shelf_only_3s.set_pose(pose, name)

    def add_table(self, size: tuple = (1.90, 1.8, 0.01)):
        rospy.loginfo('Adding the table object')
        self.scene.add_box(self.table_obj.name, self.table_obj.pose_msg, size=size)

    def get_objects(self) -> list:
        self.objects_name = self.scene.get_known_object_names()
        return self.objects_name

    def remove_object(self, name: str):
        self.scene.remove_world_object(name)


class SceneObject:
    def __init__(self, name: str = ''):
        self.name = name
        self.pose_msg = geometry_msgs.msg.PoseStamped()

    def set_pose(self, pose: list):
        quaternion = quaternion_from_euler(*pose[3:], axes='rxyz')
        # print("quaternion -- ", quaternion)
        self.pose_msg.header.frame_id = 'base_link'
        self.pose_msg.pose.position.x = pose[0]
        self.pose_msg.pose.position.y = pose[1]
        self.pose_msg.pose.position.z = pose[2]
        self.pose_msg.pose.orientation.x = quaternion[0]
        self.pose_msg.pose.orientation.y = quaternion[1]
        self.pose_msg.pose.orientation.z = quaternion[2]
        self.pose_msg.pose.orientation.w = quaternion[3]

    def change_pose(self, pose: list):
        self.set_pose(pose)

    def get_pose(self) -> geometry_msgs.msg.PoseStamped:
        return self.pose_msg


class ShelfOnly3s(SceneObject):
    def __init__(self, name: str = 'shelf_only_3s'):
        super().__init__(name)
        self.initial_pose: list = [0.35, 1.29, 0.01, 1.60, 0.50, 0.00]
        self.set_pose(self.initial_pose)


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
        self.planned_path_pub = rospy.Publisher("/move_group/display_planned_path",
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20,
                                                )


if __name__ == "__main__":
    setup = SetUp()
    scene = Scene(setup)
    print(scene.table_obj.name)
    scene.clean_scene()
    # scene.add_mesh()
    # scene.add_table()
