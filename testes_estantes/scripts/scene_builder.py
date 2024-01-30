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
from utils.json_manager import JsonManager


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
    def __init__(self,
                 move_group: moveit_commander.MoveGroupCommander,
                 planner: str = "RRTConnect",
                 pose_ref_frame: str = "base_link",
                 allow_replanning: bool = False,
                 planning_attempts: int = 100,
                 planning_time: float = 2.6,
                 goal_tolerance: float = 0.025
                 ):

        self.goal_tolerance = goal_tolerance
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts
        self.allow_replanning = allow_replanning
        self.pose_ref_frame = pose_ref_frame
        self.planner = planner
        self.move_group = move_group
        self.current_pose = self.get_current_pose()

    def get_current_pose(self) -> PoseStamped:
        return self.move_group.get_current_pose()

    def plan_and_execute(self, start_state, goal_state):
        """
        Plan the path from start state to goal state.
        First checks if the robot is in start state,
        in case not, move the robot to start state
        and then plans the trajectory to goal.

        Args:
            start_state:
            goal_state:
        Return:
            ...
        """
        self.move_group.set_start_state_to_current_state()

        self.move_group.set_pose_target(start_state)
        # move_group.set_goal_tolerance(0.025)
        self.use_standard_plan_config()
        print("planner query id -- ", self.move_group.get_planner_id())
        plan = self.move_group.plan()

        if not MoveGroup.plan_is_successful(plan):
            return

        print("going to position")
        success = self.move_group.execute(plan[1], wait=True)

        if not success:
            return

        self.current_pose = self.move_group.get_current_pose()
        print("current pose\n", self.current_pose)

        self.move_group.set_start_state_to_current_state()

    def use_standard_plan_config(self):
        """
        Use methods present in most planning scenes. It is possible
        to change them by using the change_default_config method.
        Or by directly changing the instance attributes of this class.
        or  These are the standard configurations:

        planner: str = "RRTConnect",
        pose_ref_frame: str = "base_link",
        allow_replanning: bool = False,
        planning_attempts: int = 100,
        planning_time: float = 2.6,
        goal_tolerance: float = 0.025 (not allowed by default)

        """
        self.move_group.set_planner_id(self.planner)
        self.move_group.set_pose_reference_frame(self.pose_ref_frame)
        self.move_group.allow_replanning(self.allow_replanning)
        self.move_group.set_num_planning_attempts(self.planning_attempts)
        self.move_group.set_planning_time(self.planning_time)

        # Parâmetro para melhorar a taxa de sucesso,
        # porém, aumentando as chances de colisão.
        # self.move_group.set_goal_tolerance(0.025)

    def change_default_config(self,
                              planner: str = "RRTConnect",
                              pose_ref_frame: str = "base_link",
                              allow_replanning: bool = False,
                              planning_attempts: int = 100,
                              planning_time: float = 2.6,
                              goal_tolerance: float = 0.025,
                              ):
        """
        Args:
            planner: Planner to be used. Set it as a string
             equal to the name in rviz graphical interface
            pose_ref_frame: Reference frame inside PoseStamped object
            planning_attempts: Tries till timeout
            planning_time: Time till timeout. The timeout occurs when planning_attempts or planning time is reached
            goal_tolerance: Helps when planning is consistently failing, but raises collision probability
            allow_replanning:
        """

        self.planning_time = planning_time
        self.planning_attempts = planning_attempts
        self.allow_replanning = allow_replanning
        self.pose_ref_frame = pose_ref_frame
        self.planner = planner

    @staticmethod
    def plan_is_successful(plan: tuple):
        """
        Args:
             plan (tuple): A tuple with the following elements:
                (MoveItErrorCodes, trajectory_msg, planning_time, error_code)

        Returns:
            bool: True if plan successfully computed.
        """

        print("plan success", plan[0])
        return plan[0]


class RobotCommander:
    def __init__(self, move_group: MoveGroup):
        self.move_group = move_group
        self.pose_stamped: PoseStamped = PoseStamped()

    def get_pose(self) -> PoseStamped:
        return self.move_group.move_group.get_current_pose()

    def get_radian_angles(self) -> tuple:
        self.pose_stamped = self.get_pose()

        orientation_angles = PoseJsonAdapter.adapt_quaternion_to_radian(self.pose_stamped)

        return orientation_angles

    def get_cartesian(self) -> list:
        orientation_angles = self.get_radian_angles()

        # A pose do kinova tem 3 valores de posição x, y, z, e
        # 3 valores de orientação adicionadas pelo método append
        pose_kortex = [0] * 3

        pose_kortex[0] = round(self.pose_stamped.pose.position.x, 5)
        pose_kortex[1] = round(self.pose_stamped.pose.position.y, 5)
        pose_kortex[2] = round(self.pose_stamped.pose.position.z, 5)

        angles = PoseJsonAdapter.adapt_radian_angles_to_degrees(orientation_angles)

        pose_kortex.extend(angles)

        return pose_kortex

    def move(self, start_state, goal_state):
        start_state = PoseJsonAdapter.adapt_robot_to_pose_stamped(start_state)
        goal_state = PoseJsonAdapter.adapt_robot_to_pose_stamped(goal_state)

        self.move_group.plan_and_execute(start_state, goal_state)


class Scene:
    def __init__(self, setup: SetUp):
        self.objects_name = None
        self.scene_pub: rospy.Publisher = None

        self.setup = setup

        self.scene = self.setup.get_scene()
        self.move_group = self.setup.get_move_group()

        self.mg = MoveGroup(self.move_group)
        self.robot = RobotCommander(self.mg)

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
        self.pose_msg = PoseJsonAdapter.adapt_to_pose_stamped(pose)

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


class PoseJsonAdapter:
    """
    Adapts the json registry to be used in MoveIt.
    """

    def __init__(self, json_path: str = ''):
        self.json_path = json_path
        self.json_manager: JsonManager = JsonManager(json_path)

    @staticmethod
    def adapt_to_pose_stamped(pose: list, degrees=False, axes='rxyz', frame_id='base_link') -> PoseStamped:
        """
        Transforms a pose list to a PoseStamped object. The MoveGroup from MoveIt
        uses this as an argument some of its methods.
        """
        pose_msg = geometry_msgs.msg.PoseStamped()

        if degrees:
            pose[3] = math.radians(pose[3])
            pose[4] = math.radians(pose[4])
            pose[5] = math.radians(pose[5])
            axes = 'sxyz'

        quaternion = quaternion_from_euler(*pose[3:], axes=axes)

        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = pose[2]
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        return pose_msg

    @staticmethod
    def adapt_robot_to_pose_stamped(pose: list, degrees=False, axes='rxyz', frame_id='world') -> PoseStamped:
        """
        Transforms a pose list to a PoseStamped object. The MoveGroup from MoveIt
        uses this as an argument some of its methods.
        """

        pose_msg = PoseJsonAdapter.adapt_to_pose_stamped(pose, degrees=True, axes='sxyz', frame_id='world')

        return pose_msg

    @staticmethod
    def adapt_quaternion_to_radian(pose: PoseStamped) -> tuple:
        quaternion = [0] * 4

        quaternion[0] = pose.pose.orientation.x
        quaternion[1] = pose.pose.orientation.y
        quaternion[2] = pose.pose.orientation.z
        quaternion[3] = pose.pose.orientation.w

        radian_angles = euler_from_quaternion(quaternion)

        return radian_angles

    @staticmethod
    def adapt_radian_angles_to_degrees(radian_angles: tuple) -> list:
        angles = []

        for angle in radian_angles:
            degrees_angle = round(math.degrees(angle), 5)
            angles.append(degrees_angle)

        return angles


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
