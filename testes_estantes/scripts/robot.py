#! /usr/bin/env python3
import threading
from copy import deepcopy
from time import sleep
from typing import Tuple, Union

from kortex_api.Exceptions.KException import KException

from kortex_api.autogen.messages import Base_pb2, DeviceConfig_pb2, Common_pb2

from abstract_robot import AbstractRobot
from robot_connection import RobotConnection
from utils.robot_enum import RobotErrorEnum


class Robot(AbstractRobot):
    """
    This class is responsible for the robot connection and movement. Now, the robot used is the Kinova Gen3 Lite.
    The methods implemented in this class are the basic movements of the robot, such as cartesian and joints movements.
    This class implements the AbstractRobot class, which has the basic methods for the robot.
    """

    TIMEOUT = 60

    def __init__(self):
        self.base = None
        self.action = None
        self.device = None
        self.active_state = None
        self.error_number = None
        self.gripper_command = None
        self.arm_state_notif_handle = None
        self.critical_error = False
        self.action_finished = True
        self.final_position = None
        self.trajectory_info = []

    def __connect(self, connection_ip: str = "192.168.2.10"):
        """
        Connect api with the robot, using the ethernet connection ip as default connection

        To connect with usb, pass "192.168.1.10" as connection_ip

        :param(str) connection_ip: The ip address to make the connection

        """
        # Create a connection to a device to get the services
        self.device = RobotConnection.create_tcp_connection(connection_ip)
        self.device.connect()

    def request_services(self, control_robot=False):

        # Requested services
        self.base = self.device.get_base_client()
        self.base_cyclic = self.device.get_base_cyclic_client()

        if control_robot:
            return

        self.gripper = self.device.get_gripper_cyclic_client()
        self.device_config = self.device.get_device_config_client()

    def prepare_for_action(self):
        """
        Actions to be executed immediately after connecting the robot.
        """

        if self.critical_error:
            self.clear_faults()

        self.open_tool(0.60)

    def connect_with_ethernet(self):
        """
        Connect api with the robot, using the ethernet connection ip
        """
        self.__connect("192.168.2.10")

    def connect_with_usb(self):
        """
        Connect api with the robot, using the usb connection ip
        """
        self.__connect("192.168.1.10")

    def disconnect(self):
        """
        Finish connection with robot
        """
        if self.device is None:
            return
        self.device.disconnect()

    def check_for_end_or_abort(self, e) -> object:
        """
        Return a closure checking for END or ABORT notifications

        Args:
            (any) e: event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """

        def check(notification, event=e):
            if notification.action_event == Base_pb2.ACTION_FEEDBACK:
                trajectory_info = deepcopy(notification.trajectory_info)
                self.trajectory_info.extend(trajectory_info)
            if notification.action_event == Base_pb2.ACTION_ABORT:
                self.action_finished = False
                if notification.abort_details == Base_pb2.ROBOT_IN_FAULT:
                    print("Abort details - Robot in Fault")
                event.set()
            if notification.action_event == Base_pb2.ACTION_END:
                event.set()

        return check

    def check_error(self) -> bool:
        """
        Analyse the robot error and warning, classifying the error level.
        It may be a critical error or not, setting critical_error attribute to True.
        It might also be Just a warning, setting the error_number attribute
        to the warning's message value

        Returns:
        (bool): Return False if the robot is IN fault and True if not.
        """

        arm_state = self.base.GetArmState()

        base_cyclic_feedback = self.base_cyclic.RefreshFeedback()
        valid_error__bank_a = RobotErrorEnum.is_valid_error(base_cyclic_feedback.base.fault_bank_a)
        valid_error__bank_b = RobotErrorEnum.is_valid_error(base_cyclic_feedback.base.fault_bank_b)

        # Escalável para um método, caso necessite ampliar a checkagem para mais erros.
        errors_list = [Base_pb2.ARMSTATE_IN_FAULT, Base_pb2.ARMSTATE_SERVOING_LOW_LEVEL]

        if arm_state.active_state in errors_list:

            if valid_error__bank_a:
                self.error_number = base_cyclic_feedback.base.fault_bank_a
                self.critical_error = True

                return False

            elif valid_error__bank_b:
                self.error_number = base_cyclic_feedback.base.fault_bank_b
                self.critical_error = True

                return False

            else:
                # Caso em que o robô está em falta, mas não é um erro crítico
                self.critical_error = False
                return False

        elif base_cyclic_feedback.base.warning_bank_a in errors_list:
            self.critical_error = False
            self.error_number = base_cyclic_feedback.base.warning_bank_a

            return True

        elif base_cyclic_feedback.base.warning_bank_b in errors_list:
            self.critical_error = False
            self.error_number = base_cyclic_feedback.base.warning_bank_b

            return True

        elif arm_state.active_state == 3:
            return True

        return True

    def get_error_message_by_number(self, number=None) -> str:

        if number is None:
            if self.error_number is None:
                return ''

            number = self.error_number

        try:
            robot_error = RobotErrorEnum(number)
            return robot_error.name
        except (ValueError,) as e:
            print(e)
            return "Number error not defined"

    def subscribe_to_arm_state_notification(self):
        def callback_arm_state(data):
            print("arm state topic data -", data)
            if data.active_state == Base_pb2.ARMSTATE_IN_FAULT:
                self.active_state = data.active_state

            if data.active_state == Common_pb2.ARMSTATE_SERVOING_PLAYING_SEQUENCE:
                print("arm state playing sequence", data)
                self.active_state = data.active_state

        try:
            arm_state_notif_callback_handle = \
                self.base.OnNotificationArmStateTopic(callback_arm_state,
                                                      Base_pb2.NotificationOptions())
            return arm_state_notif_callback_handle
        except KException as k_ex:
            print("kexception ", k_ex)

    def subscribe_to_safety_notification(self):
        def callback_safety(data):

            print("Safety event happened")

        try:
            safety_notif_callback_handle = \
                self.device_config.OnNotificationSafetyTopic(callback_safety,
                                                             DeviceConfig_pb2.NotificationOptions())
            return safety_notif_callback_handle
        except KException as k_ex:
            print("kexception ", k_ex)

    def get_safety_list(self) -> list:

        return self.base.SafetyNotificationList

    def __detection_move(self) -> bool:
        """
        Subscribes to an action movement topic.

        Sends the action request to robot's base to execute the movement.
        It waits till the action ends or a timeout occurs. Then, unsubscribes
        and evaluates the success of the movement.

        Returns:
            (bool): An indication of movement success
        """

        e = threading.Event()
        self.trajectory_info = []
        self.action_finished = True
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        try:
            self.base.ExecuteAction(self.action)
        except (Exception,) as e:
            print("Detection move executing action exception", e)
            return False

        finished = e.wait(Robot.TIMEOUT)

        self.action_finished &= finished
        self.base.Unsubscribe(notification_handle)

        if not self.action_finished:
            return False

        self.action_finished &= self.check_action_end_reason()

        return True

    def clear_faults(self):
        self.base.ClearFaults()

    def move_joints(self, joints_list: list[float]) -> bool:
        """
        Set movement for robot with joints values
        Args:
            (list) joints_list: lista with values for all joints for movement

        Returns:
            (bool) move is finished
        """

        self.action = Base_pb2.Action()
        self.action.name = "Angular action movement"
        self.action.application_data = ""

        for joint_id in range(len(joints_list)):
            joint_angle = self.action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = joints_list[joint_id]

        finished = self.__detection_move()

        return finished

    def move_cartesian(self, pose_list: list[float]) -> bool:
        """
        Set movement for robot with cartesian coordinates
        Args:
            (float) pose[0]: x value
            (float) pose[1]: y value
            (float) pose[2]: z value
            (float) theta_x: theta_x value
            (float) theta_y: theta_y value
            (float) theta_z: theta_z value

        Returns:
            (bool): Move is finished
        """
        self.action = Base_pb2.Action()
        self.action.name = "Example Cartesian action movement"
        self.action.application_data = ""

        cartesian_pose = self.action.reach_pose.target_pose
        cartesian_pose.x = pose_list[0]  # [meters]
        cartesian_pose.y = pose_list[1]  # [meters]
        cartesian_pose.z = pose_list[2]  # [meters]
        cartesian_pose.theta_x = pose_list[3]  # [degrees]
        cartesian_pose.theta_y = pose_list[4]  # [degrees]
        cartesian_pose.theta_z = pose_list[5]  # [degrees]

        finished = self.__detection_move()

        return finished

    def close_tool(self) -> bool:
        """
        This function closes the gripper and tries detected object
        Returns:
            (bool): object_detect: returns whether an object was detected or not detected
        """
        object_detected = False
        loops = 0
        currents = 0
        variation = 0.28
        while not object_detected and float(self.attribute_from_gripper()["position"]) < 92.8:
            average = 0
            gripper_command = Base_pb2.GripperCommand()
            finger = gripper_command.gripper.finger.add()
            gripper_command.mode = Base_pb2.GRIPPER_POSITION
            finger.finger_identifier = 1
            finger.value = self.__increment()
            self.base.SendGripperCommand(gripper_command)

            first_current = float(self.attribute_from_gripper()["current_motor"])
            if 4 > first_current:
                loops += 1
                average = currents / loops
            else:
                print("atypical current")

            if loops > 1 and average is not 0:
                if variation <= first_current - average and first_current > 0.6:
                    finger.value = self.__increment()
                    self.base.SendGripperCommand(gripper_command)
                    second_current = float(self.attribute_from_gripper()["current_motor"])

                    if variation <= second_current - average and second_current > 0.61:
                        object_detected = True
                        finger.value = self.__increment()
                        self.base.SendGripperCommand(gripper_command)
                        self.final_position = float(self.attribute_from_gripper()["position"]) / 100
                currents += first_current

        return object_detected

    def open_tool(self, value: float = 0.70):
        """
        Open griper with value
        Args
            :(float) value: Value for open grips
        """
        self.gripper_command = Base_pb2.GripperCommand()
        finger = self.gripper_command.gripper.finger.add()

        self.gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        finger.value = value
        self.base.SendGripperCommand(self.gripper_command)

        sleep(0.8)

    def get_joint_angles(self):
        """
        Get joint angles from robot.
        Returns:
            (list): joint_angles: list with all joints angles
        """
        joint_angles_obj = self.base.GetMeasuredJointAngles()
        joint_angles_list = joint_angles_obj.joint_angles
        joint_angles = []
        for joint in joint_angles_list:
            joint_angles.append(round(joint.value, 5))

        return joint_angles

    def get_cartesian(self) -> list[float]:
        """
        Get actual cartesian pose from robot.
        Returns:
            (list): final_pose: list with cartesian pose
        """

        pose_obj = self.base.GetMeasuredCartesianPose()
        x = round(pose_obj.x, 5)
        y = round(pose_obj.y, 5)
        z = round(pose_obj.z, 5)
        theta_x = round(pose_obj.theta_x, 5)
        theta_y = round(pose_obj.theta_y, 5)
        theta_z = round(pose_obj.theta_z, 5)

        final_pose = [x, y, z, theta_x, theta_y, theta_z]

        return final_pose

    def apply_emergency_stop(self):
        self.base.ApplyEmergencyStop()

    def attribute_from_gripper(self) -> dict:
        # TODO - refatorar
        variable = self.base_cyclic.RefreshFeedback().__str__().split()
        position = variable.index("gripper_feedback")
        information_gripper = {"position": variable[position + 7],
                               "velocity": variable[position + 9],
                               "current_motor": variable[position + 11]}

        return information_gripper

    def __increment(self, have_medicine: bool = False) -> float:
        increment = [1.3, 1.5, 20, 1.3]
        position = float(self.attribute_from_gripper()["position"])

        if have_medicine:
            return (position + increment[3]) / 100
        elif position < 70:
            return (position + increment[2]) / 100
        elif position < 85:
            return (position + increment[1]) / 100
        else:
            return (position + increment[0]) / 100

    def check_action_end_reason(self) -> bool:
        """
        Checks if the action ended with success or for some other reason.

        When a problem occurs during a robot's action an ActionFeedback
        event is fired. If this event can be resolved, a TRAJECTORY_OK enum
        in the TrajectoryInfoType field is sent.

        self.trajectory_info is a list of TrajectoryInfo sent during the robot's movement
        when there is in an ActionFeedback event.

        :return: True if the action ended with success or False otherwise

        """
        if len(self.trajectory_info) == 0:
            return True
        trajectory_info_list = [trajectory.trajectory_info_type for trajectory in self.trajectory_info]

        if Base_pb2.TRAJECTORY_OK not in trajectory_info_list:
            return False

        return True


if __name__ == "__main__":
    robot = Robot()
    robot.connect_with_ethernet()
    robot.request_services()
    robot.clear_faults()
