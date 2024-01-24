from enum import Enum, unique


# TODO - Change class name to match kortex proto's name
@unique
class RobotErrorEnum(Enum):
    """
    SafetyIdentifier enum
    Admissible Base safeties.
    Used with BaseCyclic.BaseFeedback.[fault_bank_a | fault_bank_b | warning_bank_a | warning_bank_b]
    """
    MAXIMUM_AMBIENT_TEMPERATURE = 4
    MAXIMUM_CORE_TEMPERATURE = 8
    JOINT_FAULT = 16
    BRAKE_REMOVAL_FAILURE = 256
    UNABLE_TO_REACH_POSE = 1024
    JOINT_DETECTION_ERROR = 2048
    NETWORK_INITIALIZATION_ERROR = 4096
    MAXIMUM_CURRENT = 8192
    MAXIMUM_VOLTAGE = 16384
    MINIMUM_VOLTAGE = 32768
    MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY = 65536
    MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY = 131072
    MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION = 262144
    MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION = 524288
    MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE = 1048576
    MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE = 2097152
    MAXIMUM_END_EFFECTOR_PAYLOAD = 4194304
    EMERGENCY_LINE_ACTIVATED = 16777216
    INRUSH_CURRENT_LIMITER_FAULT = 33554432
    NVRAM_CORRUPTED = 67108864
    INCOMPATIBLE_FIRMWARE_VERSION = 134217728
    POWERON_SELF_TEST_FAILURE = 268435456
    DISCRETE_INPUT_STUCK_ACTIVE = 536870912
    ARM_INTO_ILLEGAL_POSITION = 1073741824

    @staticmethod
    def is_valid_error(value: int) -> bool:
        """
        Checks if an integer(error number) is the value for the RobotErrorEnum members

        The method __call__(value) raises a ValueError
        if the value doesn't belong to any class member

        :param value: An integer to look in the class members values
        :return: True if the class contains this value
        """
        try:
            RobotErrorEnum.__call__(value)
        except (ValueError,):
            return False
        return True


@unique
class RobotSubErrorCodesEnum(Enum):
    """
    These codes happen when an abort details event is fired.
    The abort motive is detailed by the following Enums.
    """
    SUB_ERROR_NONE = 0
    METHOD_FAILED = 1
    UNIMPLEMENTED = 2
    INVALID_PARAM = 3
    UNSUPPORTED_SERVICE = 4
    UNSUPPORTED_METHOD = 5
    TOO_LARGE_ENCODED_FRAME_BUFFER = 6
    FRAME_ENCODING_ERR = 7
    FRAME_DECODING_ERR = 8
    INCOMPATIBLE_HEADER_VERSION = 9
    UNSUPPORTED_FRAME_TYPE = 10
    UNREGISTERED_NOTIFICATION_RECEIVED = 11
    INVALID_SESSION = 12
    PAYLOAD_DECODING_ERR = 13
    UNREGISTERED_FRAME_RECEIVED = 14
    INVALID_PASSWORD = 15
    USER_NOT_FOUND = 16
    ENTITY_NOT_FOUND = 17
    ROBOT_MOVEMENT_IN_PROGRESS = 18
    ROBOT_NOT_MOVING = 19
    NO_MORE_STORAGE_SPACE = 20
    ROBOT_NOT_READY = 21
    ROBOT_IN_FAULT = 22
    ROBOT_IN_MAINTENANCE = 23
    ROBOT_IN_UPDATE_MODE = 24
    ROBOT_IN_EMERGENCY_STOP = 25
    SINGLE_LEVEL_SERVOING = 26
    LOW_LEVEL_SERVOING = 27
    MAPPING_GROUP_NON_ROOT = 28
    MAPPING_INVALID_GROUP = 29
    MAPPING_INVALID_MAP = 30
    MAP_GROUP_INVALID_MAP = 31
    MAP_GROUP_INVALID_PARENT = 32
    MAP_GROUP_INVALID_CHILD = 33
    MAP_GROUP_INVALID_MOVE = 34
    MAP_IN_USE = 35
    WIFI_CONNECT_ERROR = 36
    UNSUPPORTED_NETWORK_TYPE = 37
    TOO_LARGE_ENCODED_PAYLOAD_BUFFER = 38
    UPDATE_PERMISSION_DENIED = 39
    DELETE_PERMISSION_DENIED = 40
    DATABASE_ERROR = 41
    UNSUPPORTED_OPTION = 42
    UNSUPPORTED_RESOLUTION = 43
    UNSUPPORTED_FRAME_RATE = 44
    UNSUPPORTED_BIT_RATE = 45
    UNSUPPORTED_ACTION = 46
    UNSUPPORTED_FOCUS_ACTION = 47
    VALUE_IS_ABOVE_MAXIMUM = 48
    VALUE_IS_BELOW_MINIMUM = 49
    DEVICE_DISCONNECTED = 50
    DEVICE_NOT_READY = 51
    INVALID_DEVICE = 52
    SAFETY_THRESHOLD_REACHED = 53
    INVALID_USER_SESSION_ACCESS = 54
    CONTROL_MANUAL_STOP = 55
    CONTROL_OUTSIDE_WORKSPACE = 56
    CONTROL_ACTUATOR_COUNT_MISMATCH = 57
    CONTROL_INVALID_DURATION = 58
    CONTROL_INVALID_SPEED = 59
    CONTROL_LARGE_SPEED = 60
    CONTROL_INVALID_ACCELERATION = 61
    CONTROL_INVALID_TIME_STEP = 62
    CONTROL_LARGE_SIZE = 63
    CONTROL_WRONG_MODE = 64
    CONTROL_JOINT_POSITION_LIMIT = 65
    CONTROL_NO_FILE_IN_MEMORY = 66
    CONTROL_INDEX_OUT_OF_TRAJECTORY = 67
    CONTROL_ALREADY_RUNNING = 68
    CONTROL_WRONG_STARTING_POINT = 69
    CONTROL_CARTESIAN_CANNOT_START = 70
    CONTROL_UNDEFINED_CONSTRAINT = 71
    CONTROL_UNINITIALIZED = 72
    CONTROL_NO_ACTION = 73
    CONTROL_UNDEFINED = 74
    WRONG_SERVOING_MODE = 75
    CONTROL_WRONG_STARTING_SPEED = 76
    USERNAME_LENGTH_EXCEEDED = 100
    FIRSTNAME_LENGTH_EXCEEDED = 101
    LASTNAME_LENGTH_EXCEEDED = 102
    PASSWORD_LENGTH_EXCEEDED = 103
    USERNAME_ALREADY_EXISTS = 104
    USERNAME_EMPTY = 105
    PASSWORD_NOT_CHANGED = 106
    MAXIMUM_USER_PROFILES_USED = 107
    ROUTER_UNVAILABLE = 108
    ADDRESS_NOT_IN_VALID_RANGE = 120
    ADDRESS_NOT_CONFIGURABLE = 121
    SESSION_NOT_IN_CONTROL = 130
    METHOD_TIMEOUT = 131
    UNSUPPORTED_ROBOT_CONFIGURATION = 132
    NVRAM_READ_FAIL = 133
    NVRAM_WRITE_FAIL = 134
    NETWORK_NO_ADDRESS_ASSIGNED = 135
    READ_PERMISSION_DENIED = 136
    CONTROLLER_INVALID_MAPPING = 137
    ACTION_IN_USE = 138
    SEND_FAILED = 139
    CONTROL_WAYPOINT_TRAJECTORY_ABORTED = 140
    CONTROL_PERMISSION_DENIED = 141

    @staticmethod
    def is_valid_error(value: int) -> bool:
        """
        Checks if an integer(error number) is the value for the RobotErrorEnum members

        The method __call__(value) raises a ValueError
        if the value doesn't belong to any class member

        :param value: An integer to look in the class members values
        :return: True if the class contains this value
        """
        try:
            RobotSubErrorCodesEnum.__call__(value)
        except (ValueError,):
            return False
        return True


if __name__ == "__main__":
    robot_error1 = RobotErrorEnum.JOINT_DETECTION_ERROR
    robot_error2 = RobotErrorEnum.BRAKE_REMOVAL_FAILURE
    try:
        RobotErrorEnum(0)
    except (ValueError,) as e:
        print(e)

    print(robot_error1.name)
    print(robot_error1.value)
    print(robot_error2.name)
    print(robot_error2.value)
