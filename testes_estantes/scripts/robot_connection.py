import gc
import sys

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.GripperCyclicClientRpc import GripperCyclicClient
from kortex_api.autogen.messages import Session_pb2


class RobotConnection:
    """
    Class that manages connection
        
    """

    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def create_tcp_connection(ip: str = "192.168.2.10",
                              username: str = "admin",
                              password: str = "admin"):
        """
        returns RouterClient required to create
        services and send requests to device or sub-devices,
        
        """

        return RobotConnection(ip, port=RobotConnection.TCP_PORT, credentials=(username, password))

    @staticmethod
    def create_udp_connection(ip: str = "192.168.2.10",
                              username: str = "admin",
                              password: str = "admin"):
        """
        returns RouterClient that allows to create services and send requests
        to a device or its sub-devices @ 1khz.
        
        """

        return RobotConnection(ip, port=RobotConnection.UDP_PORT, credentials=(username, password))

    def __init__(self, ip_address, port=TCP_PORT, credentials=("", "")):

        self.router = None
        self.transport = None
        self.device_config = None
        self.base_cyclic = None
        self.gripper = None
        self.base = None
        self.ip_address = ip_address
        self.port = port
        self.credentials = credentials

        self.session_manager = None

    def connect(self):
        """
        Method responsible for connecting robot. It returns the following tuple:
        (BaseClient, GripperCyclicClient, BaseCyclicClient, DeviceConfigClient)
        """
        # Setup API
        self.transport = TCPTransport() if self.port == RobotConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)
        self.transport.connect(self.ip_address, self.port)

        if self.credentials[0] != "":
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 40000  # (milliseconds)
            session_info.connection_inactivity_timeout = 20000  # (milliseconds)

            self.session_manager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ip_address)
            self.session_manager.CreateSession(session_info)
        else:
            raise Exception("No credentials provided")

        return

    def disconnect(self):
        """"
        Method responsible for disconnecting robot, by closing SessionManager object

        It returns the following tuple:
        (BaseClient, GripperCyclicClient, BaseCyclicClient, DeviceConfigClient)

        """
        session_manager_is_valid = self.session_manager is not None
        transport_is_valid = self.transport is not None

        if not transport_is_valid:
            return

        if session_manager_is_valid:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 100

            self.session_manager.CloseSession(router_options)

        self.transport.disconnect()

    def get_base_client(self) -> BaseClient:
        self.check_router_connection()
        return BaseClient(self.router)

    def get_gripper_cyclic_client(self) -> GripperCyclicClient:
        self.check_router_connection()
        return GripperCyclicClient(self.router)

    def get_base_cyclic_client(self) -> BaseCyclicClient:
        self.check_router_connection()
        return BaseCyclicClient(self.router)

    def get_device_config_client(self) -> DeviceConfigClient:
        self.check_router_connection()
        return DeviceConfigClient(self.router)

    def check_router_connection(self):
        if self.router is None:
            msg = "Empty router object. Make sure there is a router object available."
            raise Exception(msg)
