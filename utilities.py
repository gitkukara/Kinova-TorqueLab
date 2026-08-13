"""Kortex TCP/UDP 连接和会话管理辅助工具。"""

import argparse

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

def parseConnectionArguments(parser=argparse.ArgumentParser()):
    """添加 Kortex 连接参数并解析命令行。"""

    parser.add_argument("--ip", type=str, help="机械臂 IP 地址", default="192.168.1.10")
    parser.add_argument("-u", "--username", type=str, help="登录用户名", default="admin")
    parser.add_argument("-p", "--password", type=str, help="登录密码", default="admin")
    return parser.parse_args()

class DeviceConnection:
    """在 ``with`` 代码块内建立并自动关闭 Kortex 连接。"""
    
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(args):
        """创建用于常规服务和请求的 TCP RouterClient。"""

        return DeviceConnection(args.ip, port=DeviceConnection.TCP_PORT, credentials=(args.username, args.password))

    @staticmethod
    def createUdpConnection(args):
        """创建用于高频周期通信的 UDP RouterClient。"""

        return DeviceConnection(args.ip, port=DeviceConnection.UDP_PORT, credentials=(args.username, args.password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # 根据端口创建 TCP 或 UDP 传输层，并交给 Kortex RouterClient 管理。
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    def __enter__(self):
        """进入 ``with`` 代码块时连接设备并创建登录会话。"""
        
        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000  # 单位：ms
            session_info.connection_inactivity_timeout = 2000  # 单位：ms

            self.sessionManager = SessionManager(self.router)
            print("正在登录设备：", self.ipAddress, "，用户名：", self.credentials[0])
            self.sessionManager.CreateSession(session_info)

        return self.router

    def __exit__(self, exc_type, exc_value, traceback):
        """退出 ``with`` 代码块时关闭会话并断开传输连接。"""
    
        if self.sessionManager != None:

            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000 
            
            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()
