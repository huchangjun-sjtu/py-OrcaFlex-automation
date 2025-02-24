# -*- coding: utf-8 -*-
# Copyright (c) 2025 Huchangjun <huchangjun1999@qq.com>
#
# 这是用于客户端的Orcaflex交互代码，应在OrcaFlexControllerServer运行后启动.
#
# 主要功能：
#   1. 向OrcaFlexControllerServer发送指令进行仿真参数设定与仿真启停
#   2. 与正在运行的OrcaFlex程序通过ExternalFunction建立链接，接受实时位置，发送实时控制力
#
import socket
import re
import threading
# from subprocess import HIGH_PRIORITY_CLASS
import numpy as np


class OrcaFlexEnv:
    def __init__(self) -> None:
        self.wave_Hs = 0     # 波高m
        self.wave_Tz = 8     # 周期s
        self.wave_Dir = 0    # 来浪方向 deg
        self.current_Speed = 0 #流速 0.5
        self.current_Dir = 0 #方向
        self.wind_Speed = 0 #风速
        self.wind_Dir = 0 #风向
        self.duration = 10000

# ######################################### Fun for connect Orcaflex sim using Tcp ################
def D2R(deg):
    return deg / 180 * np.pi

def R2D(rad):
    return rad / np.pi * 180

def pose_unpack(_recvbuf: bytes):
    # recvbuf 形如 #x3004.296164y2756.962127z-165.185877e
    recvbuf = str(_recvbuf, encoding="utf8")
    x = y = psi = 0.0
    bool_recv = True
    if recvbuf[0] != '#':
        return x, y, psi, False
    searchObj = re.search(r'#x(.*)y(.*)z(.*)e.*', recvbuf, re.M | re.I)
    x = float(searchObj.group(1))
    y = float(searchObj.group(2))
    psi = D2R(float(searchObj.group(3)))
    return x, y, psi, bool_recv


def force_pack(X: float, Y: float, N: float):
    sendforce = "#x" + str(X) + "y" + str(Y) + "z" + str(N) + "e"
    # print("send:",sendforce)
    sendbuf = bytes(sendforce, encoding="utf8")
    return sendbuf


def communicate_orcaflex_sim(connectionSocket: socket, X, Y, N):
    sendbuf = force_pack(X, Y, N)
    # 向客户端发送数据
    connectionSocket.send(sendbuf)
    # print("Send Force: x: ", X, " y: ", Y, " heading: ", N)
    # 接收客户端发送数据
    recvbuf = connectionSocket.recv(1024)
    x, y, psi, bool_recv = pose_unpack(recvbuf)
    # print("Get position: x: ",x," y: ",y," heading: ",psi)

    return x, y, psi

def prepare_server_socket():
    serverPort = 2222 #3334
    # 创建套接字
    # 第一个参数表示使用的地址类型，一般都是ipv4即AF_INET
    # 第二个参数表示套接字类型：tcp一般用SOCK_STREAM数据流传输
    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # 绑定套接字
    serverSocket.bind(('', serverPort))
    # 从client监听请求，参数是能连接的最大值
    serverSocket.listen(1)
    # print('the RL server for connect to sim is ready')
    return serverSocket

# ##################################### End Fun for connect Orcaflex sim using Tcp #############

# ######################################### Fun for connect Orcaflex controller using Tcp ############
def prepare_client_socket():
    serverName = socket.gethostbyname(socket.gethostname())  
    # hostname代表运行TCPServer.py的计算机IP,这里是本机
    print(f"hostIP is :{socket.gethostname()}")
    serverPort = 8888
    clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 第一个参数代表IPv4，第二个代表TCP
    clientSocket.connect((serverName, serverPort))  # TCP需要建立连接
    print('the RL client for connect to orcaflex controller is ready')
    return clientSocket

def pack_sim_parameter(x, y, psi, wave_Hs, wave_Tz, wave_Dir,
                       current_Speed, current_Dir, wind_Speed, wind_Dir, duration):
    sendparameter = "#x"+str(x) + "y"+str(y) + "z"+str(psi) + "wh"+str(wave_Hs) + "wt"+str(wave_Tz) + "wd"+str(wave_Dir) \
                + "cs"+str(current_Speed) + "cd"+str(current_Dir) + "ws"+str(wind_Speed) + "wD"+str(wind_Dir) \
                + "D"+str(duration)+ "e"
    # print("send:",sendparameter)
    sendbuf = bytes(sendparameter, encoding="utf8")
    return sendbuf

def order_new_sim(clientSocket,sendParameterBuf):
    clientSocket.send(sendParameterBuf)  # 把sentence通过socket直接发送到TCP连接里
    back = clientSocket.recv(1024)
    # print("start sim Command executed")
    return None

def order_stop_sim(clientSocket):
    stop = "#!stop!"
    sendbuf = bytes(stop, encoding="utf8")
    clientSocket.send(sendbuf)  # 把sentence通过socket直接发送到TCP连接里
    back = clientSocket.recv(1024)
    # print("stop sim Command executed")
    return None

def order_close_connect(clientSocket):
    stop = "#$close"
    sendbuf = bytes(stop, encoding="utf8")
    clientSocket.send(sendbuf)  # 把sentence通过socket直接发送到TCP连接里
    # print("close connect to orcaflex connect Command executed")
    return None

# #################################### end Fun for connect Orcaflex controller using Tcp ############

class OrcaFlexShip:

    def __init__(self):
        self.simulationSocket = socket.socket()
        self.serverSocket = socket.socket()
        self.dt_out = 0.2  # 5Hz

        # ############## build connect with orcaflex controller server
        self.clientSocket = prepare_client_socket()
        

    def reset(self,init_x, init_y, init_psi,cfg):
        # OrcaFlex环境力参数
        wave_Hs = cfg.simEnv.wave_Hs
        wave_Tz = cfg.simEnv.wave_Tz     # 周期s
        wave_Dir = cfg.simEnv.wave_Dir   # 来浪方向 deg
        current_Speed = cfg.simEnv.current_Speed #流速
        current_Dir = cfg.simEnv.current_Dir #方向
        wind_Speed = cfg.simEnv.wind_Speed #风速
        wind_Dir = cfg.simEnv.wind_Dir #风向
        duration = cfg.simEnv.duration

        # 启动OrcaFlex并建立通讯---------------------------------------------------
        # # 给orcaflex controller 指令停止旧的 simulation
        order_stop_sim(self.clientSocket)
        # # 准备好与orcaflex simulation 通讯的服务器套接字
        self.simulationSocket.close()
        self.serverSocket.close()
        self.serverSocket = prepare_server_socket()
        t1 = threading.Thread(target=self.connect_orcaflex_sim, args=(self.serverSocket,))
        t1.start()
        # # 给orcaflex controller 指令启动新的 simulation
        modelParaBuf = pack_sim_parameter(init_x, init_y, init_psi, wave_Hs, wave_Tz,
                                                wave_Dir,current_Speed, current_Dir, wind_Speed, wind_Dir, duration)
        order_new_sim(self.clientSocket, modelParaBuf)                            
        t1.join()
        # 完成建立通讯---------------------------------------------------

        # 与本次仿真Case交互
        get_x, get_y, get_psi = communicate_orcaflex_sim(self.simulationSocket, 0, 0, 0) #KN 注意单位 global_force TODO

        return [get_x, get_y, get_psi]

    def step(self, global_force):
        get_x, get_y, get_psi = communicate_orcaflex_sim(self.simulationSocket, global_force[0], global_force[1], global_force[2]) #KN 注意单位 global_force TODO
        return [get_x, get_y, get_psi]

    def connect_orcaflex_sim(self,serverSocket):
        # 接受客户端连接请求
        # 收到connectionSocket：套接字,负责与链接到的客户端收发数据
        # 收到addr：即客户端的地址与端口号
        connectionSocket, addr = serverSocket.accept()
        self.simulationSocket = connectionSocket

    def close(self):
        order_stop_sim(self.clientSocket)
        self.simulationSocket.close()
        self.serverSocket.close()
        order_close_connect(self.clientSocket)
        self.clientSocket.close()
        return
