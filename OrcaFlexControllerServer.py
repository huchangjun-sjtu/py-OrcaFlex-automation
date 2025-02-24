# -*- coding: utf-8 -*-
# Copyright (c) 2025 Huchangjun <huchangjun1999@qq.com>
#
# 这是用于服务端的Orcaflex交互代码，应首先启动.
# 主要功能：
#   1. 调用OrcFxAPI操控OrcaFlex的启动停止及参数加载
#   2. 监听客户端的指令，按需启停

import OrcFxAPI  # 参考手册安装https://www.orcina.com/webhelp/OrcFxAPI/Default_Left.htm#StartTopic=html/Pythoninterface,Introduction.htm|SkinName=Web%20Help
import numpy as np
import socket
import re
import threading
import time


def parameter_unpack(_recvbuf: bytes):
    # 接受Orcaflex设置参数顺序：initial_x, initial_y, initial_heading, wave_Hs, wave_Tz, wave_Dir, current_Speed,
    #                        current_Dir, wind_Speed, wind_Dir, duration
    # recvbuf 形如 #x0.0y0.0z10.0wh1.5wt5.0wd180.0cs0.5cd180ws5.0wD180.0D1000.0e
    # recvbuf = str(_recvbuf, encoding="utf8")
    recvbuf = _recvbuf
    x = y = psi = wave_Hs = wave_Tz = wave_Dir = current_Speed = current_Dir = wind_Speed = wind_Dir = duration = 0.0
    bool_recv = True
    if recvbuf[0] != '#':
        print("wrong orcaflex parameter")
        return x, y, psi, wave_Hs, wave_Tz, wave_Dir, current_Speed, current_Dir, wind_Speed, wind_Dir, duration, False
    searchObj = re.search(r'#x(.*)y(.*)z(.*)wh(.*)wt(.*)wd(.*)cs(.*)cd(.*)ws(.*)wD(.*)D(.*)e.*', recvbuf, re.M | re.I)
    x = float(searchObj.group(1))
    y = float(searchObj.group(2))
    psi = float(searchObj.group(3))
    wave_Hs = float(searchObj.group(4))
    wave_Tz = float(searchObj.group(5))
    wave_Dir = float(searchObj.group(6))
    current_Speed = float(searchObj.group(7))
    current_Dir = float(searchObj.group(8))
    wind_Speed = float(searchObj.group(9))
    wind_Dir = float(searchObj.group(10))
    duration = float(searchObj.group(11))
    return x, y, psi, wave_Hs, wave_Tz, wave_Dir, current_Speed, current_Dir, wind_Speed, wind_Dir, duration, bool_recv


def force_pack(X: float, Y: float, N: float):
    sendforce = "#x" + str(X) + "y" + str(Y) + "z" + str(N) + "e"
    # print("send:",sendforce)
    sendbuf = bytes(sendforce, encoding="utf8")
    return sendbuf


def controller_server_prepare():
    '''
    连接到主程序，返回与之通信的套接字，以便根据主程序要求启动orcaflex
    :return:用于通信的套接字
    '''
    serverPort = 8888
    # 创建套接字
    # # 第一个参数表示使用的地址类型，一般都是ipv4即AF_INET
    # # 第二个参数表示套接字类型：tcp一般用SOCK_STREAM数据流传输
    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 绑定套接字
    serverSocket.bind(('', serverPort))
    # 从client监听请求，参数是能连接的最大值
    serverSocket.listen(1)
    print('the orcaflex controller server for connect to RL main is ready')
    return serverSocket


def connect_main(serverSocket):
    # 接受客户端连接请求
    # # 收到connectionSocket：套接字负责与链接到的客户端收发数据
    # # 收到addr：即客户端的地址与端口号
    print('the Orcaflex controller server is waiting...')
    connectionSocket, addr = serverSocket.accept()
    print(f'IP is:{addr}')
    print('the Orcaflex controller server is connected to RL main')
    return connectionSocket


# ######################################### Fun for Ctrl Orcaflex ######################################################
OrcaFlexStop = False

def D2R(deg):
    return deg / 180 * np.pi


def R2D(rad):
    return rad / np.pi * 180


def SetVesselInitialPosition(Vessel, x, y, heading):
    L = 65.69
    X = x - L * np.cos(D2R(heading))  # trans origin to gravity center
    Y = y - L * np.sin(D2R(heading))
    Vessel.SetData('InitialX', 1, X)  # m
    Vessel.SetData('InitialY', 1, Y)  # m
    Vessel.SetData('InitialHeading', 1, heading)  # deg(-360,360)
    return None


def GetVesselInitialPosition(Vessel):
    print(f"Initial X of Vessel ={Vessel.GetData('InitialX', 1)};")
    print(f"Initial Y of Vessel ={Vessel.GetData('InitialY', 1)};")
    print(f"Initial Heading of Vessel ={Vessel.GetData('InitialHeading', 1)};")
    return None


def SetStageTimeStep(general, duration):
    '''
   Used to set the number of timesteps for each run.
   Here, the internal timestep is 0.01s, and the total simulation time is 0.05*timesteps
   '''
    general.InnerTimeStep = 0.05  # InnerTimeStep =0.05s
    # duration = 0.05*steps
    general.StageDuration = 0.01, duration
    return None


def SetEnvParameter(env, wave_Hs, wave_Tz, wave_Dir, current_Speed,
                    current_Dir, wind_Speed, wind_Dir):
    '''
   Used to set the environment.
   '''
    env.WaveDirection = wave_Dir  # deg
    env.WaveHs = wave_Hs  # m
    env.WaveTz = wave_Tz  # s
    env.RefCurrentSpeed = current_Speed  # m/s
    env.RefCurrentDirection = current_Dir  # deg
    env.WindSpeed = wind_Speed  # m/s
    env.WindDirection = wind_Dir  # deg


def DynamicsProgressHandler(model, time, start, stop):
    Cancel = False
    #Cancel = OrcaFlexStop
    # print(f"111Now, the OrcaFlexStop is {OrcaFlexStop},the state of orcaflex model is {model.state}")
    if OrcaFlexStop:
        model.PauseSimulation()
        Cancel = True
        # model.Reset()
    return Cancel


def StartNewSimulation(initial_x, initial_y, initial_heading, wave_Hs, wave_Tz, wave_Dir, current_Speed,
                       current_Dir, wind_Speed, wind_Dir, duration):
    # # # Load model files and parameters ##
    model_name = 'D:/SKLOE/RL_workspace/RL_DP/orcaflex/oiltank/tanker.dat'
    attempts = 0
    success = False
    while attempts<3 and not success: # 防止上一进程正在访问该文件
        try:
            model = OrcFxAPI.Model(model_name)
            success = True
        except:
            time.sleep(1)
            attempts +=1
            print(f"retry : {attempts} times")
            if attempts == 3:
                break
    
    vessel = model['Vessel1']
    general = model.general
    env = model.environment
    # # Initialize the model and calculate parameters ##
    SetVesselInitialPosition(vessel, initial_x, initial_y, initial_heading)  # (x_m,y_m,heading_deg)
    GetVesselInitialPosition(vessel)
    SetEnvParameter(env, wave_Hs, wave_Tz, wave_Dir, current_Speed,
                    current_Dir, wind_Speed, wind_Dir)
    SetStageTimeStep(general, duration)
    
    # # Start Simulation ##
    print(f"Now, the orcaflex model is starting, OrcaFlexStop is {OrcaFlexStop}")
    model.dynamicsProgressHandler = DynamicsProgressHandler  # Monitor the running of the sim
    model.RunSimulation()

    print(f"222Now, the state of orcaflex model is {model.state}")


# ######################################### End Fun for Ctrl Orcaflex ##################################################


# ------------------------main---------------------------
if __name__ == '__main__':
    # # origin model config set
    # model_name = 'oiltanker_try.dat'
    initial_x = 0  # m
    initial_y = 0  # m
    initial_heading = 0  # deg
    duration = 10800  # s
    wave_Hs = 1.5  # m
    wave_Tz = 8  # s
    wave_Dir = 270  # deg
    current_Speed = 0.5  # m/s
    current_Dir = wave_Dir  # deg
    wind_Speed = 5  # m/s
    wind_Dir = wave_Dir  # deg
    simulations = [] # simulation queue
    # # build communication with RL main ##
    serverSocket = controller_server_prepare()
    RL_connectionSocket = connect_main(serverSocket)
    IsConnect = True
    while True:
        if IsConnect:
            # 接收客户端发送数据
            recvbuf = RL_connectionSocket.recv(1024)
            if len(recvbuf) > 0:
                recvbuf = str(recvbuf, encoding="utf8")
                if recvbuf[0] == '#':
                    if recvbuf[1] == 'x':
                        print("Received Start command")
                        if len(simulations) > 0:
                            simulations[-1].join()
                            simulations.remove(simulations[-1])
                        OrcaFlexStop = False
                        initial_x, initial_y, initial_heading, wave_Hs, wave_Tz, wave_Dir, current_Speed, current_Dir, wind_Speed, \
                        wind_Dir, duration, bool_recv = parameter_unpack(recvbuf)
                        simulation = threading.Thread(target=StartNewSimulation,
                                              args=(initial_x, initial_y, initial_heading, wave_Hs, wave_Tz,
                                                    wave_Dir, current_Speed, current_Dir, wind_Speed, wind_Dir, duration,))
                        simulations.append(simulation)
                        simulations[-1].start()
                        print(f"the {len(simulations)} sim is start")
                        RL_connectionSocket.send(bytes("Command start executed", encoding="utf8"))
                    elif recvbuf[1] == '!':
                        print("Received Stop command")
                        OrcaFlexStop = True
                        print(f"the {len(simulations)} sim is stop")
                        RL_connectionSocket.send(bytes("Command Stop executed", encoding="utf8"))
                    elif recvbuf[1] == '$':
                        print("Received Close command")
                        OrcaFlexStop = True
                        RL_connectionSocket.close()
                        IsConnect = False
                        print("Last RL_connectionSocket Closed")
        else:
            RL_connectionSocket = connect_main(serverSocket)
            IsConnect = True
