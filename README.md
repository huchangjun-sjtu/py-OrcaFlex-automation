# py-OrcaFlex-automation
A Python-based demo for controlling OrcaFlex.
![Link](https://github.com/user-attachments/assets/5ca76be1-ae7f-4bb0-b068-b294dd63ae23)
## 1. `ExternalFunctionSourceCPP`
**功能描述**:  
这是 OrcaFlex 仿真软件的扩展源代码文件，用于实现与外部系统的动态交互功能。

**核心模块**:  
- **DynamicPositioningSystem.cpp**  
  负责仿真运行期间通过 TCP 协议与外部控制程序进行实时数据交互，支持动态定位指令传输和状态反馈。

- **官方 API 引擎**  
  包含完整的 OrcFxAPI 接口实现（遵循 [OrcFxAPI 文档](https://www.orcina.com/webhelp/OrcFxAPI/)），提供底层仿真控制能力。
---

## 2. `OrcaFlexControllerServer`
**角色定位**:  
控制中心服务进程，负责管理仿真软件生命周期。

**关键特性**:  
- **命令监听**：持续监听来自 `OrcaFlexControllerClient` 的启动/停止指令
- **状态管理**：维护仿真进程与外部控制器的连接状态
---

## 3. `OrcaFlexControllerClient`
**功能定位**:  
开发者可调用的客户端 SDK 库，提供控制接口。

**核心功能**:  
```python
# 示例代码片段
from OrcaFlexControllerClient import OrcaFlexShip

Ship = OrcaFlexShip()

# 启动仿真
Ship.reset(init_x, init_y, init_psi,cfg)

# 获取实时数据
global_pose = Ship.step(global_force)
