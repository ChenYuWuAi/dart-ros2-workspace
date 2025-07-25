# 飞镖ROS工作空间

## 如何克隆本仓库

```shell
git clone --recurse-submodules git@github.com:ChenYuWuAi/dart-ros2-workspace.git
```

> 本工作空间是飞镖机器人的**ROS**工作空间，包含了飞镖机器人的所有ROS功能包。

> 目前工作流依赖`Github Pull Request`，需要**fork**仓库之后向上游提交pull request。

```mermaid
graph LR
    A[Dart ROS2 Workspace] --> B[dart_msgs]
    A --> C[dart_flysystem_hardware]
    A --> D[dart_flysystem_description]
    A --> E[dart_detector]
    A --> F[dart_launcher]
    A --> G[dart_mcu]
```

## 自瞄支持包(dart_detector)
> 自瞄支持包是飞镖机器人的自瞄功能的ROS功能包，可以检测目标绿灯，并且计算目标的屏幕坐标。

其通信协议如下(ROS msg格式)：

/dart_msgs/**GreenLight.msg**
``` protobuf
std_msgs/Header header
bool is_detected
geometry_msgs/Point location
```

通讯时序图如下：
```mermaid
sequenceDiagram
    participant B as 相机节点(camera_node)
    participant A as 自瞄响应节点(dart_detector_node)
    participant C as 飞镖发射架UI(node_lvgl_ui)
    B->>+A: /camera/image
    A->>-C: /dart_detector/green_light
```

注意，自瞄支持包使用的是**OpenCV**库，需要自行编译安装。

/camera/image是飞镖机器人的相机节点(camera_node)，来源为大华工业相机SDK。

/camera/image的消息通过**cv_bridge**与**OpenCV**的**Mat**类型进行互转，然后进行目标检测。

飞镖架上使用的相机不需要更换，但是飞镖上的相机为MIPI接口，需要自行编写驱动，通过**v4l2**接口获取图像。

自瞄支持包内的**Parameter.cpp**可以编译为可执行文件，用于导入数据集，调试自瞄算法的参数。详参[Parameter.cpp](src/dart_detector/src/Parameters.cpp)。

### 重构RoadMap
1. 把**Parameter.cpp**的功能整合到**dart_detector**包内，可以通过ros2 run一键调用调参软件。
2. 把Detector和Camera分开，增加一个Camera的功能包，用于支持多种相机。
2. 引入多种驱动，支持多种相机，多种架构(ARM/x86)。
3. 需要一些测试用例，用于测试系统的稳定性。
4. 添加(主要是制导镖的)相机的参数同步功能。
5. 引入生命周期(ROS2 Lifecycle)控制。

### 重构后的架构
```mermaid
graph LR
    A[ROS Package:
    dart_detector]
    A --> G[ParameterTuner] --> F[Parameter]
    B[ROS Package:
    dart_camera]
    A --> E[NodeDartDetector] --> F[Detector]
    B --> C[NodeCamera] --> D[CameraDriver（实现）]
```
> 注意，这里的CameraDriver是一个基类的实现，需要使用pluginlib编写插件，用于支持不同的相机。
```mermaid
classDiagram
    class CameraDriver{
        <<abstract>>
        +open(std::unordered_map<string, string>)
        +close()
        +CV::Mat read()
        +bool write(std::string, std::string)
    }
    class CameraDriverRPI{
        +open(std::unordered_map<string, string>)
        +close()
        +CV::Mat read()
        +bool write(std::string, std::string)
    }
    class CameraDriverDH{
        +open(std::unordered_map<string, string>)
        +close()
        +CV::Mat read()
        +bool write(std::string, std::string)
    }
    class CameraDriverMIPI{
        +open(std::unordered_map<string, string>)
        +close()
        +CV::Mat read()
        +bool write(std::string, std::string)
    }
    CameraDriver <|-- CameraDriverDH
    CameraDriver <|-- CameraDriverMIPI
    CameraDriver <|-- CameraDriverRPI
    
```

## 飞镖架发射控制包(dart_launcher)
> 飞镖架发射控制包是飞镖机器人的发射控制功能的ROS功能包。主要功能是飞镖架的上位机控制和发射参数池的维护。内置非线性回归机器学习算法(依赖于GSL库)，可以由参数数据集拟合出合适的发射参数，可以通过LVGL UI界面调用/调整发射参数。

dart24版本最终架构图如下：
```mermaid
graph LR
    A[node_lvgl_ui.cpp
     NodeLVGLUI
     GUI主节点]
    A --> C[dart_algorithm.cpp
    DartAlgorithm
    发射算法支持库]
    A --> D[get_ip.cpp
    getIPAddress
    获取IP地址]
    B[dart_calibrate.cpp
    dart_calibration
    发射算法调试器] --> C
    A --> E[dart_config.cpp
    DartConfig
    发射参数声明公用库]
    A --> F[node_dart_config.cpp
    NodeDartConfig
    发射参数池节点]
    G[node_can_agent.cpp
    NodeCanAgent
    CAN总线代理节点] --> F
    A <--> G
    F --> E
    H[node_dart_logger_dog.cpp:
    NodeDartLoggerDog
    看门狗节点]
```

GUI更新是在node_lvgl_ui.cpp中，按照定时器触发的方式，由LVGL库绘制画布并刷新/dev/fb0，同时获取触摸屏的输入。根据触发的事件，调用其他节点的服务，完成发射参数的调整。

通信格式如下：
/dart_msgs/**DartLauncherStatus.msg**
``` protobuf
std_msgs/Header header
bool motor_ls_online 0
bool motor_y_online 0
bool motor_dm_online 0
bool judge_online 0
bool[4] motor_fw_online [0,0,0,0]
bool rc_online 0
bool dart_launcher_online 0

uint8 dart_state 255 # boot = 100, protect = 101, remote = 102, match = 103-106 Enter Wait Launch Reload, undefined=255

bool motor_y_resetting 0
bool motor_dm_resetting 0
bool motor_ls_resetting 0

uint8 dart_launch_process 0

int32 motor_y_angle 0
int32[4] motor_fw_velocity [0,0,0,0]
float64[4] motor_fw_current [0,0,0,0]

float64[4] bus_voltage [0,0,0,0]
```
/dart_msgs/**DartParam.msg**
``` protobuf
std_msgs/Header header
int32 target_yaw_angle 0
int32 target_yaw_angle_offset 0 # Debug mode only
int32 target_fw_velocity 0
int32 target_fw_velocity_offset 0 # Debug mode only
float64 target_fw_velocity_ratio 0
int32[4] target_yaw_launch_angle_offset [0, 0, 0, 0]
int32[4] target_fw_velocity_launch_offset [0, 0, 0, 0]
bool auto_yaw_calibration 0
bool auto_fw_calibration 0
float64 target_yaw_x_axis 640.0
float64 target_distance 0.0
float64 target_delta_height 0.42
string[4] dart_selection ['Default', 'Default', 'Default', 'Default']
```
/dart_msgs/**Judge.msg** 裁判系统数据包
``` protobuf
std_msgs/Header header
uint8 dart_launch_opening_status
uint8 game_progress
uint8 dart_remaining_time
uint16 latest_launch_cmd_time
uint16 stage_remain_time
bool judge_online
```
具体订阅关系图，请参考RQT图。
打开方式，执行ros2 launch dart_launcher dart.launch.py之后，打开rqt，选择topic，然后选择dart_launcher，即可看到所有的订阅关系。

### 重构RoadMap
1. 增加BLE通信功能，用于与待发射的飞镖进行通信，通知飞镖校准陀螺仪、启动发动机，进入发射程序。
2. 针对拉簧飞镖制式，设计新的UI，修改发射参数池的内容，将摩擦轮转速参数改为扳机位置参数。
3. 重构config的数据流，把飞镖参数池和发射参数池分开，加入发射参数的存档功能。
4. 针对下位机切换为MicroROS，需要重新设计该包的数据流，简少维护压力。
5. 引入生命周期(ROS2 Lifecycle)控制。
6. 改进抛物线预测算法。他还不太准。

### 重构数据流
#### 最重要的是这个**Config Manager**的重新编写。写得好是可以拿来给开源社区用的。
首先，分析之前的数据流。
```mermaid
sequenceDiagram
    participant A as NodeDartConfig
    participant B as NodeLVGLUI
    participant C as NodeCanAgent
    participant D as NodeCamera
    participant E as NodeDetector

    A->>+A: Load Config
    A->>+B: Config同步到node parameter
    B->>B: 向UI更新参数，按参数内容计算发射参数
    deactivate B
    A->>C: Config同步到node parameter
    A->>D: Config同步到node parameter
    A->>E: Config同步到node parameter
    deactivate A
    B->>B: 用户调整参数
    activate B
    B->>C: /dart_launcher/dart_launcher_param_cmd
    activate C
    B->>+A: /dart_launcher/dart_launcher_param_cmd
    C->>C: 发射参数同步到CAN总线
    deactivate C
    A->>-A: 更新到文件
    deactivate B
    activate C
    C->>C: CAN总线上参数变化
    C->>+A: /dart_launcher/dart_launcher_param_cmd
    A->>-A: 更新到文件
    C->>B: /dart_launcher/dart_launcher_param_cmd
    deactivate C
    activate B
    B->>B: 向UI更新参数，按参数内容计算发射参数
    deactivate B
    D->>E: /camera/image
    E->>+B: /dart_detector/green_light
    B->>-B: 更新UI参数，计算发射参数
    B->>+C: /dart_launcher/dart_launcher_param_cmd
    B->>+A: /dart_launcher/dart_launcher_param_cmd
    A->>-A: 更新到文件
    C->>-C: 发射参数同步到CAN总线
```

抽象出新的拉簧发射架发射控制应用层的参数如下：
```mermaid
classDiagram
    class DartLauncherParams{
        +int32 primary_yaw, 发射目标主偏航角
        +int32 primary_yaw_offset, 发射目标主偏航角偏移量, 调试用
        +int32 primary_pitch, 发射目标主俯仰角
        +int32 primary_force, 发射扳机主位置
        +int32 primary_force_offset, 发射扳机主位置偏移量, 调试用
        +int32[4] auxiliary_yaw_offsets, 发射目标副偏航角偏移量, 按程序发射时使用
        +int32[4] auxiliary_force_offsets, 发射扳机副位置偏移量, 按程序发射时使用
        +int16 sequence_offset, 从第几发开始发射，调试用，若该值大于等于4，则取余
        +uint64 last_param_update_time, 上次参数更新时间, 用于同步, 单位ms 高位优先
    }
```

拉簧发射架状态回传数据如下：
```mermaid
classDiagram
    class DartLauncherStatus{
        +bool motor_pitch_online, 电机Pitch在线
        +bool motor_yaw_online, 电机Yaw在线
        +bool[2] motor_loader_online, 电机Load[0, 1]在线
        +bool motor_trigger_online, 电机Trigger在线
        +bool judge_online, 裁判系统在线
        +bool in_game, 是否在赛场上
        +bool rc_online, 遥控器在线
        +uint8 dart_state, 发射架状态, 100 Boot, 101 Protect, 102 Remote, 103-106 Match Enter Wait Launch Reload, 255 Undefined
        +uint8 dart_launch_process, 发射流程(1/4)
        +int32 motor_pitch_angle, 电机Pitch角度
        +int32 motor_yaw_angle, 电机Yaw总角度
        +int32 motor_trigger_angle, 电机Trigger总角度
        +bool servo_trigger_state, 伺服Trigger状态, 0初始, 1触发
        +int32[2] motor_loader_current, 电机Load[0, 1]相电流
        +int32[2] motor_loader_angle, 电机Load[0, 1]总角度
        +double last_launch_speed, 上次发射检测速度
        +int32 last_launch_time, 上次发射检测数据时间UNIX戳，若无时间戳/speed则为0
    }
```

构建传输层的数据结构如下：
基于 `std_msgs/Int32MultiArray` 的定义方案，通过约定数组中各元素的含义，将所有参数打包到一个 `int32` 数组中。假设数组总长度为 14，每个索引代表的参数定义如下：

| 数组索引 | 参数名称                              | 说明                                                         |
|----------|---------------------------------------|--------------------------------------------------------------|
| 0        |`primary_yaw`                     | 发射目标主偏航角                                             |
| 1        |`primary_pitch`                   | 发射目标主俯仰角(很快会被移除)                                             |
| 2        |`primary_force`                   | 发射扳机主位置                                             |
| 3-6        |`auxiliary_yaw_offsets`          | 发射目标副偏航角偏移量                                       |
| 7-10        |`auxiliary_force_offsets`        | 发射扳机副位置偏移量                                       |
| 11        |`sequence_offset`                | 从第几发开始发射，调试用                                     |
| 12-13        |`last_param_update_time`         | 上次参数更新时间, 用于同步, 单位ms 高位优先                   |
| 14        |`dart_state`                      | 发射架状态, 100 Boot, 101 Protect, 102 Remote, 103-106 Match Enter Wait Launch Reload, 255 Undefined |

与此同时，由于裁判系统串口连接到STM32上，需要将裁判系统的数据包转发到ROS2系统中，以便日志记录和其他功能的实现。

裁判系统上行链路数据包定义如下：
```mermaid
classDiagram
    class JudgeData{
        +uint8 dart_launch_opening_status, 发射架开启状态
        +uint8 game_progress, 比赛进程
        +uint8 dart_remaining_time, 剩余发射时间
        +uint16 latest_launch_cmd_time, 最新发射指令时间
        +uint16 stage_remain_time, 当前阶段剩余时间
        +bool judge_online, 裁判系统在线状态
    }
```

裁判系统数据包的传输方案与发射参数传输方案类似，但为了减少数据包的传输量，使用 `std_msgs/Int16MultiArray` 执行数据传输。数组总长度为 6，每个索引代表的参数定义如下：

| 数组索引 | 参数名称                              | 说明                                                         |
|----------|---------------------------------------|--------------------------------------------------------------|
| 0        | `dart_launch_opening_status`       | 发射架开启状态                                               |
| 1        | `game_progress`                    | 比赛进程                                                     |
| 2       | `dart_remaining_time`              | 剩余发射时间                                                 |
| 3       | `latest_launch_cmd_time`           | 最新发射指令时间                                             |
| 4       | `stage_remain_time`                | 当前阶段剩余时间                                             |
| 5       | `judge_online`                     | 裁判系统在线状态                                             |

### 通信拓扑
1. 生命周期管理、日志及看门狗节点
    - 管理其他节点的生命周期
    - 在节点出现看门狗问题的时候复位系统，将其他节点调为配置模式，将目标节点复位
    - 通过裁判系统通告和镖架状态通告记录日志
2. 界面、配置及节点：
    - 管理整个系统的Config，并处理打击Protocol的新建、存储、修改和选择
    - 显示系统状态、当前硬件Yaw, Pitch, Trigger角度，提供快速更改参数的途径
    - 显示一部分裁判系统消息
    - 系统设置
    - 提供协议选择、当前协议二维码获取、当前协议速览
    - 显示系统通告
    - 需要时显示相机画面
3. 自瞄相机节点
4. 参数相机节点

框图如下：

```mermaid
flowchart LR
    %% 上位机（控制层）
    subgraph 上位机 [上位机 - 控制层]
        U[GUI节点和日志节点]

    end

    %% 下位机（执行层）
    subgraph 下位机 [下位机 - 执行层]
        L[MCU通讯节点]
    end

    U -- std_msgs/Int32MultiArray<br>/dart_launcher/cmd --> L
    L -- std_msgs/Int16MultiArray<br>/dart_launcher/judge --> U
    L -- std_msgs/Int32MultiArray<br>/dart_launcher/status --> U

```

## 比赛需求功能
### 打击协议
1. 协议指的是一种一次性更新的参数集合，包含发射各轴角度、各槽位飞镖ID等。
2. 每次进入比赛状态时同步打击协议到MCU上，此后除非通过编辑器重新保存了协议，或者视觉自瞄识别目标后发生了自瞄Yaw调整，否则不再更新。
3. 比赛状态下MCU没有改变协议的权限，协议只能由上位机编辑器或者自瞄模块修改。
4. 比赛状态下自瞄模块产生的Yaw调整会被MCU接收并应用到发射参数中，但不会更新原有协议，而是在协议中增加一个Yaw偏移量，MCU会将其应用到发射参数中。该Yaw偏移量根据协议中的“是否自动保留键值参数”决定是否保留到协议中。远期计划根据比赛中的打击目标：固定目标则保留到协议中，移动目标则不保留。产生的一切变化都需要记录在日志中。
5. 遥控模式下，MCU会上报遥控器事件造成的参数变化，上位机存储到当前硬件状态参数中。除非勾选了“是否自动生成新协议”，否则不会更新原有协议或者产生任何新协议。若勾选了“是否自动生成新协议”，则会在当前协议的基础上生成一个新的协议，协议名称为"tmp_*当前时间戳*"，并将当前硬件状态参数存储到新协议中。这种临时协议会在当前启动中被默认选择。然而，这种临时协议不会被下次开机加载，但是可以下次开机前手动加载。
6. 在协议编辑器中，可以选择更新协议参数。协议参数可以在单独项或全选项中手动选择“引用”——可以引用已有协议，也可以从当前硬件参数中加载，也可以手动调整。

### 快速参数调整
使用Android手机APP编辑发射参数后生成json格式的二维码协议文件，由镖架后摄扫描后
