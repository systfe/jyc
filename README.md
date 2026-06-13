# 救援车

改自 b 站阿杰老师的 [RoboCon25](https://github.com/6-robot/robocon25_proof)。

这是一个通过鱼眼相机进行纯视觉定位的小车工程。

# 项目结构

## `robocon25_sim`

仿真环境包，负责 Gazebo 场地、小车模型和仿真运动。

```text
robocon25_sim/
├── worlds/robocon25.world                 # Gazebo 场地世界文件
├── models/robot.model                     # 小车模型描述
├── models/RoboCon25_Field/                # 场地模型资源
├── materials/textures/lines.png           # 场地图案贴图
├── meshes/robot.dae                       # 小车网格模型
├── src/cmd_vel_model_plugin.cpp           # 将 /robot/cmd_vel 转成仿真模型运动的插件
└── launch/gazebo_no_eol.launch.py         # 启动 Gazebo 世界的底层 launch
```

## `robocon_localization`

视觉定位底层包，负责图像输入、颜色识别、距离标定、模板匹配和位姿发布。

```text
robocon_localization/
├── config/
│   ├── dist_table.txt                     # 像素半径到真实距离的标定表
│   ├── camera.yaml                   # RTSP 相机连接和发布参数
│   ├── color_reference.yaml               # 颜色识别参考 RGB
│   ├── field_lines.png                    # 场地彩色线条原图
│   ├── field_bg.png                       # 定位监视窗口背景图
│   ├── white_lines.png                    # 洋红/紫色边缘匹配模板，历史命名为 white
│   ├── red_lines.png                      # 红色安全区匹配模板
│   └── blue_lines.png                     # 蓝色安全区匹配模板
├── include/
│   ├── distance_lookup.h                  # 距离表读取接口
│   ├── lines_map.h                        # 模板图生成接口
│   ├── lines_matcher.h                    # 点云和模板匹配接口
│   └── ros2_compat.h                      # ROS1 风格日志到 ROS2 日志的兼容宏
├── launch/
│   ├── localization.launch                # 单独启动定位节点
│   ├── rtsp_camera.launch.py              # 单独启动 RTSP 相机发布节点
│   ├── calibrate_dist.launch              # 仿真距离标定
│   └── hsv_adjust.launch                  # HSV 阈值调试窗口
├── scripts/
│   └── rtsp_camera_publisher.py           # RTSP/HTTP 相机读取并发布 /robot/image_raw
└── src/
    ├── loc_sidelines.cpp                  # 主视觉定位节点
    ├── omni_mirror_sim.cpp                # 仿真全景相机转换节点
    ├── load_lines_map.cpp                 # 由 field_lines.png 生成匹配模板
    ├── calibrate_dist.cpp                 # 距离标定节点
    ├── hsv_adjust.cpp                     # 颜色阈值调试节点
    └── utility/                           # 距离表、模板图、匹配算法实现
```

## `robot_bringup`

统一启动包，负责把仿真、实车相机、实车定位按模块组合起来。

```text
robot_bringup/
├── launch/
│   ├── sim.launch.py                      # 启动 Gazebo、小车、仿真相机转换和定位
│   ├── real_camera.launch.py              # 启动实车相机模块，发布 /robot/image_raw
│   └── real_localization.launch.py        # 启动实车视觉定位模块
├── robot_bringup/__init__.py              # Python 包初始化文件
├── package.xml                            # ROS 2 包描述
└── setup.py                               # Python 包安装配置
```

## `robot_control`

控制层 Python 包，控制逻辑只通过统一话题和底层交互。

```text
robot_control/
├── src/
│   ├── Camera.py                          # 控制层读取 /robot/image_raw 的相机接口
│   ├── DriveControl.py                    # 控制层发送运动命令的接口
│   └── main.py                            # 控制逻辑入口或测试程序
├── package.xml                            # ROS 2 包描述
├── setup.py                               # Python 包安装配置
└── setup.cfg                              # Python 包安装路径配置
```

# 使用方法

## 环境

首次使用时配置 ROS 2 和工作区环境：

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo-11/setup.bash
source /home/cst/jyc/install/setup.bash
```

## 编译

编译必须在工作区根目录 `jyc` 下执行，不要在 `jyc/src` 下执行：

```bash
cd /home/cst/jyc
colcon build
source install/setup.bash
```

## 实车启动

先启动相机模块，让它发布 `/robot/image_raw`：

```bash
ros2 launch robot_bringup real_camera.launch.py
```

再另开一个终端启动视觉定位：

```bash
ros2 launch robot_bringup real_localization.launch.py
```

相机节点本身不会打开窗口；定位节点收到图像后会打开 `result`、`match_result`、`定位` 窗口。

- 参数 `image_topic`: 摄像头发布的话题，默认是 `/robot/image_raw`。
- 参数 `odom_topic`: 底盘里程计发布的话题，默认是 `/robot/odom`。
- 参数 `camera_params_file`: 摄像头参数文件，默认读取 `robocon_localization/config/camera.yaml`。
- 例:

```bash
ros2 launch robot_bringup real_localization.launch.py image_topic:=/camera/image_raw odom_topic:=/wheel/odom
```

表示启动时使用 `/camera/image_raw` 作为图像话题，使用 `/wheel/odom` 作为里程计话题。

## 仿真启动

```bash
ros2 launch robot_bringup sim.launch.py start_point:=1
```

- `start_point` 可取 `1/2/3/4`，分别代表四个出发点，默认是 `1`。

如果 Gazebo 状态异常，可以先清理旧进程：

```bash
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f calibrate_dist
pkill -9 -f omni_mirror_sim
```

## 移动控制

用 rqt 手动控制：

```bash
ros2 run rqt_robot_steering rqt_robot_steering
```

话题选择 `/robot/cmd_vel`。

用键盘手动控制：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/robot/cmd_vel
```

## 控制层

# 调试

## 摄像头配置

实车相机模块入口是：

```text
robot_bringup/launch/real_camera.launch.py
```

以后换不同鱼眼摄像头时，优先只改这个启动文件，让新的相机驱动仍然发布 `/robot/image_raw`。

当前 RTSP 相机参数在这里：

```text
robocon_localization/config/camera.yaml
```

常用项：

```yaml
rtsp_url: ""                 # 知道完整图像流地址时直接填写
camera_ip: "192.168.10.100"  # 相机 IP
rtsp_port: 554               # RTSP 端口
username: "admin"            # 账号
password: "1111111"          # 密码
image_topic: "/robot/image_raw"
output_width: 640
output_height: 640
```

需要临时换另一份相机参数文件时：

```bash
ros2 launch robot_bringup real_camera.launch.py \
  camera_params_file:=/path/to/camera.yaml
```

## 颜色参考

更改参考颜色，主要改这里：

```text
robocon_localization/config/color_reference.yaml
```

只需要填写看到的 RGB 参考值：

```yaml
red_rgb_reference: [250, 74, 59]
blue_rgb_reference: [92, 168, 222]
purple_rgb_reference: [1, 52, 108]
magenta_rgb_reference: [255, 25, 255]
black_rgb_reference: [0, 0, 0]
```

程序会自动把 RGB 转成 OpenCV HSV，并生成识别阈值。蓝色和紫色会额外自动生成 BGR 门限，用来减少互相误识别。修改后重启定位节点即可生效。

## 话题接口

对外统一接口如下：

```text
/robot/image_raw   sensor_msgs/msg/Image
/robot/pose        geometry_msgs/msg/PoseStamped
/robot/odom        nav_msgs/msg/Odometry
/robot/cmd_vel     geometry_msgs/msg/Twist
```

- `/robot/image_raw`：定位图像。仿真由 `omni_mirror_sim` 发布，实物由相机模块发布。
- `/robot/pose`：视觉定位结果，单位米，坐标系 `map`。
- `/robot/odom`：里程计参考，只用于显示对比，不参与纯视觉定位。
- `/robot/cmd_vel`：控制命令，仿真和实物控制层都发这个话题。

仿真内部仍有 `/sim_camera/image_raw`，这是 Gazebo 原始相机图像，只作为 `omni_mirror_sim` 的输入，不建议控制层使用。

## 窗口说明

### `result`

显示原始图像上的颜色检测和边缘 X 标记。

如果没有 X：

- 检查摄像头是否有图像。
- 检查 `color_reference.yaml` 中的参考 RGB。
- 使用 `hsv_adjust.launch` 辅助观察颜色。

### `match_result`

显示场地图和投影点。左上角会显示匹配状态：

```text
match tracking 12/60 score=...
match hold 0/60 score=...
```

- `tracking`：当前帧匹配质量足够，允许更新位姿。
- `hold`：检测点没有有效匹配模板，本帧不更新位姿。

### `定位`

显示视觉定位位置和里程计参考。

## HSV 调试

```bash
ros2 launch robocon_localization hsv_adjust.launch
```

默认读取 `/robot/image_raw`。

## 生成模板图

运行：

```bash
ros2 run robocon_localization load_lines_map
```

会在当前目录生成 `red_lines.png`、`blue_lines.png`、`white_lines.png` 三张图片。生成后放到：

```text
robocon_localization/config/
```

修改 `field_lines.png` 或模板生成逻辑后，需要重新生成模板。只修改 `color_reference.yaml` 不需要重新生成模板。

## 距离标定

距离表文件：

```text
robocon_localization/config/dist_table.txt
```

仿真标定：

```bash
ros2 launch robocon_localization calibrate_dist.launch red_ball_x:=0.1
```

启动后，小车前方 `red_ball_x` 米处会有一个红色小球，终端会输出对应像素距离。把像素距离和实际距离写入 `dist_table.txt`。

实车标定：

```bash
ros2 launch robot_bringup real_camera.launch.py
```

另开一个终端：

```bash
ros2 run robocon_localization calibrate_dist --ros-args \
  -p image_topic:=/robot/image_raw
```

在车前方 `0.1m/0.2m/...` 的位置放红色标记，记录终端输出的像素距离，并写入 `dist_table.txt`。

建议距离表至少覆盖到 `1.5m`；如果需要覆盖场地中心到角落，建议覆盖到 `2.2m` 左右。实车标定时如果识别到红色安全区，可以先遮住安全区。

# 原理
 
当前主流程在：

```text
robocon_localization/src/loc_sidelines.cpp
```

流程：

1. 订阅 `/robot/image_raw`。
2. 根据 `color_reference.yaml` 自动生成颜色阈值，分割洋红、紫色、黑色、红色、蓝色区域。
3. 从图像中心按 `1deg` 射线扫描颜色边缘。
4. 用 `dist_table.txt` 把像素半径换成真实距离。
5. 将检测点投影到 `3m x 3m` 场地坐标。
6. 和 `white_lines.png`、`red_lines.png`、`blue_lines.png` 模板匹配。
7. 通过局部搜索更新视觉位姿。
8. 匹配质量不足时进入 `hold`，保持上一帧位姿。

注意：

- `white_lines.png` 是历史命名，实际表示洋红/紫色边缘模板。
- `/robot/odom` 不参与定位，只在窗口中对比显示。

# 常见问题

## 摄像头延迟很大

如果日志中 `frame_age` 只有几十毫秒，但肉眼看仍有接近 1 秒延迟，说明程序内部没有明显排队，延迟主要来自摄像头编码或 RTSP 码流。优先在摄像头网页后台调整：

```text
子码流：开启
分辨率：640x640 / 720x720 / 640x480
帧率：15fps 或 20fps
编码：H.264
B 帧：关闭
I 帧间隔/GOP：1s 或更小
码率控制：CBR
码率：1000~2000 kbps
```

也可以在 `camera.yaml` 中调整 `http_paths` 和 `rtsp_paths` 的顺序。

## 定位窗口没有出现

先确认图像话题是否有数据：

```bash
ros2 topic hz /robot/image_raw
```

如果没有数据，先检查相机模块：

```bash
ros2 launch robot_bringup real_camera.launch.py
```

如果有图像但没有特征点，检查 `color_reference.yaml` 中的 RGB 参考值。
