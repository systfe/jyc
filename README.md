# 救援车定位与控制项目说明

本工作区位于：

```bash
/home/cst/jyc
```

源码目录位于：

```bash
/home/cst/jyc/src
```

项目当前适配 ROS 2 Humble + Gazebo Classic 11，目标是在 `3m x 3m` 场地中完成仿真、全景相机模拟、纯视觉定位、统一接口桥接和上层 Python 控制。

定位原则：小车定位必须使用纯视觉。里程计只允许作为显示、调试和对比参考，不能用于修正视觉定位。代码中 `use_odom_for_tracking` 默认应保持 `false`。

## 一、包的作用

### `robocon25_sim`

仿真环境包。

主要内容：

```text
robocon25_sim/
├── worlds/robocon25.world              Gazebo 世界
├── models/robot.model                  小车模型
├── models/RoboCon25_Field/             场地模型
├── materials/textures/lines.png        Gazebo 场地贴图
├── meshes/robot.dae                    小车外观模型
├── src/cmd_vel_model_plugin.cpp        Gazebo 速度控制插件
└── launch/                             Gazebo 启动文件
```

功能：

- 启动 `3m x 3m` 场地。
- 生成机器人实体。
- 订阅 `/cmd_vel` 控制机器人运动。
- 发布 `/odom` 作为仿真里程计参考。

### `robocon_localization`

视觉定位核心包。

主要内容：

```text
robocon_localization/
├── config/
│   ├── dist_table.txt                  像素半径到实际距离的标定表
│   ├── field_lines.png                 定位使用的彩色线图
│   ├── field_bg.png                    定位窗口背景图
│   ├── white_lines.png                 洋红/紫色边缘模板，名字是历史遗留
│   ├── red_lines.png                   红色安全区模板
│   └── blue_lines.png                  蓝色安全区模板
├── include/
│   ├── distance_lookup.h               距离表查询接口
│   ├── lines_map.h                     模板图生成接口
│   ├── lines_matcher.h                 局部匹配接口
│   └── ros2_compat.h                   ROS 日志兼容封装
├── src/
│   ├── loc_sidelines.cpp               当前主定位节点
│   ├── omni_mirror_sim.cpp             仿真相机转全景图节点
│   ├── load_lines_map.cpp              生成匹配模板图
│   ├── calibrate_dist.cpp              距离标定节点
│   ├── hsv_adjust.cpp                  HSV 调试节点
│   ├── relocalization.cpp              重定位辅助节点
│   ├── localization.cpp                早期定位节点
│   └── utility/                        距离、地图、匹配工具实现
└── launch/
    ├── localization.launch             旧版完整仿真入口
    ├── loc_sidelines.launch            定位节点测试入口
    ├── calibrate_dist.launch           距离标定入口
    ├── hsv_adjust.launch               HSV 调试入口
    └── test_*.launch                   测试入口
```

当前主节点是 `loc_sidelines`。它订阅全景图像，提取场地颜色特征，投影到场地坐标，和模板图匹配，最后发布 `/robot/pose`。

### `robot_bringup`

统一启动和接口桥接包。

主要内容：

```text
robot_bringup/
├── launch/
│   ├── sim.launch.py                   仿真统一启动入口
│   └── real_localization.launch.py     实物定位启动入口
└── robot_bringup/interface_bridge.py   仿真原始话题和 /robot/* 话题桥接
```

功能：

- 仿真时启动 Gazebo、机器人、全景相机模拟、视觉定位和接口桥接。
- 实物时只启动视觉定位节点。
- 给控制层提供统一话题，避免控制代码区分仿真和实物。

### `robot_control`

上层 Python 控制代码目录。

当前内容：

```text
robot_control/
└── src/
    ├── DriveControl.py                 PurePursuit 控制器
    └── main.py                         示例控制程序
```

当前 `robot_control` 目录只是普通 Python 脚本目录，不是完整 ROS 2 package。它通过统一接口读取 `/robot/pose` 并发布 `/robot/cmd_vel`。

## 二、统一接口

上层控制层建议只使用这些话题：

```text
/robot/image_raw   sensor_msgs/msg/Image
/robot/pose        geometry_msgs/msg/PoseStamped
/robot/odom        nav_msgs/msg/Odometry
/robot/cmd_vel     geometry_msgs/msg/Twist
```

含义：

- `/robot/image_raw`：统一图像入口。仿真时由 `/omni_camera/image_raw` 桥接而来；实物时由真实摄像头驱动发布或 remap 而来。
- `/robot/pose`：视觉定位输出，单位为米，坐标系为 `map`。
- `/robot/odom`：里程计参考。只用于显示和对比，不用于视觉定位修正。
- `/robot/cmd_vel`：控制层速度命令。仿真时桥接到 Gazebo 的 `/cmd_vel`。

仿真内部原始话题：

```text
/sim_camera/image_raw
/omni_camera/image_raw
/cmd_vel
/odom
```

控制代码不要直接依赖这些原始话题。

## 三、编译

完整编译：

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

只编译定位和启动相关包：

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build --packages-select robocon_localization robot_bringup
source install/setup.bash
```

如果修改了 Gazebo 插件：

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build --packages-select robocon25_sim
source install/setup.bash
```

每次新终端运行前都要：

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
```

## 四、仿真启动

推荐入口是 `robot_bringup`：

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
ros2 launch robot_bringup sim.launch.py start_point:=1
```

`start_point` 可取 `1/2/3/4`，默认是 `1`。

启动前如果 Gazebo 状态异常，可以先清理旧进程：

```bash
pkill -f gzserver
pkill -f gzclient
```

仿真启动后会包含：

- Gazebo 场地和小车。
- `omni_mirror_sim`：把仿真普通相机图像转换成模拟全景图。
- `loc_sidelines`：纯视觉定位。
- `relocalization`：重定位辅助。
- `interface_bridge`：把仿真原始话题桥接成 `/robot/*`。

旧入口也可以使用：

```bash
ros2 launch robocon_localization localization.launch start_point:=1
```

区别是旧入口不启动 `interface_bridge`，控制话题需要直接使用 `/cmd_vel`。

## 五、实物启动

实物不启动 Gazebo，也不启动 `omni_mirror_sim`。真实鱼眼相机需要由摄像头驱动发布图像。

如果摄像头已经发布到 `/robot/image_raw`：

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
ros2 launch robot_bringup real_localization.launch.py
```

如果摄像头发布到其他话题，例如 `/camera/image_raw`：

```bash
ros2 launch robot_bringup real_localization.launch.py image_topic:=/camera/image_raw
```

如果底盘里程计发布到其他话题，例如 `/wheel/odom`：

```bash
ros2 launch robot_bringup real_localization.launch.py odom_topic:=/wheel/odom
```

注意：

- 实物鱼眼相机必须重新标定 `dist_table.txt`。
- 实物光照会影响 HSV 阈值，第一次上车建议先看 `result` 窗口和 HSV 调试工具。
- `use_odom_for_tracking` 保持 `false`，定位仍然纯视觉。

## 六、控制层使用

`robot_control/src/DriveControl.py` 提供了一个简单控制器 `PurePursuit`。

它使用：

```text
订阅：/robot/pose
发布：/robot/cmd_vel
```

运行示例：

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
cd /home/cst/jyc/src/robot_control/src
python3 main.py
```

常用函数：

```python
drive.Print_pose()
drive.Move(x, y, target_yaw_deg=None)
drive.Move_relative(dx, dy)
drive.Turn_to(target_yaw_deg)
drive.Turn_by(delta_yaw_deg)
drive.stop()
```

坐标约定：

- `/robot/pose` 的 `x/y` 单位是米。
- `yaw` 外部显示和调用时一般用度，内部计算用弧度。
- `Move(x, y)` 是地图绝对坐标。
- `Move_relative(dx, dy)` 默认是车体坐标：`dx > 0` 向前，`dy > 0` 向左。

用 rqt 手动控制：

```bash
ros2 run rqt_robot_steering rqt_robot_steering
```

仿真统一入口下，话题选择：

```text
/robot/cmd_vel
```

旧 `localization.launch` 入口下，话题选择：

```text
/cmd_vel
```

## 七、定位原理

当前主定位流程在 `robocon_localization/src/loc_sidelines.cpp` 中。

### 1. 图像输入

仿真中：

```text
/sim_camera/image_raw -> omni_mirror_sim -> /omni_camera/image_raw
```

`loc_sidelines` 默认订阅 `/omni_camera/image_raw`。

实物中：

```text
真实鱼眼相机 -> /robot/image_raw -> loc_sidelines
```

### 2. 颜色分割

图像转换到 HSV 后提取这些区域：

- 洋红：出发区。
- 紫色：安全区边框。
- 黑色：安全区中间分割线。
- 红色：红色安全区。
- 蓝色：蓝色安全区。

蓝色和紫色 Hue 很接近，因此代码中使用 HSV 候选 + BGR 通道门槛做蓝紫互斥，避免紫色边框被识别成蓝色安全区。

主要位置：

```text
robocon_localization/src/loc_sidelines.cpp
robocon_localization/src/utility/lines_map.cpp
```

### 3. 射线扫描和距离表

以图像中心为原点，每 `1deg` 扫描一条射线。

射线遇到颜色边缘后，得到：

```text
角度 angle
像素半径 length
```

再用 `dist_table.txt` 把像素半径转换成真实距离：

```text
实际距离m : 图像半径像素
```

示例：

```text
0.0 : 0
0.1 : 50
0.2 : 70
```

最后把极坐标点投影到 `3m x 3m` 场地坐标中。

### 4. 模板匹配

定位使用三类模板：

```text
white_lines.png  洋红/紫色边缘模板，名字是历史遗留
red_lines.png    红色安全区模板
blue_lines.png   蓝色安全区模板
```

模板由 `load_lines_map` 从 `field_lines.png` 生成。

匹配时会把当前帧投影点旋转和平移到候选位姿，再到模板图上取分数。分数越高，说明当前视觉点云越接近场地图中的真实线条。

### 5. 连续跟踪

定位初始化成功后，每帧围绕上一帧位姿做局部搜索：

- 小范围搜索 yaw。
- 小范围搜索 x/y。
- 用洋红、紫色、黑线、红区、蓝区分别细化。
- 如果一帧内位置或 yaw 跳变过大，就回退到上一帧。

这是纯视觉连续性约束，不是里程计融合。

### 6. 匹配质量闸门

现在定位器会统计：

```text
真正落到模板附近的点数 / 当前检测到的特征点数
```

如果检测点很多，但没有点真正匹配模板，状态会变成 `hold`，本帧不会更新位姿。这样可以避免靠近安全区时“点云跟着小车走”。

`match_result` 左上角会显示：

```text
match tracking 12/60 score=...
match hold 0/60 score=...
```

含义：

- `tracking`：当前帧匹配质量足够，允许更新定位。
- `hold`：当前帧匹配质量不足，保持上一帧定位。

## 八、窗口含义

### `result`

显示原始图像上的颜色边缘检测结果。

如果这里没有 X 标记，优先检查：

- 摄像头图像是否正常。
- HSV 阈值是否适合当前光照。
- 图像中心是否被小车自身遮挡。

### `match_result`

显示场地图背景和投影后的特征点。

常见颜色：

- 绿色点：洋红/紫色边缘综合点。
- 红色点：红色安全区点。
- 蓝色点：蓝色安全区点。

左上角 `match tracking/hold` 是最重要的调试信息。

### `定位`

显示视觉定位出的机器人位置和朝向，同时显示里程计参考。

注意：里程计只用于参考显示，不参与定位。

## 九、生成模板图

修改以下内容后需要重新生成模板：

- `field_lines.png`
- 颜色阈值
- 模板边缘半径
- 蓝紫分离逻辑

命令：

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build --packages-select robocon_localization
source install/setup.bash
ros2 run robocon_localization load_lines_map --ros-args \
  -p lines_file:=/home/cst/jyc/src/robocon_localization/config/field_lines.png
```

生成结果：

```text
robocon_localization/config/white_lines.png
robocon_localization/config/red_lines.png
robocon_localization/config/blue_lines.png
```

## 十、距离标定

距离表：

```text
robocon_localization/config/dist_table.txt
```

启动标定：

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
ros2 launch robocon_localization calibrate_dist.launch
```

指定红球实际距离：

```bash
ros2 launch robocon_localization calibrate_dist.launch red_ball_x:=0.9
```

建议标定到至少 `2.2m`，因为场地中心到角落约 `2.12m`。如果距离表只覆盖近距离，远处点只能靠外推，投影容易整体偏小或偏大。

修改距离表后不需要重新生成模板，只需要重启定位节点。

## 十一、关键调试参数

### `projection_scale_correction`

位置：

```text
robocon_localization/src/loc_sidelines.cpp
```

作用：修正投影距离比例。

现象：

- 点云整体比真实距离小：适当调大。
- 点云整体比真实距离大：适当调小。

修改后需要重新编译。

### `dist_table.txt`

作用：像素半径到实际距离的映射。

现象：

- 近处准、远处不准：距离表远距离点不足。
- 所有距离都偏：距离表或 `projection_scale_correction` 有问题。

### 蓝紫阈值

位置：

```text
createSeparatedBluePurpleMasks()
```

文件：

```text
robocon_localization/src/loc_sidelines.cpp
robocon_localization/src/utility/lines_map.cpp
```

现象：

- 紫色边框被识别成蓝色：收紧蓝色亮度/饱和度门槛，或放宽紫色门槛。
- 蓝色安全区识别不到：放宽蓝色门槛。

改完后需要重新生成模板图。

### 匹配质量闸门

位置：

```text
isMatchQualityAcceptable()
countMatchedPointsOnMap()
```

现象：

- `match hold 0/很多点`：检测点多，但没有真正落到模板上，检查距离表和投影比例。
- 一直 `hold`：匹配阈值可能太严，或模板/距离表不对。
- 明显不匹配还 `tracking`：阈值可能太松。

## 十二、推荐调试流程

### 1. 先确认话题

```bash
ros2 topic list
ros2 topic echo /robot/pose
ros2 topic echo /robot/odom
ros2 topic hz /robot/image_raw
```

仿真中也可以检查原始话题：

```bash
ros2 topic echo /odom
ros2 topic hz /omni_camera/image_raw
```

### 2. 看 `result`

如果 `result` 没有 X：

- 检查摄像头图像。
- 检查 HSV 阈值。
- 使用 `hsv_adjust.launch` 辅助调色。

```bash
ros2 launch robocon_localization hsv_adjust.launch
```

### 3. 看 `match_result`

如果 `result` 有 X，但 `match_result` 点位置不对：

- 检查 `dist_table.txt`。
- 调整 `projection_scale_correction`。
- 确认 `field_lines.png` 和 Gazebo 贴图一致。

如果左上角一直 `hold`：

- 当前检测点没有真正匹配模板。
- 先不要放松阈值，优先检查投影距离和模板图。

### 4. 看 `定位`

如果视觉位置准，但控制不动：

- 检查 `/robot/cmd_vel` 是否有速度。
- 检查仿真入口是否启动了 `interface_bridge`。
- 旧入口要发 `/cmd_vel`，新入口要发 `/robot/cmd_vel`。

### 5. 安全区附近错位

如果靠近安全区时，单个正方形分区被竖着匹配：

- 确认黑色分割线 `black` 点是否稳定出现。
- 确认红/蓝安全区点数足够。
- 看 `match_result` 是否是 `hold`。如果是不匹配还被更新，说明质量闸门太松；如果一直 `hold`，说明模板或投影需要调。

## 十三、常见问题

### 修改代码后运行没有变化

重新编译并 source：

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build --packages-select robocon_localization
source install/setup.bash
```

如果 Gazebo 插件也改了，编译 `robocon25_sim`。

### Gazebo 状态异常

```bash
pkill -f gzserver
pkill -f gzclient
```

然后重新启动。

### `result` 有点，`match_result` 没点

优先检查：

1. `dist_table.txt` 是否覆盖足够远。
2. `projection_scale_correction` 是否合适。
3. `max_feature_distance` 是否太小。
4. 是否重新 source 最新编译结果。

### 点云整体偏小或偏大

先看距离表，再调：

```text
projection_scale_correction
```

调大：点投得更远。

调小：点投得更近。

### 颜色混淆

改 HSV/BGR 阈值后要做两件事：

```bash
colcon build --packages-select robocon_localization
ros2 run robocon_localization load_lines_map --ros-args \
  -p lines_file:=/home/cst/jyc/src/robocon_localization/config/field_lines.png
```

否则运行时识别和模板图可能不一致。

## 十四、开发注意

- 控制层只使用 `/robot/*`，不要直接依赖仿真原始话题。
- 纯视觉定位时 `use_odom_for_tracking=false`。
- `white_lines.png` 是历史命名，不代表白线。
- 改 `loc_sidelines.cpp` 后只需编译 `robocon_localization`。
- 改 Gazebo 插件后需要编译 `robocon25_sim` 并重启 Gazebo。
- 改模板生成逻辑后必须重新运行 `load_lines_map`。
