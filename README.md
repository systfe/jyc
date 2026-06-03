# 救援车 使用说明

本项目当前适配 ROS 2 Humble + Gazebo Classic 11，用于 3000mm x 3000mm 场地上的机器人仿真、全景相机模拟、场地颜色特征提取和定位显示。

## 当前系统结构

- `robocon25_sim`
  - 作用：仿真环境
  - Gazebo 世界、场地模型、机器人模型和 `/cmd_vel` 控制插件。
  - 场地尺寸为 `3m x 3m`。
  - 场地贴图来自 `robocon25_sim/materials/textures/lines.png`。
  - 机器人模型在 `robocon25_sim/models/robot.model`。

- `robocon_localization`
  - 相机图像转全景图：`omni_mirror_sim`
  - 距离标定：`calibrate_dist`
  - 主定位显示：`loc_sidelines`
  - 重定位测试：`relocalization`
  - 颜色模板和距离表在 `robocon_localization/config/`

- `robot_bringup`
  - 作用：仿真/实物的统一启动和接口桥接。
  - 仿真入口：启动 Gazebo、全景图模拟、定位节点，并把底层话题桥接到 `/robot/*`。
  - 实物入口：启动定位节点，默认从 `/robot/image_raw` 读取真实摄像头图像。

- `robot_control`
  - 作用：控制层 Python 包。
  - 当前只保留包骨架，后续只通过统一接口读取数据、发布控制命令，不直接依赖 Gazebo 或具体摄像头。

## 统一接口

控制层建议只使用下面这些通用话题：

```txt
/robot/image_raw   sensor_msgs/msg/Image
/robot/pose        geometry_msgs/msg/PoseStamped
/robot/odom        nav_msgs/msg/Odometry
/robot/cmd_vel     geometry_msgs/msg/Twist
```

含义：

- `/robot/image_raw`：统一摄像头图像。仿真时由 `/omni_camera/image_raw` 桥接而来；实物时由真实鱼眼摄像头驱动发布或 remap 过来。
- `/robot/pose`：视觉定位输出，单位为米，坐标系为 `map`。
- `/robot/odom`：底盘里程计。仿真时由 `/odom` 桥接而来；实物时由底盘驱动发布或 remap 过来。
- `/robot/cmd_vel`：控制层发布的速度命令。仿真时会桥接到 Gazebo 底盘插件的 `/cmd_vel`。

底层仍然可以保留自己的原始话题，例如 `/cmd_vel`、`/odom`、`/omni_camera/image_raw`。控制层不直接使用这些原始话题，避免仿真和实物切换时修改控制代码。

## 编译

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

只编译模块化相关包：

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build --packages-select robocon_localization robot_bringup
source install/setup.bash
```

## 启动完整定位仿真

建议每次启动前清理旧 Gazebo 进程，避免旧模型或旧插件残留：

```bash
pkill -f gzserver
pkill -f gzclient
```

启动：

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
ros2 launch robot_bringup sim.launch.py gui:=true verbose:=true
```

可以选择 1 到 4 号出发区：

```bash
ros2 launch robot_bringup sim.launch.py start_point:=1
```

旧入口仍然可用：

```bash
ros2 launch robocon_localization localization.launch gui:=true verbose:=true
```

启动后会出现：

- Gazebo 窗口：仿真场地和机器人
- `result`：相机图像上的颜色边缘检测结果，X 标记代表原始图像中检测到的特征边缘
- `match_result`：场地图背景 + 投影后的特征点
- `定位`：定位显示窗口，显示机器人在场地图上的位置和方向

## 机器人控制

使用 rqt 控制：

```bash
ros2 run rqt_robot_steering rqt_robot_steering
```

使用 `robot_bringup sim.launch.py` 启动时，话题选择 `/robot/cmd_vel`。桥接节点会转发到仿真底盘插件的 `/cmd_vel`。

如果使用旧的 `robocon_localization localization.launch` 启动，话题仍然选择 `/cmd_vel`。

当前机器人使用自定义 Gazebo 插件 `librobocon_cmd_vel_model_plugin.so`，直接订阅 `/cmd_vel` 并发布 `/odom`。如果机器人不转或不动，优先确认 `robot.model` 中插件名称不是旧的 `libgazebo_ros_planar_move.so`。

## 启动实物定位

实物上不启动 `robocon25_sim`，也不需要修改仿真的全景相机模拟节点。真实鱼眼摄像头需要由摄像头驱动发布图像，并 remap 到统一图像接口：

```txt
/robot/image_raw
```

然后启动定位：

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
ros2 launch robot_bringup real_localization.launch.py
```

如果摄像头驱动发布的是其他话题，例如 `/camera/image_raw`，可以启动时指定：

```bash
ros2 launch robot_bringup real_localization.launch.py image_topic:=/camera/image_raw
```

真实鱼眼摄像头必须重新标定 `robocon_localization/config/dist_table.txt`。仿真的距离表不能直接用于实物。

## 窗口含义

### `result`

显示全景图中的原始颜色检测结果。

- 紫色 X：洋红出发区边缘
- 黄色/红色相关 X：红色安全区边缘
- 浅蓝 X：蓝色安全区边缘

如果 `result` 中能看到 X，但 `match_result` 没有点，说明颜色识别成功，问题在距离表、投影比例或距离过滤。

### `match_result`

显示场地图背景和投影后的特征点：

- 绿色点：洋红出发区特征
- 红色点：红色安全区特征
- 蓝色点：蓝色安全区特征

### `定位`

显示机器人在场地图上的位置和朝向。当前仿真中优先使用 `/odom` 更新显示，因此只要机器人运动正常，窗口中的机器人也应该跟着运动。

## 距离标定

距离表文件：

```txt
robocon_localization/config/dist_table.txt
```

当前格式示例：

```txt
0.0 : 0
0.3 : 86
0.6 : 122
0.9 : 148
1.2 : 170
1.5 : 189
```

含义是：

```txt
实际距离m : 图像半径像素
```

启动距离标定：

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
ros2 launch robocon_localization calibrate_dist.launch
```

可以指定红球距离，例如：

```bash
ros2 launch robocon_localization calibrate_dist.launch red_ball_x:=0.9
```

标定建议：

1. 依次放置红球在 `0.3m / 0.6m / 0.9m / 1.2m / 1.5m` 等距离。
2. 在标定窗口中读取对应像素半径。
3. 更新 `dist_table.txt`。
4. 如果要识别场地角落，从中心到角落约 `2.12m`，建议继续标定到 `1.8m / 2.1m / 2.3m`，否则超过 1.5m 的部分只能靠外推，远距离投影会不准。

更新距离表后重新启动定位即可；如果修改了代码才需要重新编译。

## 颜色特征调试

当前场地图主要颜色：

- 红色安全区：`rgb(250,74,59)`
- 蓝色安全区：`rgb(92,169,221)`
- 洋红出发区：`rgb(255,25,255)`

对应检测阈值主要在：

```txt
robocon_localization/src/loc_sidelines.cpp
robocon_localization/src/utility/lines_map.cpp
robocon_localization/src/relocalization.cpp
```

当前逻辑：

- 绿色点只对应洋红出发区
- 红色点只对应红色区域
- 蓝色点只对应亮蓝安全区
- 深色紫/蓝边缘通过亮度和饱和度排除，避免被识别成蓝色安全区

修改颜色阈值后，需要重新生成模板图：



## 地图和模板文件

Gazebo 场地贴图：

```txt
robocon25_sim/materials/textures/lines.png
```

定位使用的缩放地图：

```txt
robocon_localization/config/field_lines.png
robocon_localization/config/field_bg.png
```

匹配模板：

```txt
robocon_localization/config/white_lines.png
robocon_localization/config/red_lines.png
robocon_localization/config/blue_lines.png
```

注意：`white_lines.png` 是历史命名，现在实际表示洋红出发区的绿色显示点模板，不再表示白线。

如果替换了 Gazebo 的 `lines.png`，需要同步更新 `field_lines.png`、`field_bg.png` 和三张模板图。

## 常见问题

### Gazebo 里模型或插件没有更新

清理旧进程并重新 source：

```bash
pkill -f gzserver
pkill -f gzclient
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
```

如果改了 `robocon25_sim` 的 C++ 插件，必须重新编译：

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build --packages-select robocon25_sim --allow-overriding robocon25_sim
source install/setup.bash
```

### `result` 有 X，`match_result` 没点

说明颜色识别成功，但投影失败。检查：

1. `dist_table.txt` 是否覆盖到足够远的距离。
2. `loc_sidelines.cpp` 中 `max_feature_distance` 是否太小。
3. `projection_scale_correction` 是否太大导致点投到地图外。
4. 是否重新 source 了最新编译结果。

### `match_result` 点距离明显偏小或偏大

调整：

```cpp
projection_scale_correction
```

位置：

```txt
robocon_localization/src/loc_sidelines.cpp
```

调小会让点更靠近机器人；调大会让点更远。

### 紫色边缘被识别成蓝色

蓝色安全区和深色紫/蓝边 Hue 接近，需要通过亮度 V 和饱和度 S 区分。

当前蓝色阈值：

```cpp
Scalar(96, 70, 170), Scalar(110, 200, 255)
```

如果紫色仍被识别成蓝色，继续提高 V 下限或降低 S 上限。

### 出发区绿色点识别不到

放宽洋红阈值：

```cpp
Scalar(130, 45, 60), Scalar(170, 255, 255)
```

如果误检变多，可以收紧 H 范围或提高 S/V 下限。



## 推荐调试顺序

1. 启动 `localization.launch`。
2. 看 Gazebo 中机器人能否正常运动。
3. 用 `/cmd_vel` 测试原地转和走圆。
4. 看 `result` 是否有 X 标记。
5. 如果 `result` 有 X，再看 `match_result` 是否有对应颜色点。
6. 如果 `match_result` 没点，检查距离表和投影参数。
7. 如果颜色混淆，调整 HSV 阈值并重新生成模板。
8. 如果定位窗口机器人不动，先检查统一接口 `/robot/pose` 和 `/robot/odom`：

```bash
ros2 topic echo /robot/pose
ros2 topic echo /robot/odom
```

旧入口没有启动 `robot_bringup` 桥接时，检查原始 `/odom`。
