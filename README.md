# 救援车

本项目用于 RoboCon 场地小车的仿真、纯视觉定位和上层控制。当前环境为 ROS 2 Humble + Gazebo Classic 11。

核心约定：

- 所有对外接口统一使用 `/robot/*`。
- 定位必须是纯视觉，里程计只用于显示和对比参考。
- 底层节点直接发布/订阅统一话题，不再使用桥接节点转换话题。
- `robot_bringup` 是完整系统启动包，负责一次性启动仿真或实物定位，不再一层层套旧启动文件。

## 包结构

### `robocon25_sim`

仿真环境包。

```text
robocon25_sim/
├── worlds/robocon25.world
├── models/robot.model
├── models/RoboCon25_Field/
├── materials/textures/lines.png
├── meshes/robot.dae
├── src/cmd_vel_model_plugin.cpp
└── launch/gazebo_no_eol.launch.py
```

作用：

- 启动 Gazebo 场地和小车模型。
- 小车插件直接订阅 `/robot/cmd_vel`。
- 小车插件直接发布 `/robot/odom`。

### `robocon_localization`

视觉定位底层包。

```text
robocon_localization/
├── config/
│   ├── dist_table.txt
│   ├── field_lines.png
│   ├── field_bg.png
│   ├── white_lines.png
│   ├── red_lines.png
│   └── blue_lines.png
├── include/
├── launch/
│   ├── localization.launch
│   ├── calibrate_dist.launch
│   └── hsv_adjust.launch
└── src/
    ├── loc_sidelines.cpp
    ├── omni_mirror_sim.cpp
    ├── load_lines_map.cpp
    ├── calibrate_dist.cpp
    ├── hsv_adjust.cpp
    └── utility/
```

作用：

- `loc_sidelines`：主定位节点，订阅 `/robot/image_raw`，发布 `/robot/pose`。
- `localization.launch`：只启动定位节点本身，用于单独接入已有图像和里程计话题。
- `omni_mirror_sim`：仿真中把 `/sim_camera/image_raw` 转成 `/robot/image_raw`。
- `load_lines_map`：根据 `field_lines.png` 生成匹配模板。
- `calibrate_dist`：距离表标定。
- `hsv_adjust`：颜色阈值调试。

### `robot_bringup`

完整启动包。

```text
robot_bringup/
├── launch/
│   ├── sim.launch.py
│   └── real_localization.launch.py
└── robot_bringup/__init__.py
```

作用：

- `sim.launch.py`：直接启动 Gazebo、小车、全景模拟和视觉定位。
- `real_localization.launch.py`：实物只启动视觉定位。
- 不再包含 `interface_bridge`，因为底层已经直接使用统一话题。

### `robot_control`

Python 控制层。

```text
robot_control/src/
├── DriveControl.py
└── main.py
```

控制层只使用：

- 订阅 `/robot/pose`
- 发布 `/robot/cmd_vel`

## 统一话题

对外统一接口如下：

```text
/robot/image_raw   sensor_msgs/msg/Image
/robot/pose        geometry_msgs/msg/PoseStamped
/robot/odom        nav_msgs/msg/Odometry
/robot/cmd_vel     geometry_msgs/msg/Twist
```

说明：

- `/robot/image_raw`：定位图像。仿真由 `omni_mirror_sim` 发布，实物由真实相机发布或 remap。
- `/robot/pose`：视觉定位结果，单位米，坐标系 `map`。
- `/robot/odom`：里程计参考，只用于显示对比。
- `/robot/cmd_vel`：控制命令，仿真和实物控制层都发这个话题。

仿真内部仍有 `/sim_camera/image_raw`，这是 Gazebo 原始相机图像，只作为 `omni_mirror_sim` 的输入，不建议控制层使用。

## 编译

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

只编译当前相关包：

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build --packages-select robocon25_sim robocon_localization robot_bringup
source install/setup.bash
```

## 仿真启动

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
ros2 launch robot_bringup sim.launch.py start_point:=1
```

`start_point` 可取 `1/2/3/4`，默认 `1`。

如果 Gazebo 状态异常，先清理：

```bash
pkill -f gzserver
pkill -f gzclient
```

仿真启动后直接使用统一话题：

```bash
ros2 topic echo /robot/pose
ros2 topic echo /robot/odom
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

## 实物启动

实物侧要求相机发布到 `/robot/image_raw`，底盘里程计发布到 `/robot/odom`。

默认启动：

```bash
source /opt/ros/humble/setup.bash
source /home/cst/jyc/install/setup.bash
ros2 launch robot_bringup real_localization.launch.py
```

如果实物相机或里程计话题不同，可以启动时指定：

```bash
ros2 launch robot_bringup real_localization.launch.py image_topic:=/camera/image_raw odom_topic:=/wheel/odom
```

这只是让定位节点订阅指定输入；定位输出仍是 `/robot/pose`。

## 控制层使用

手动控制：

```bash
ros2 run rqt_robot_steering rqt_robot_steering
```

话题选择：

```text
/robot/cmd_vel
```

键盘控制示例：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/robot/cmd_vel
```

Python 控制示例：

```bash
cd /home/cst/jyc/src/robot_control/src
python3 main.py
```

常用接口：

```python
drive.Print_pose()
drive.Move(x, y, target_yaw_deg=None)
drive.Move_relative(dx, dy)
drive.Turn_to(target_yaw_deg)
drive.Turn_by(delta_yaw_deg)
drive.stop()
```

## 定位原理

当前主流程在：

```text
robocon_localization/src/loc_sidelines.cpp
```

流程：

1. 订阅 `/robot/image_raw`。
2. HSV/BGR 分割洋红、紫色、黑色、红色、蓝色区域。
3. 从图像中心按 `1deg` 射线扫描颜色边缘。
4. 用 `dist_table.txt` 把像素半径换成真实距离。
5. 将检测点投影到 `3m x 3m` 场地坐标。
6. 和 `white_lines.png`、`red_lines.png`、`blue_lines.png` 模板匹配。
7. 通过局部搜索更新视觉位姿。
8. 匹配质量不足时进入 `hold`，保持上一帧位姿。

注意：

- `white_lines.png` 是历史命名，实际表示洋红/紫色边缘模板。
- `/robot/odom` 不参与定位，只在窗口中对比显示。

## 窗口说明

### `result`

显示原始图像上的颜色检测和边缘 X 标记。

如果没有 X：

- 检查摄像头图像。
- 检查 HSV/BGR 阈值。
- 使用 `hsv_adjust.launch` 调色。

### `match_result`

显示场地图和投影点。

左上角会显示：

```text
match tracking 12/60 score=...
match hold 0/60 score=...
```

- `tracking`：当前帧匹配质量足够，允许更新位姿。
- `hold`：检测点没有有效匹配模板，本帧不更新位姿。

### `定位`

显示视觉定位位置和里程计参考。

## 生成模板图

修改颜色阈值、模板生成逻辑或 `field_lines.png` 后，需要重新生成模板：

```bash
cd /home/cst/jyc
source /opt/ros/humble/setup.bash
colcon build --packages-select robocon_localization
source install/setup.bash
ros2 run robocon_localization load_lines_map --ros-args \
  -p lines_file:=/home/cst/jyc/src/robocon_localization/config/field_lines.png
```

生成：

```text
robocon_localization/config/white_lines.png
robocon_localization/config/red_lines.png
robocon_localization/config/blue_lines.png
```

## 距离标定

距离表：

```text
robocon_localization/config/dist_table.txt
```

仿真标定：

```bash
ros2 launch robocon_localization calibrate_dist.launch red_ball_x:=0.9
```

实物标定时，应让真实相机发布 `/robot/image_raw`，然后运行标定节点或对应 launch。

建议距离表覆盖到至少 `2.2m`。场地中心到角落约 `2.12m`，距离表太短会导致远处点只能外推，定位容易偏。

## HSV 调试

```bash
ros2 launch robocon_localization hsv_adjust.launch
```

默认读取 `/robot/image_raw`。

## 常用调试顺序

1. 看话题：

```bash
ros2 topic list
ros2 topic hz /robot/image_raw
ros2 topic echo /robot/pose
ros2 topic echo /robot/odom
```

2. 看 `result` 是否有 X。

3. 看 `match_result` 中投影点是否落在模板轮廓上。

4. 看 `match tracking/hold` 状态。

5. 如果点云整体偏小或偏大，优先检查：

```text
robocon_localization/config/dist_table.txt
robocon_localization/src/loc_sidelines.cpp 中 projection_scale_correction
```

`projection_scale_correction` 调大，点投得更远；调小，点投得更近。

## 开发注意

- 不要再新增桥接节点来转换 `/cmd_vel`、`/odom`、`/image_raw`。
- 新增底层节点时，直接使用 `/robot/*` 统一话题。
- 控制层只依赖 `/robot/pose` 和 `/robot/cmd_vel`。
- 纯视觉定位时保持 `use_odom_for_tracking=false`。
- 改 Gazebo 插件或 `robot.model` 后需要重新编译 `robocon25_sim` 并重启 Gazebo。
- 改定位代码后重新编译 `robocon_localization`。
- 改模板生成逻辑后重新运行 `load_lines_map`。
