# MoSA (Motion-based Sensing Architecture) 预览版——正在进行全面测试

<div align="center" style="display:flex; justify-content:center; align-items:center; gap:16px;">
  <img src="img/Horizon.png" alt="Horizon Team" style="height:280px; width:auto;" />
  <a href="https://www.youtube.com/watch?v=SYaig2eHV5I" target="_blank">
    <img src="img/cover.bmp" alt="video" style="height:280px; width:auto;" />
  </a>
</div>

---

MoSA 是面向实时机器人系统的高性能运动感知框架，专门为 ROBOMASTER 比赛激光雷达应用场景设计。它受到香港大学 MARS Lab 提出的 M-Detector 启发，并在其基础上完成深度工程化与系统级优化，重点关注：

- 极低延迟实时监测
- CPU-only 高吞吐，强并行友好
- 稳定输出 动态点云 + AABB
- 无学习依赖、无先验假设

MoSA 可在无语义、无训练数据的条件下稳定检测动态目标，并作为下游感知、规划、规避、检测等模块基础输入。

---


## 1. 参考

- 原作者单位：The University of Hong Kong, MARS Lab
- 论文：[Moving Event Detection from LiDAR Point Streams（Nature Communications, 2024）](https://www.nature.com/articles/s41467-023-44554-8)

MoSA 保留并致谢原作者的算法思想与论文贡献，在实现层面对性能、并行模型与工程接口进行了系统性重构。


---

## 2. 效果演示




---

## 3. 环境与依赖

- Ubuntu 22.04
- ROS2 Humble
- PCL >= 1.8
- Eigen >= 3.3.4
- oneTBB (`libtbb-dev`)

安装示例：
```bash
sudo apt install libpcl-dev libeigen3-dev libtbb-dev
```

---

## 4. 工作空间结构

推荐目录结构：
```
ros2_ws/
  src/
    mosa/
```

> [!IMPORTANT]
> 若使用 Livox 设备，需要确保 `livox_interfaces` 已存在并被 source（建议由 Livox ROS2 驱动工作空间单独提供）。


---


## 5. 获取与编译

### 5.1 获取代码
```bash
git clone https://github.com/BreCaspian/MoSA.git
```

### 5.2 编译
```bash
mkdir -p ~/ros2_ws/src
cp -r MoSA ~/ros2_ws/src/mosa
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---


## 6. 运行

### 6.1 在线运行（示例）
```bash
ros2 launch mosa mid70.launch.py
```

其他内置设备配置：
```bash
ros2 launch mosa mid360.launch.py
ros2 launch mosa horizon.launch.py
ros2 launch mosa avia.launch.py
```

### 6.2 RViz 可视化
默认使用 `rviz/mosa.rviz`。关闭 RViz：
```bash
ros2 launch mosa mid70.launch.py rviz:=false
```

> [!TIP]
> RViz 仅影响显示，不影响算法输出与性能。


---


## 7. 输入与输出

### 7.1 输入

**点云输入：**
- Livox CustomMsg（默认）：
  - 话题：`/livox/lidar`
  - 参数：`dyn_obj/use_livox_custom: true`
- 标准 PointCloud2：
  - 话题：`dyn_obj/points_topic`
  - 参数：`dyn_obj/use_livox_custom: false`

**里程计输入：**
- 话题：`dyn_obj/odom_topic`
- 默认：`/aft_mapped_to_init`

> MoSA 仅依赖短时连续位姿稳定性，不要求全局闭环或高精度地图。

### 7.2 输出

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/mosa/point_out` | PointCloud2 | 点级动态结果（最低延迟） |
| `/mosa/frame_out` | PointCloud2 | 帧级稳定动态结果 |
| `/mosa/std_points` | PointCloud2 | 静态点云 |
| `/mosa/aabb` | MarkerArray | 动态目标 AABB |

可选文件输出：
- `dyn_obj/out_file`
- `dyn_obj/out_file_origin`
- `dyn_obj/cluster_out_file`
- `dyn_obj/time_file`
- `dyn_obj/time_breakdown_file`

---

## 8. 静止模式（无里程计依赖）

当 LiDAR 完全静止且坐标系稳定时，可关闭里程计：
```yaml
dyn_obj/use_odom: false
dyn_obj/static_pos: [0.0, 0.0, 0.0]
dyn_obj/static_quat: [1.0, 0.0, 0.0, 0.0]
```

> [!CAUTION]
> 仅适用于传感器完全静止场景。传感器运动会引入明显误检。


---

## 9. 参数体系与设备差异

### 9.1 通用参数模板

`config/base/base.yaml` 包含与设备无关的核心算法参数，建议作为全局基线。

关键参数分类：
- 输入：`use_livox_custom`、`points_topic`、`use_odom`、`odom_topic`
- 时序/缓存：`frame_dur`、`buffer_delay`、`points_num_perframe`
- 动态稳定化：`dyn_points_stable_en`、`dyn_points_accu_limit`、`dyn_points_voxel`
- 追踪与输出：`track_enable`、`track_min_hits`、`track_max_age`、`track_gate`、`aabb_alpha`
- 性能：`cluster_async`、`cluster_queue_size`

### 9.2 设备差异参数

| 设备 | 需要重点调整的参数 |
| --- | --- |
| MID70 | `fov_*`, `points_num_perframe`, `frame_dur`, `max_pixel_points` |
| MID360 | 同 MID70，点密度更高 |
| Horizon | 横向 FOV 大，需调整 `fov_*` |
| 通用 PC2 | `use_livox_custom=false`, `points_topic`, `frame_dur` |

通用 PC2 示例：`config/pc2/pc2.yaml`

推荐加载方式：
```bash
ros2 run mosa dynfilter --ros-args \
  --params-file config/base/base.yaml \
  --params-file config/mid70/mid70.yaml
```


---



## 10. 下游任务接口

### 10.1 坐标系
- 所有输出统一 `frame_id = dyn_obj/frame_id`
- 使用里程计时应为稳定全局系（如 `map` / `camera_init`）

### 10.2 时间戳
- 使用扫描结束时刻 `scan_end_time`
- 点云与 AABB 时间戳一致

### 10.3 AABB 语义约定
- 轴对齐包围盒（AABB）
- 中心：`pose.pose.position`
- 尺寸：存储于 `pose.covariance`
- Marker：绿色线框立方体
- `id`：跟踪 ID
- 不包含语义类别


---


## 11. 离线评估（可选）
```bash
ros2 launch mosa cal_recall.launch.py \
  dataset:=0 dataset_folder:=/path/to/dataset \
  start_se:=0 end_se:=0 start_param:=0 end_param:=0 is_origin:=false
```

---


## 12. 常见问题

### RViz 没有动态点
- 检查 `/mosa/point_out` 是否有频率输出
- 若 `/livox/lidar` 同时存在 CustomMsg 和 PointCloud2，请确认订阅类型一致

> [!TIP]
> 可用 `ros2 topic info --verbose /livox/lidar` 确认实际发布类型。

### /livox/lidar 无法 echo
- 话题类型冲突时请用 `ros2 topic info --verbose /livox/lidar` 查看真实类型

### 只看到静态点或动态点很少
- 确认输入点云频率与 `dyn_obj/frame_dur` 匹配
- 适当调大 `dyn_obj/dyn_points_accu_limit` 与 `dyn_obj/dyn_points_voxel`
- 检查是否启用 `dyn_obj/use_odom`，并确保里程计可用

### AABB 不显示或数量为 0
- 确认 `/mosa/aabb` 有发布（MarkerArray）
- 检查 `dyn_obj/track_enable` 是否开启
- 若启用 `cluster_async=true`，`/mosa/frame_out` 不会输出属于正常现象


---


## 13. 许可证

本项目采用 **GPL-3.0** 许可证发布，任何分发与衍生作品必须遵守 GPL-3.0 的条款。

---


## 14. 致谢

原作者：
- Huajie Wu — wu2020@connect.hku.hk
- Yihang Li — yhangli@connect.hku.hk

MoSA 在工程实现层面对原方法进行了优化与扩展，完整保留算法来源与作者署名。

联系人：
Yuzhuo Yao — yaoyuzhuo6@gmail.com
