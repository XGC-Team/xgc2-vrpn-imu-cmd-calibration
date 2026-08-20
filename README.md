# vrpn-imu-cmd-calibration

面向平面 UGV 的一键短时工程标定：自动发布安全约束下的 `cmd_vel`、录制一份
约 1–3 分钟的 ROS1 bag，并从同一份包辨识当前开机/当前动捕场景所需的参数
量级。目标是让状态估计和控制链可用，不是制作传感器 datasheet。

物理语义始终只有车上三帧：

- `C`：`cmd_vel` FLU 控制系，`+x` 是车头。
- `I`：原始 IMU 器件系。
- `M`：VRPN marker 刚体系。

动捕提供唯一的外部世界。工具不再发明任务世界或第四套坐标系。

## 自动覆盖范围

| 顺序 | 输出 | 方法 |
| ---: | --- | --- |
| 1 | `T_CM.yaw` | 两个相差 60° 的航向，各做前进/倒车重复段；使用 VRPN 空间位移 |
| 2 | gyro-z bias/sign/scale | 首尾静止 + 正反两档角速度，积分 gyro 对 VRPN pose `Δyaw` |
| 3 | `T_CI.yaw` | 正反直线起步的去偏 `Δv` 稳健方向 |
| 4 | VRPN `R` | 首尾静止块的去趋势 MAD，选择附近保守量级 |
| 5 | IMU `Q` | 样本噪声按实测 `dt` 换成连续时间 density |
| 6 | bias random walk 工程值 | 首尾静止 bias 变化，在小型对数候选集中向上选值 |

顺序 4–6 明确标记为 `short-run engineering estimate`。没有 Allan 长测、温漂
扫描或全工况曲线。VRPN twist/accel 不进入辨识。

## 8×8 米默认激励

第一个稳定 VRPN 位置就是场地中心，因此动捕坐标不必是 `(0, 0)`。默认软件
围栏为以该点为中心的 8×8 米矩形，四周留 0.75 米；名义轨迹只离起点约
1.2 米：

1. 静止 20 秒。
2. 航向 A 前进/倒车往返两次。
3. 顺逆时针分别以 0.22、0.40 rad/s 转 45° 并回正。
4. 转 60° 到航向 B，再做两次前进/倒车往返。
5. 回到初始航向，静止 20 秒。

配置见 [`config/default.yaml`](config/default.yaml)，安全边界见
[`docs/safety.md`](docs/safety.md)。软件围栏不是避障；空场、操作员和物理急停
仍是执行前提。

## 构建与试运行

把仓库放入 catkin workspace 后：

```bash
catkin build vrpn_imu_cmd_calibration
source devel/setup.bash

# 只打印轨迹，不发非零速度
rosrun vrpn_imu_cmd_calibration run_calibration.py
```

确认底盘控制话题、动捕、IMU、8×8 米空场和物理急停后才执行：

```bash
rosrun vrpn_imu_cmd_calibration run_calibration.py \
  --execute \
  --base-estimator-yaml /path/to/vehicle-estimator.yaml
```

默认拒绝与其他 `/cmd_vel` publisher 并存；VRPN/IMU stale、越界、超时、信号
或异常都会持续发布零速并终止。运行目录同时保存 bag、phase、完整 rosparam、
topic 清单和 manifest，录制结束后自动分析。默认输出到
`~/Documents/XGC/Calibration/vrpn-imu-cmd/`（尊重 `XDG_DOCUMENTS_DIR`），也可用
`--output-root` 或 `XGC_VRPN_IMU_CMD_OUTPUT_ROOT` 覆盖。

推荐先使用统一 E2E 入口。仿真使用独立 ROS master 和专用话题，不会命中车辆
`/cmd_vel`；物理预检只订阅传感器并重复发布零速：

```bash
# 自动闭环：cmd -> 仿真车辆 -> VRPN/IMU -> bag -> 分析 -> YAML -> 安装 dry-run
rosrun vrpn_imu_cmd_calibration run_vehicle_calibration_e2e.sh simulation

# 实时检查 topic 类型/新鲜度、静止性和 cmd_vel publisher 冲突；不发非零速度
rosrun vrpn_imu_cmd_calibration run_vehicle_calibration_e2e.sh physical-preflight
```

实车模式仍要求现场操作员、空的 8×8 米场地、可用物理急停、车辆 ID 和现有
estimator YAML。只有显式确认后才发非零速度：

```bash
export XGC_VRPN_IMU_CMD_VEHICLE_ID=scout
export XGC_VRPN_IMU_CMD_BASE_ESTIMATOR_YAML=/absolute/path/to/vehicle-estimator.yaml
export XGC_VRPN_IMU_CMD_PHYSICAL_CONFIRMED=YES
rosrun vrpn_imu_cmd_calibration run_vehicle_calibration_e2e.sh physical
```

物理 E2E 完成后会验证 bag、围栏、末尾零速、1–6 级质量门和生成 YAML，并对
估计器/可选控制器目标执行安装 dry-run；不会自动覆盖运行配置。

已有 bag 可单独分析：

```bash
rosrun vrpn_imu_cmd_calibration analyze_calibration_bag.py \
  calibration.bag \
  --base-estimator-yaml /path/to/vehicle-estimator.yaml \
  --output-dir /tmp/vrpn-imu-cmd-result
```

## YAML 联动

分析输出契约见 [`docs/output-schema.md`](docs/output-schema.md)：

- `calibration.yaml`：带证据、质量门和三帧语义的版本化标定资产。
- `estimator.yaml`：状态估计器直接消费的完整配置或参数 overlay。
- `controller.yaml`：只固定 `state_source: state_estimator` 和对应状态话题，不调
  NMPC 权重。

PASS 后可先 dry-run，再原子写入车辆配置；写入时会保存时间戳备份：

```bash
rosrun vrpn_imu_cmd_calibration install_calibration.py RESULT_DIR \
  --estimator-target /path/to/vehicle-estimator.yaml \
  --controller-target /path/to/unicycle_ugv_controller.yaml

# 确认 diff 后增加 --apply
```

也可以不改源文件，直接让生成的 estimator YAML 启动估计器，并强制控制器通过
估计状态联动：

```bash
roslaunch vrpn_imu_cmd_calibration calibrated_ugv_stack.launch \
  estimator_config:=/absolute/path/to/RESULT_DIR/estimator.yaml \
  ns:=ugv1
```

当前估计器没有 `T_CI` 或 gyro scale 的运行时输入。工具会辨识并检查它们；若
安装偏角或比例超限，整份资产 FAIL，`extrinsic_verified` 保持 false，绝不写入
虚构参数。

## 测试

```bash
PYTHONPATH=src python3 -m unittest discover -s test -v
scripts/run_vehicle_calibration_e2e.sh simulation
```

## APT

独立包名为 `ros-noetic-xgc2-vrpn-imu-cmd-calibration`。它只硬依赖完成激励、
录包、分析和 YAML 生成所需的 ROS/Python 运行库；估计器与控制器是生成结果的
可选消费者，不进入 Debian `Depends`。

生产发布只通过 `xgc2-devops` 的 `release-orchestrator` 复用 exact-SHA push CI
产物；产品仓自身不直接修改 APT 索引。
