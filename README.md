# microduck-servo-dashboard

用于 Radxa Zero 3W + URT-2 + Feetech HD-1910 的 MicroDuck 调试仪表盘。它订阅
`robotd` 的 `robot.state`，实时显示 15 个关节的指令、实测、误差、相位滞后、
振荡和电流，并提供姿态、策略和动作控制。

## v3.0.0

- 实时舵机曲线、关节表、记录和 CSV/JSONL 下载。
- `init`、`enable`、暂停、卸力和速度控制。
- 训练模型的正立 3D 姿态、关节方向检查、`+15°` 预览和 home 参考图。
- 策略慢放：50/25/10/5/2 Hz。
- 手工单步推理：一次点击只运行一帧 ONNX。
- 动作按钮：坐下/起立、捡取、左踢、前滚翻。
- 自动处理 root 权限，原子更新 `robotd.toml`，并热重载策略。

## 依赖

- Python 3.9+，Dashboard 本身只用标准库。
- 正在运行的 `robotd`，默认 socket 为 `/run/robotd.sock`。
- 3D 页面需要同目录下的 `duck.bin`。
- 慢放和单步需要带 `policy_step_divisor` 支持的 companion `robotd`。

慢放使用的 `robotd` 二进制放在工作区的 `slowmo-v2` 交付目录中。官方原版
`robotd` 不认识这个配置项。

## 启动

把 `duck_dash.py` 和 `duck.bin` 放在板子的同一目录：

```bash
cd ~/microduck-servo-dashboard
sudo python3 duck_dash.py
```

如果忘记 `sudo`，程序会在打开网页端口前自动通过 `sudo` 重启自身。没有交互式终端
时请直接使用 `sudo`。

从 Windows 建立 SSH 转发：

```powershell
ssh -N -L 8770:127.0.0.1:8770 radxa@<板子IP>
```

浏览器打开：

```text
http://127.0.0.1:8770
```

## 策略慢放

页面顶部可选择：

| 显示 | 策略推理频率 | 说明 |
| --- | ---: | --- |
| 1× | 50 Hz | 正常训练节奏 |
| 2× | 25 Hz | 每两拍推理一次 |
| 5× | 10 Hz | 每五拍推理一次 |
| 10× | 5 Hz | 每十拍推理一次 |
| 25× | 2 Hz | 逐段观察 |
| single-step | 手动 | 点击一次按钮只推理一帧 |

控制环仍以正常运行频率写舵机；未推理的拍保持上一组完整目标。慢放只作用于
`walk`/`stand`，技能和起坐保持训练时长。

应用档位会先卸力，原子更新 `/etc/robot/robotd.toml`，再调用
`robot.reloadPolicies`。应用后重新执行：

```text
init -> enable
```

慢放用于观察和定位，不能用于评价策略原本是否正常。调试结束应切回 `1×`。

## 单步推理

1. 选择 `single-step · manual` 并应用。
2. 执行 `init`。
3. 执行 `enable`；此时只保持当前位置。
4. 点击“单步推理”运行第一帧。
5. 每再点击一次，就用最新传感器状态和上一帧动作运行一帧。

## 动作按钮

动作按钮需要先 `init` 和 `enable`。它们和下面命令等价：

```bash
robotctl robot do sit_toggle
robotctl robot do ground_pick
robotctl robot do kick_left
robotctl robot do roulade
```

当前 HD-1910 模型包没有 `kick_right`，因此页面不放右踢按钮。这些定时动作不受
慢放影响。

## 3D 方向检查

扶住鸭子后按 `relax`，选择一个关节，再用手只转动该关节：

- 网页中动的应是同一个关节；
- 左侧实测姿态应跟随右侧 `+15°` 预览；
- 反向说明该关节 `direction` 需要翻转。

3D 视图固定躯干竖直，只用于检查关节；IMU 倾斜请使用 `robotctl monitor`。

## 权限与排错

配置路径默认是 `/etc/robot/robotd.toml`。检查：

```bash
grep -nE '^\[policy\]|^policy_step_divisor' /etc/robot/robotd.toml
sudo lsattr /etc/robot/robotd.toml
```

如果文件带 `i` 属性：

```bash
sudo chattr -i /etc/robot/robotd.toml
```

常见错误：

- `Permission denied`：用 `sudo python3 duck_dash.py` 启动。
- `switch ... to single-step first`：TOML 当前不是 `policy_step_divisor = 0`。
- 单步无反应：确认已安装 companion `robotd`，并完成 `init -> enable`。

## 安全

- `relax` 会断电，鸭子会倒。
- 暂停策略不等于卸力。
- 前滚翻和踢腿第一次测试时清空空间并扶稳机器人。
- 仪表盘默认只监听 `127.0.0.1`，通过 SSH 转发访问。
