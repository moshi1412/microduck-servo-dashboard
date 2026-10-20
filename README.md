# microduck-servo-dashboard

## v2.0.0

- 新增训练模型的 3D 关节方向检查。
- 新增固定正立的 home 姿态参考图。
- 3D 画布保持原始宽高比，模型不再被横向压扁。
- 窄屏自动改为单列布局。

## 3D 方向确认（2026-10-20）

页面会读取 `robot.state` 里的 15 个实测关节角，用训练时的同一份 `duck.bin`
渲染鸭子。左侧是实机当前姿态，右侧是所选关节 `+15°` 的期望姿态；橙色箭头是
模型正方向，蓝色部件是所选关节。右侧另有固定正立的 home 参考图，不受 IMU
安装方向影响。三个 3D 视图都锁定了躯干竖直，只用于检查关节；要看 IMU 倾斜，
继续使用 `robotctl monitor`。

安全用法：

```bash
# 板子上，先把 duck_dash.py 和 duck.bin 放进同一个目录
cd ~/microduck-servo-dashboard
sudo python3 duck_dash.py
```

电脑端建立 SSH 转发：

```powershell
ssh -L 8770:127.0.0.1:8770 radxa@<板子IP>
```

浏览器打开 `http://127.0.0.1:8770`。扶住鸭子，按页面上的 `relax`，选择一个关节，
只用手转动这一颗；左侧蓝线姿态若跟右侧橙色预览同向，说明 ID、关节映射和
`direction` 同号。若反向，就是这个关节的 `direction` 需要翻转。不要让策略
`enable` 来做这项检查。

一块跑在机器人的板子上的网页仪表盘，用来回答一个问题：**是舵机的问题，还是策略/指令的问题。**

它订阅 `robotd` 的 `robot.state`（50 Hz），实时显示 15 个关节的指令、实测、跟踪误差、相位滞后、振荡频率、电流，以及指令是否超出了这颗舵机自己 EEPROM 里的行程窗口；页面上的按钮可以直接 `init`、`enable`、暂停和断力矩。

在 Radxa Zero 3W + URT-2 + 飞特 HD-1910M 的 MicroDuck 上开发和验证，但只要 `robotd` 在跑、`/run/robotd.sock` 在，任何后端都能用。

## 依赖

- 运行仪表盘：Python 3.9+，**只用标准库**（`socket` / `http.server` / `json`）
- 离线出图 `plot_state.py`：`matplotlib`
- 读舵机限位 `set_limits.py`：`pyserial`

## 快速开始

```bash
# 电脑上
scp duck_dash.py radxa@<板子IP>:~

# 板子上
cd ~ && sudo python3 duck_dash.py

# 电脑上另开一个终端做端口转发
ssh -L 8770:127.0.0.1:8770 radxa@<板子IP>
# 浏览器打开 http://127.0.0.1:8770
```

标题右边有版本号，用来确认板上跑的是哪一版。

## 页面

**按钮**

| 按钮 | 发的命令 | 效果 |
| --- | --- | --- |
| init | `robot.init` | 通电并斜坡到 home 姿态 |
| enable | `robot.enable {on:true}` | 把机器人交给策略 |
| 暂停（空格） | `robot.enable {on:false}` | 策略停手，关节**仍带电**保持姿势 |
| 停止 / 脱力（Esc） | `robot.relax` | 断掉全部力矩，电流归零 |
| 按住前进 | 持续 `robot.move {vx}` | 松开发 0 并停；deadman 500 ms |
| 自动暂停 | 开关 | 误差/电压超阈值时自动停止，默认**关** |

**14 格曲线**（嘴不参与步态，不画）

蓝 = 指令（策略请求的角度，**裁剪之前**），绿 = 实测，红虚线 = 这颗舵机自己的行程窗口。每格纵轴各自缩放，左上/左下是该格的最大/最小值，标题右侧是 `max|e|`。

**表格**

`cmd / meas / err` 是最后一帧；`mean|err| / max` 是窗口内的统计；`lag` 是指令与实测的互相关延迟，`r` 是那个相关的可信度（低于 0.3 说明这段里关节没怎么动，`lag` 别看）；`osc/s` 是实测速度每秒换向次数；`限位` 是"指令落在窗口外的帧比例 / 最多超出多少度"。

判定列是这些数字的粗分类：`跟得住`、`撞窗口`、`顶着（限位或卡死）`、`在振荡`、`太软/跟不上`、`稳态偏（零点）`。

## 数据

每次启动都会把**每一帧**追加到 `duck-run-<时间>.jsonl`（与 8 秒显示窗口无关，冻结也不影响写入）。页面上两个下载链接分别给出冻结窗口和整份记录。

离线出图：

```bash
python plot_state.py duck-run-20261018-153012.jsonl -o curves.png
python plot_state.py duck-run-xxx.jsonl --joints left_hip_roll,head_yaw
```

每行是一帧：

```json
{"t":337.265,"policy":"walk","joints":[...15 弧度...],"targets":[...15 弧度...],
 "currents_ma":[...15],"safety":{"fallen":true,"limp":false,"gain":200},"loop":{"hz":50.0,"missed":4}}
```

## 真实行程窗口

曲线上的红虚线和表格里的「限位」列需要一份从舵机读出来的窗口文件。读它要独占串口，所以 `robotd` 得先停：

```bash
sudo systemctl stop robotd
python3 set_limits.py --port /dev/ttyACM0 --save /etc/robot/limits.json
sudo systemctl restart robotd
```

没有这份文件时，界面照常工作，只是「限位」列和红虚线为空。

`set_limits.py` **不带 `--apply` 时是只读的**；带 `--apply` 会写舵机 EEPROM，那是不可中断的操作，用之前请确认窗口数值。

## 安全

- 「暂停」不等于「停止」：暂停后关节仍在用力保持姿势，若在较劲电流不会下降。防掉电要按「停止 / 脱力」。
- 自动停止（`--watch`）默认关闭，并且 enable 之后 3 秒内不会触发——机器人正在起身时最容易误判。
- 低压会自动停止：`--floor` 默认 6.6 V。
- 这个程序本身**不写**机器人任何东西，除了你按下的那三个按钮。

## 已知限制

- 没有 3D 模型。`robotctl monitor` 已经有（用 `robotctl/assets/duck.bin`），这里不重复实现。
- 只订阅本地 unix socket，不做认证；默认监听 `127.0.0.1`，请用 SSH 转发访问，不要直接暴露到网络。

## 版本

`duck_dash.py` 顶部的 `VERSION` 会显示在页面标题旁。每次行为变化都改它，省得对着旧页面判断新逻辑。
