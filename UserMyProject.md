# UserMyProject：内容一 / 内容二 使用说明

作业目录：`机器人系统设计课程作业`。两个课题互不依赖，分开打开对应子文件夹即可。

| 课题 | 文件夹 | 平台 | 你日常只改/只跑什么 |
|---|---|---|---|
| 内容一 四旋翼控制 | `内容一_四旋翼\` | MATLAB R2024a + Simulink | 改 `init.m` 三行，运行 `init` |
| 内容二 轨迹规划 | `内容二_轨迹规划\` | Python 3 + numpy + matplotlib | 改 `init_run.py` 两行，或跑 `run_compare.py` |

Cursor 的 MATLAB 扩展**不能**运行 `.slx`。内容一仿真用 MATLAB 桌面或扩展跑 `.m`；看示波器必须用 MATLAB 打开 Simulink。

---

# 一、内容一：四旋翼控制算法设计与仿真

## 1. 打开工程

1. 启动 **MATLAB**（不要只用 Cursor 点 `.slx`）。
2. 当前文件夹切到：

`...\机器人系统设计课程作业\内容一_四旋翼`

3. 在命令行确认：

```matlab
cd('完整路径\内容一_四旋翼')
addpath(pwd)
```

## 2. 日常仿真（推荐）

打开 `init.m`，**只改最上面 3 行**，保存后运行 `init`（或编辑器 Run）：

```matlab
algo = 1;        % 1=串级 PID    2=反步
exp_id = 1;      % 1悬停 2升降 3前后 4左右 5五航点
use_slx = 0;     % 0=本目录 RK4（快，出图）  1=Simulink
```

一次运行 = **一组**（一种算法 × 一个实验），弹出/保存 **5 张图**：

| 文件 | 含义 |
|---|---|
| `e{实验}_a{算法}_traj3d.png` | 三维轨迹（蓝虚线期望，红线实际；跟太紧时蓝线会被盖住） |
| `*_pos_err.png` | 位置误差 \(e_x,e_y,e_z\)（米） |
| `*_att.png` | 姿态角（度） |
| `*_rate.png` | 体轴角速度（看超调/发散） |
| `*_u.png` | 控制量 \(U_1\sim U_4\) |

同时生成 `result_e{实验}_a{算法}.mat`。图在 `report_figs\`。

### 必须跑齐的组合（PPT）

每种 `algo` 都要把 `exp_id` 1～5 跑一遍，共 10 组。可手动改 10 次 `init`，或一次出齐：

```matlab
run_all
```

`run_all` 后台存图、不弹窗。完成后看 `report_figs\compare.csv`。

### 文件名怎么读

- `e3_a2_pos_err.png` = 前后机动 + 反步 + 位置误差。
- e1/e2 的 att、rate 经常是 **0 度直线**：只垂直飞，机身不用转，正常。

## 3. 调参

改 `params.m` 后**再跑 `init`**（不要单独跑 `pid_ctrl`）。

| 目的 | 改什么 |
|---|---|
| PID 跟得慢/晃 | `Kp_pos` `Kd_pos`（x,y,z 三个数）；姿态环 `Kp_att` `Kd_att` |
| 反步松/紧 | `bs_c1` `bs_c2` 外环，`bs_c3` `bs_c4` 内环（内环要更大） |
| 防发散 | 减小外环 P，或减小 `att_lim` |
| 抗风（加分） | `par.wind = [0.5; 0; 0];` |
| 换一组五航点 | `rng_seed` 后设 `exp_id=5` |

PPT「调参优化」：同一 `exp_id`+`algo`，改一档增益，截调前/调后两套图。

## 4. Simulink 示波器（PPT 要求）

**不要在 Cursor 里运行 `.slx`。**

1. `init.m` 设 `use_slx = 1`。
2. 在 **MATLAB 命令行** 先 `cd` 到本目录，再运行 `init`。  
   会 `addpath`、写入 `slx_config`，然后 `sim('Quadrotor_Control')`。
3. 或先跑一次 `use_slx=0` 的 `init`，再：

```matlab
bdclose('all')
cd('...\内容一_四旋翼')
addpath(pwd)
open_system('Quadrotor_Control')
```

点 Run。`Scope_x` 看 12 维状态，`Scope_U` 看控制量。

若报错 `函数或变量 slx_ctrl 无法识别`：当前目录不在路径上。用上面的 `cd` + `addpath`，不要只从资源管理器双击 slx。

模型坏了或要重建：

```matlab
build_simulink
```

## 5. 内容一文件要不要点

| 文件 | 你要不要动 |
|---|---|
| `init.m` | **主入口，改三行** |
| `params.m` | 调参时改 |
| `run_all.m` | 出齐 10 组图 |
| `Quadrotor_Control.slx` | 只在 MATLAB 里打开看示波器 |
| `simulate.m` / `pid_ctrl.m` / `backstepping_ctrl.m` 等 | 答辩讲原理时看，日常不用点运行 |
| `introduction.md` | 50 张图的文字说明，写报告用 |

## 6. 常见问题

- **一次 init 弹出 5 张图：** 同一组仿真的 5 类曲线，不是 5 个实验。
- **e4_a2 只有 traj3d：** 中途关图或打断了 `run_all`。设 `exp_id=4; algo=2` 再跑 `init`。
- **traj3d 看不见蓝虚线：** 跟踪到毫米/厘米，红线盖住期望，看 `pos_err`。
- **VS Code / Cursor 跑 slx 报错：** 扩展只支持 `.m`。

---

# 二、内容二：移动机器人运动轨迹规划

## 1. 环境

需要：Python 3。在 `内容二_轨迹规划\` 下安装依赖：

```text
python -m pip install -r requirements.txt
```

（`numpy`、`matplotlib`，版本见该文件）

在终端里把当前目录切到：

`...\机器人系统设计课程作业\内容二_轨迹规划`

后面所有 `python xxx.py` 都在这个目录执行。

## 2. 两条管线、三张地图

| 代码 | 含义 |
|---|---|
| `sparse` | 稀疏障碍 |
| `maze` | 密集迷宫 |
| `corridor` | 狭长通道（有往复动态障碍） |
| `dwa` | 管线 A：A\* 全局 + DWA 局部 |
| `dqn` | 管线 B：A\* 全局 + DQN 局部（前方无障则跟 A\*） |

全局 A\* **看不见**动态障碍；局部窗口才能看见。动态逻辑在 `params.py` 的 `DYNAMICS`（往复 pingpong / 循环 loop）。

图例（轨迹图）：黑=墙，蓝=A\* 全局路径，红=实际走过，绿=起点，黄=终点。

## 3. 看全局搜索过程（PPT：逐步展示）

```text
python init_search.py
```

默认地图是 `params.py` 里的 `MAP_NAME`。改成 `maze` 或 `corridor` 再跑，会覆盖生成该图的搜索快照。

或在 Python 里：

```python
from init_search import main
main("maze")
```

输出在 `report_figs\`：

- `search_{地图}_astar_p25/p50/p75/final.png`：A\* 扩展到 25%/50%/75%/全部
- `search_{地图}_dijkstra_*.png`：Dijkstra 对照（无启发，狭缝图扩展格更多）

蓝=已扩展格，红=最终路径，绿=S，黄=G。`final` 才画完整路径。

## 4. 跑一条端到端（全局+局部+动态障碍）

打开 `init_run.py`，改开头两行：

```python
MAP_NAME = "sparse"   # sparse / maze / corridor
LOCAL = "dwa"         # dwa 或 dqn
```

```text
python init_run.py
```

终端会打印 `ok=True/False`、步数、碰撞次数。图：`report_figs\traj_{地图}_{dwa|dqn}.png`。

建议至少亲手跑：

1. `sparse` + `dwa`
2. `corridor` + `dwa`（狭缝上 DWA 可能偏 1 格绕行动态障碍）
3. `maze` + `dqn`

## 5. 一次出齐对比（写报告用）

```text
python run_compare.py
```

会：

- 三张地图的 A\* / Dijkstra 搜索图
- 三张地图 × 两条管线的轨迹图
- `report_figs\compare.csv`（成功、步数、碰撞、偏离、全局路径长、扩展格数）

当前实现下 6 组端到端均可到终点。狭缝 `A*+DWA` 步数会比全局路径多 1、`max_dev=1`，表示绕行后回到指引线。

## 6. 调参与重训 DQN

改 `params.py`：

| 项 | 变量 |
|---|---|
| 默认搜索地图 | `MAP_NAME` |
| DWA 贴路径/避障 | `DWA_W_PATH` `DWA_W_CLEAR` `DWA_W_LOOK` |
| 动态障碍格子 | `DYNAMICS`（必须写在空地上，否则障碍不更新） |
| 仿真步数上限 | `MAX_STEPS` |

重训局部 DQN（权重：`weights\dqn_sparse.npz`）：

```text
python train_dqn.py
```

管线 B **不是**纯端到端乱走：前方无动态时跟 A\*，被挡才问 DQN。即使权重一般，跟路径仍能到终点。这是 PPT 说的「DRL 与传统混合」。

改地图字符：`maps\*.txt` 里 `#` 墙、`.` 空、`S` 起点、`G` 终点。改完用 `init_search` 确认 A\* 仍有路径，再跑 `init_run`。

## 7. 内容二文件要不要点

| 文件 | 你要不要动 |
|---|---|
| `init_run.py` | **单次行驶入口** |
| `init_search.py` / `params.MAP_NAME` | 搜索动画 |
| `run_compare.py` | 全量对比 |
| `params.py` | 调参、障碍逻辑 |
| `maps\*.txt` | 改地图 |
| `train_dqn.py` | 可选重训 |
| `env\` `global_planner\` `local_planner\` `pipeline\` | 答辩讲代码时看 |

## 8. 常见问题

- **必须在内容二根目录运行**，否则 `import env` 失败。
- **A\* 报无路径：** 检查 S/G 是否被墙围死，或 `INFLATE` 把狭缝堵死（狭缝保持 0）。
- **动态障碍没出现：** `DYNAMICS` 的格子必须是 `.`，不能写在 `#` 上。
- **中文标题方框：** 图题已用英文；不影响结果。
- **DQN 训练回报一直负：** 正常，训练环境难；推理时混合策略仍可靠。以 `init_run` / `compare.csv` 为准。

---

# 三、写报告时两份图往哪贴

**内容一** `内容一_四旋翼\report_figs\`

- 精度：各 `*_pos_err.png` + `compare.csv`
- 稳定：e3/e4/e5 的 `*_att.png` `*_rate.png`（e1/e2 直线要解释）
- 复杂轨迹：`e5_a1_*` 与 `e5_a2_*` 对比
- 图意说明：`内容一_四旋翼\introduction.md`

**内容二** `内容二_轨迹规划\report_figs\`

- 搜索过程：`search_*_p25` → `final`（证明「逐步展示」）
- 端到端：`traj_sparse/maze/corridor_dwa.png` 与 `*_dqn.png`
- 对比表：`compare.csv`
- 障碍运动逻辑：抄 `params.py` 里 `DYNAMICS` 那段写进「方法」

原理图要手绘、公式用公式编辑器（PPT 规定）。程序和图可以截本工程。

---

# 四、答辩 30 秒流程

**内容一：** 打开 `init.m` → `exp_id=5, algo=1` 跑一次 → 再改 `algo=2` → 指 `pos_err` 和 `compare.csv` 说反步水平跟踪更好 → 需要时 MATLAB 打开 slx 给示波器。

**内容二：** `python init_search.py` 指 A\* 比 Dijkstra 扩展少（迷宫/狭缝）→ `python init_run.py` 设 `corridor`+`dwa` 指红线相对蓝线偏开一格 → 打开 `compare.csv` 说两条管线都能到终点。
