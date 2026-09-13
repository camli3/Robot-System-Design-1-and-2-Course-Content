# 内容二：栅格规划超参（改这里即可复现实验）

# 地图名：sparse / maze / corridor
MAP_NAME = "sparse"

# 膨胀半径（格）；狭缝图请保持 0
INFLATE = {"sparse": 0, "maze": 0, "corridor": 0}

# 邻接：4 连通，与 DQN 动作一致
CONNECT4 = True

# A* 启发：曼哈顿
# 局部窗口边长（奇数）
WINDOW = 7

# DWA 前向预测步数与权重
DWA_HORIZON = 3
DWA_W_LOOK = 2.0      # 靠近前瞻点
DWA_W_PATH = 1.2      # 贴全局路径
DWA_W_CLEAR = 1.5     # 离动态障碍远
DWA_W_STEP = 0.15     # 愿意前进

# 仿真
MAX_STEPS = 400
LOOKAHEAD = 3         # 沿全局路径前瞻格数

# DQN
DQN_ACTIONS = 5       # 上/下/左/右/停
DQN_HIDDEN = 64
DQN_GAMMA = 0.95
DQN_LR = 0.002
DQN_EPS_START = 0.9
DQN_EPS_END = 0.05
DQN_EPS_DECAY = 400
DQN_BATCH = 32
DQN_MEMORY = 4000
DQN_WEIGHT = "weights/dqn_sparse.npz"

# 训练
TRAIN_EPISODES = 250
TRAIN_MAX_STEPS = 180

# 动态障碍：不写入全局地图（A* 看不见）
# type=pingpong 两端折返；type=loop 循环
DYNAMICS = {
    "sparse": [
        {"type": "pingpong", "cells": [(7, c) for c in range(2, 19)]},
        {"type": "loop", "cells": [
            (4, 3), (4, 4), (4, 5), (4, 6), (5, 6),
            (6, 6), (6, 5), (6, 4), (6, 3), (5, 3),
        ]},
    ],
    "maze": [
        {"type": "pingpong", "cells": [(7, c) for c in range(4, 17)]},
    ],
    "corridor": [
        {"type": "pingpong", "cells": [(13, c) for c in range(8, 32)]},
    ],
}
