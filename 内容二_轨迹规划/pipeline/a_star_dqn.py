# 管线 B：A* 全局 + DQN（前方无障则跟路径）
from pipeline.core import simulate  # 共用仿真


def run(map_name):
    # 跑管线 B
    return simulate(map_name, local="dqn")  # DQN
