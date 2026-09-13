# 管线 A：A* 全局 + DWA 局部
from pipeline.core import simulate  # 共用仿真


def run(map_name):
    # 跑管线 A
    return simulate(map_name, local="dwa")  # DWA
