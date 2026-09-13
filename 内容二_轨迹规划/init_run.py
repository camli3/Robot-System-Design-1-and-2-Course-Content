# 入口：改两行后跑一条端到端（出轨迹图）
MAP_NAME = "sparse"   # sparse / maze / corridor
LOCAL = "dwa"         # dwa 或 dqn

import sys  # 路径
from pathlib import Path  # 根

ROOT = Path(__file__).resolve().parent  # 根目录
sys.path.insert(0, str(ROOT))  # import

from pipeline.core import simulate  # 仿真
from vis.draw import draw_traj  # 画轨迹


def main():
    # 跑一次并存图
    log = simulate(MAP_NAME, local=LOCAL)  # 仿真
    out = ROOT / "report_figs"  # 目录
    out.mkdir(exist_ok=True)  # 创建
    fp = out / f"traj_{MAP_NAME}_{LOCAL}.png"  # 文件
    draw_traj(log, fp)  # 画
    print(  # 摘要
        f"ok={log.get('ok')} steps={log.get('steps')} "
        f"coll={log.get('collisions')} -> {fp}"
    )


if __name__ == "__main__":
    main()  # 运行
