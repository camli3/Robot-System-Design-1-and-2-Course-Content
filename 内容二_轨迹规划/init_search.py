# 入口：按地图画 A* / Dijkstra 逐步搜索图
import sys  # 改路径
from pathlib import Path  # 根目录

ROOT = Path(__file__).resolve().parent  # 内容二根
sys.path.insert(0, str(ROOT))  # 可 import 包

from env.grid_map import load_map  # 读图
from global_planner.astar import astar  # A*
from global_planner.dijkstra import dijkstra  # Dijkstra
from vis.draw import draw_search  # 画图


def main(map_name="sparse"):
    # 生成搜索过程图
    out = ROOT / "report_figs"  # 输出
    out.mkdir(exist_ok=True)  # 建夹
    grid, start, goal = load_map(map_name)  # 地图
    for name, fn in (("astar", astar), ("dijkstra", dijkstra)):  # 两算法
        path, expanded, _ = fn(grid, start, goal)  # 搜索
        n = max(len(expanded), 1)  # 防 0
        for frac, tag in ((0.25, "p25"), (0.50, "p50"), (0.75, "p75"), (1.0, "final")):
            k = max(1, int(n * frac))  # 快照步
            draw_search(  # 存图
                grid,
                start,
                goal,
                expanded,
                path if frac == 1.0 else [],
                f"{map_name} {name} expand {k}/{n}",
                out / f"search_{map_name}_{name}_{tag}.png",
                snap_k=k,
            )
        print(f"{map_name} {name}: 路径长={len(path)} 扩展={n}")  # 终端


if __name__ == "__main__":  # 直接运行
    import params  # 默认地图
    main(params.MAP_NAME)  # 开跑
