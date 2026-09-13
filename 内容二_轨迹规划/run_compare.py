# 3 地图 × 2 管线对比，写 csv 与轨迹图
import csv  # 表
import sys  # 路径
from pathlib import Path  # 目录

ROOT = Path(__file__).resolve().parent  # 根
sys.path.insert(0, str(ROOT))  # import

from pipeline.core import simulate  # 仿真
from vis.draw import draw_traj  # 画
from init_search import main as dump_search  # 搜索图


def main():
    # 全量对比
    out = ROOT / "report_figs"  # 图目录
    out.mkdir(exist_ok=True)  # 建
    rows = []  # 表行
    for m in ("sparse", "maze", "corridor"):  # 三图
        dump_search(m)  # A*/Dijkstra 搜索快照
        for loc in ("dwa", "dqn"):  # 两管线
            log = simulate(m, local=loc)  # 跑
            draw_traj(log, out / f"traj_{m}_{loc}.png")  # 轨迹
            rows.append(  # 一行
                {
                    "map": m,
                    "pipeline": f"A*+{loc.upper()}",
                    "success": int(log["ok"]),
                    "steps": log["steps"],
                    "collisions": log["collisions"],
                    "max_dev": log["max_dev"],
                    "global_len": len(log["path"]),
                    "expanded": len(log["expanded"]),
                }
            )
            print(  # 进度
                f"{m:9s} {loc:3s} ok={log['ok']} "
                f"steps={log['steps']} coll={log['collisions']}"
            )
    csv_path = out / "compare.csv"  # 表
    with csv_path.open("w", newline="", encoding="utf-8") as f:  # 写
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))  # 头
        w.writeheader()  # 写头
        w.writerows(rows)  # 写行
    print(f"wrote {csv_path}")  # 提示


if __name__ == "__main__":
    main()  # 运行
