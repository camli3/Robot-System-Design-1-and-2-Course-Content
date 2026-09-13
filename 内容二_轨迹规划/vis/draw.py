# 画搜索过程与行驶轨迹
from pathlib import Path  # 存图
import numpy as np  # 画布
import matplotlib  # 后端
matplotlib.use("Agg")  # 无窗口
import matplotlib.pyplot as plt  # 绘图


def _ensure(out_dir):
    # 保证目录存在
    p = Path(out_dir)  # 路径
    p.mkdir(parents=True, exist_ok=True)  # 创建
    return p  # 返回


def _base_rgb(grid):
    # 白空、黑墙
    h, w = grid.shape  # 尺寸
    img = np.ones((h, w, 3), dtype=np.float32)  # 白底
    img[grid == 1] = (0.15, 0.15, 0.15)  # 墙
    return img  # RGB


def draw_search(grid, start, goal, expanded, path, title, save_path, snap_k=None):
    # 关闭/扩展用蓝，路径红，起绿终黄
    img = _base_rgb(grid)  # 底图
    n = len(expanded)  # 扩展数
    use = expanded if snap_k is None else expanded[:snap_k]  # 快照
    for i, (r, c) in enumerate(use):  # 渐变蓝
        t = i / max(n - 1, 1)  # 0~1
        img[r, c] = (0.25, 0.45 + 0.4 * t, 0.95)  # 蓝
    for r, c in path:  # 最终路径
        img[r, c] = (0.90, 0.15, 0.15)  # 红
    img[start] = (0.10, 0.80, 0.20)  # 绿起点
    img[goal] = (0.95, 0.85, 0.10)  # 黄终点
    fig, ax = plt.subplots(figsize=(6, 5))  # 画布
    ax.imshow(img, interpolation="nearest")  # 显示
    ax.set_title(title)  # 标题
    ax.set_xticks([])  # 去刻度
    ax.set_yticks([])
    fig.tight_layout()  # 紧凑
    fig.savefig(save_path, dpi=140)  # 存
    plt.close(fig)  # 关


def draw_traj(log, save_path, title=None):
    # 全局路径蓝、实际轨迹红
    grid = log["grid"]  # 地图
    img = _base_rgb(grid)  # 底
    for r, c in log["path"]:  # 全局
        img[r, c] = (0.25, 0.45, 0.95)  # 蓝
    for r, c in log["traj"]:  # 实际
        img[r, c] = (0.90, 0.20, 0.15)  # 红盖住
    img[log["start"]] = (0.10, 0.80, 0.20)  # 绿
    img[log["goal"]] = (0.95, 0.85, 0.10)  # 黄
    fig, ax = plt.subplots(figsize=(7, 5))  # 画布
    ax.imshow(img, interpolation="nearest")  # 显示
    name = title or f"{log['map']} + {log['local']}"  # 标题
    ok = "OK" if log["ok"] else "FAIL"
    ax.set_title(f"{name}  {ok}  steps={log['steps']}  coll={log['collisions']}")
    ax.set_xticks([])
    ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(save_path, dpi=140)
    plt.close(fig)
