# 栅格地图：读文件、膨胀、起终点
from pathlib import Path  # 拼路径
import numpy as np  # 数组
import params  # 超参


def map_path(name):
    # 返回 maps/name.txt
    here = Path(__file__).resolve().parent.parent  # 内容二根目录
    return here / "maps" / f"{name}.txt"  # 地图文件


def load_map(name):
    # 读字符地图，返回静态栅格、起点、终点
    lines = []  # 存放有效行
    text = map_path(name).read_text(encoding="utf-8")  # 读文本
    for raw in text.splitlines():  # 逐行
        line = raw.rstrip("\n")  # 去换行
        if line:  # 跳过空行
            lines.append(line)  # 保留
    rows = len(lines)  # 行数
    cols = max(len(x) for x in lines)  # 列数
    grid = np.zeros((rows, cols), dtype=np.int32)  # 0 可走
    start = None  # 起点
    goal = None  # 终点
    for r, line in enumerate(lines):  # 填格
        for c, ch in enumerate(line):  # 每列
            if ch in "#1":  # 墙
                grid[r, c] = 1  # 静态障碍
            elif ch in "S":  # 起点
                start = (r, c)  # 记录
            elif ch in "G":  # 终点
                goal = (r, c)  # 记录
    if start is None or goal is None:  # 缺标记
        raise ValueError("地图必须有 S 和 G")  # 报错
    rad = params.INFLATE.get(name, 0)  # 膨胀半径
    if rad > 0:  # 需要膨胀
        grid = inflate(grid, rad)  # 扩墙
    return grid, start, goal  # 三件套


def inflate(grid, rad):
    # 障碍向外扩 rad 格，当安全半径
    out = grid.copy()  # 副本
    occ = np.argwhere(grid == 1)  # 原障碍坐标
    h, w = grid.shape  # 尺寸
    for r, c in occ:  # 每个障碍
        for dr in range(-rad, rad + 1):  # 邻域行
            for dc in range(-rad, rad + 1):  # 邻域列
                nr, nc = r + dr, c + dc  # 新坐标
                if 0 <= nr < h and 0 <= nc < w:  # 在界内
                    out[nr, nc] = 1  # 标障碍
    return out  # 膨胀后


def in_bounds(grid, r, c):
    # 是否在地图内
    return 0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]  # 边界判断


def blocked_static(grid, r, c):
    # 静态是否不可走
    if not in_bounds(grid, r, c):  # 界外当墙
        return True  # 不可走
    return bool(grid[r, c] == 1)  # 墙格


def neighbors4(grid, r, c):
    # 四连通可走邻居（只看静态）
    out = []  # 结果
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):  # 上下降左
        nr, nc = r + dr, c + dc  # 邻格
        if not blocked_static(grid, nr, nc):  # 可走
            out.append((nr, nc))  # 加入
    return out  # 邻居列表


def manhattan(a, b):
    # 曼哈顿距离
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # |dr|+|dc|


def local_window(grid, dyn_set, pos, size):
    # 以机器人为中心的局部观测：0 空 1 静 2 动
    half = size // 2  # 半窗
    win = np.ones((size, size), dtype=np.float32)  # 界外当墙
    r0, c0 = pos  # 中心
    for i in range(size):  # 窗行
        for j in range(size):  # 窗列
            r = r0 + i - half  # 世界行
            c = c0 + j - half  # 世界列
            if not in_bounds(grid, r, c):  # 出界
                win[i, j] = 1.0  # 当墙
            elif (r, c) in dyn_set:  # 动态障碍
                win[i, j] = 2.0  # 标记 2
            elif grid[r, c] == 1:  # 静障碍
                win[i, j] = 1.0  # 标记 1
            else:  # 空
                win[i, j] = 0.0  # 可走
    return win  # 窗口
