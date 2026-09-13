# 离散 DWA：对速度窗口（四向+停）前向滚动打分
import params  # 权重
from env.grid_map import blocked_static, manhattan, in_bounds  # 几何

# 动作：(dr, dc) 上 下 左 右 停
ACTIONS = ((-1, 0), (1, 0), (0, -1), (0, 1), (0, 0))


def lookahead(path, pos, k):
    # 全局路径上超前 k 格的点
    if not path:  # 无路径
        return pos  # 原地
    if pos in path:  # 在路径上
        i = path.index(pos)  # 下标
    else:  # 偏离
        i = min(range(len(path)), key=lambda t: manhattan(path[t], pos))  # 最近点
    return path[min(i + k, len(path) - 1)]  # 前瞻点


def _rollout(grid, dyn, start, action, horizon):
    # 同一动作滚动 horizon 步，遇障停止
    cells = []  # 预测轨迹
    r, c = start  # 当前
    for _ in range(horizon):  # 向前
        nr, nc = r + action[0], c + action[1]  # 下一步
        if blocked_static(grid, nr, nc):  # 静障
            break  # 停
        if (nr, nc) in dyn:  # 动态
            break  # 停
        r, c = nr, nc  # 前进
        cells.append((r, c))  # 记录
    return cells  # 轨迹


def _clearance(cells, dyn):
    # 轨迹到动态障碍的最小曼哈顿距离
    if not dyn:  # 无动态
        return 5.0  # 默认安全
    if not cells:  # 没动
        return 0.0  # 0
    best = 99  # 大数
    for cell in cells:  # 轨迹点
        for d in dyn:  # 障碍
            best = min(best, manhattan(cell, d))  # 更新
    return float(best)  # 间隙


def dwa_act(grid, dyn, pos, path, goal, recent=None):
    # 选得分最高的动作
    recent = recent or []  # 最近轨迹，防振荡
    if pos in path:  # 还在指引线上
        i = path.index(pos)  # 下标
        nxt = path[min(i + 1, len(path) - 1)]  # 下一格
        if nxt != pos and nxt not in dyn:  # 前方无动态
            return (nxt[0] - pos[0], nxt[1] - pos[1])  # 直接跟 A*
    if pos not in path:  # 已经绕开
        near = min(path, key=lambda q: manhattan(q, pos))  # 最近路径点
        for cand in (
            (1 if near[0] > pos[0] else -1 if near[0] < pos[0] else 0, 0),
            (0, 1 if near[1] > pos[1] else -1 if near[1] < pos[1] else 0),
        ):
            if cand == (0, 0):  # 无效
                continue
            nr, nc = pos[0] + cand[0], pos[1] + cand[1]  # 候选
            if (not blocked_static(grid, nr, nc)) and ((nr, nc) not in dyn):
                return cand  # 先回到指引线
    look = lookahead(path, pos, params.LOOKAHEAD)  # 前瞻
    path_set = set(path)  # 加速查询
    best_s = -1e9  # 最佳分
    best_a = (0, 0)  # 默认停
    for act in ACTIONS:  # 每个窗口速度
        cells = _rollout(grid, dyn, pos, act, params.DWA_HORIZON)  # 预测
        if act != (0, 0) and not cells:  # 一动就撞
            continue  # 丢弃
        last = cells[-1] if cells else pos  # 末端
        s_look = -params.DWA_W_LOOK * manhattan(last, look)  # 靠近前瞻
        s_goal = -0.35 * manhattan(last, goal)  # 靠近终点
        on_path = 1.0 if last in path_set else 0.0  # 是否在指引线上
        s_path = params.DWA_W_PATH * on_path  # 贴路径奖
        s_clr = params.DWA_W_CLEAR * min(_clearance(cells or [pos], dyn), 4)  # 间隙
        s_go = params.DWA_W_STEP * len(cells)  # 愿走
        score = s_look + s_goal + s_path + s_clr + s_go  # 总分
        if last in recent[-10:]:  # 刚走过
            score -= 2.5  # 防来回抖
        if act == (0, 0) and any(manhattan(pos, d) <= 1 for d in dyn):
            score -= 1.5  # 贴障碍时少傻等
        if score > best_s:  # 更好
            best_s = score  # 更新
            best_a = act  # 记录
    return best_a  # 动作
