# 全局 A* + 局部规划器 闭环仿真
import params  # 超参
from env.grid_map import load_map, blocked_static, manhattan  # 地图
from env.dynamic import build_obstacles, dyn_set  # 动态
from global_planner.astar import astar  # 全局
from local_planner.dwa import dwa_act  # DWA
from local_planner.dqn_agent import (  # DQN
    DQNAgent,
    state_vec,
    follow_or_dqn,
)


def _load_dqn(grid):
    # 建网络并尽量读权重
    dim = params.WINDOW * params.WINDOW + 4  # 状态维
    agent = DQNAgent(dim, params.DQN_ACTIONS)  # 网络
    from pathlib import Path  # 路径
    wp = Path(__file__).resolve().parent.parent / params.DQN_WEIGHT  # 权重
    if wp.is_file():  # 有文件
        agent.load(str(wp))  # 加载
    return agent  # 智能体


def step_pose(grid, dyn, pos, delta):
    # 执行一拍；撞障则停在原地并记碰撞
    nr = pos[0] + delta[0]  # 新行
    nc = pos[1] + delta[1]  # 新列
    if blocked_static(grid, nr, nc):  # 静障
        return pos, True  # 碰撞
    if (nr, nc) in dyn:  # 动态
        return pos, True  # 碰撞
    return (nr, nc), False  # 成功移动


def simulate(map_name, local="dwa"):
    # 跑一条端到端，返回日志字典
    grid, start, goal = load_map(map_name)  # 读图
    path, expanded, closed = astar(grid, start, goal)  # 全局
    if not path or path[-1] != goal:  # 无解
        return {"ok": False, "reason": "全局无路径"}  # 失败
    obstacles = build_obstacles(map_name, params.DYNAMICS)  # 动态
    agent = _load_dqn(grid) if local == "dqn" else None  # 可选 DQN
    pos = start  # 当前
    traj = [pos]  # 轨迹
    collisions = 0  # 碰撞计数
    for t in range(params.MAX_STEPS):  # 逐步
        if pos == goal:  # 到达
            break  # 结束
        dyn = dyn_set(obstacles)  # 当前动态格
        if local == "dwa":  # 管线 A
            delta = dwa_act(grid, dyn, pos, path, goal, traj[-12:])  # DWA
        else:  # 管线 B
            delta = follow_or_dqn(grid, dyn, pos, path, goal, agent, 0.0)  # 混合 DQN
        pos, hit = step_pose(grid, dyn, pos, delta)  # 移动
        if hit:  # 撞了
            collisions += 1  # 计数
        traj.append(pos)  # 记录
        for ob in obstacles:  # 障碍也走
            ob.step(grid)  # 更新
    ok = pos == goal  # 是否成功
    dev = 0  # 最大偏离
    pset = set(path)  # 路径集
    for p in traj:  # 每步
        if p not in pset:  # 离开指引线
            d = min(manhattan(p, q) for q in path)  # 到路径距离
            dev = max(dev, d)  # 最大偏离
    return {  # 日志
        "ok": ok,
        "map": map_name,
        "local": local,
        "grid": grid,
        "start": start,
        "goal": goal,
        "path": path,
        "expanded": expanded,
        "closed": closed,
        "traj": traj,
        "collisions": collisions,
        "steps": len(traj) - 1,
        "length": len(traj) - 1,
        "max_dev": dev,
    }
