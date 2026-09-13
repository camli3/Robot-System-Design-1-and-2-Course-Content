# 在稀疏图上训练 DQN 局部策略，权重写入 weights/
import sys  # 路径
from pathlib import Path  # 文件

ROOT = Path(__file__).resolve().parent  # 根
sys.path.insert(0, str(ROOT))  # import

import numpy as np  # 随机
import params  # 超参
from env.grid_map import load_map, manhattan  # 地图
from env.dynamic import build_obstacles, dyn_set  # 动态
from global_planner.astar import astar  # 全局
from pipeline.core import step_pose  # 移动
from local_planner.dqn_agent import DQNAgent, Replay, state_vec, action_to_delta  # DQN


def episode(grid, start, goal, path, agent, mem, eps):
    # 一局训练
    obstacles = build_obstacles("sparse", params.DYNAMICS)  # 障碍
    pos = start  # 出生
    total = 0.0  # 回报
    for _ in range(params.TRAIN_MAX_STEPS):  # 限步
        dyn = dyn_set(obstacles)  # 动态
        s = state_vec(grid, dyn, pos, path, goal)  # 状态
        a = agent.act(s, eps)  # 动作
        delta = action_to_delta(a)  # 增量
        nxt, hit = step_pose(grid, dyn, pos, delta)  # 走
        done_g = nxt == goal  # 到终点
        if hit:  # 碰撞
            r = -25.0  # 大负
        elif done_g:  # 成功
            r = 40.0  # 大正
        else:  # 过程
            r = -0.08  # 步惩罚
            if manhattan(nxt, goal) < manhattan(pos, goal):  # 靠近
                r += 0.9  # 奖
            if nxt in path:  # 在指引线上
                r += 0.35  # 奖
        done = hit or done_g  # 结束
        ns = state_vec(grid, dyn, nxt, path, goal)  # 下一状态
        mem.add(s, a, r, ns, done)  # 进池
        agent.train_batch(mem, params.DQN_BATCH)  # 学一步
        total += r  # 累加
        pos = nxt  # 更新
        for ob in obstacles:  # 障碍动
            ob.step(grid)
        if done:  # 结束局
            break
    return total, pos == goal  # 回报、是否成功


def main():
    # 训练主循环
    grid, start, goal = load_map("sparse")  # 稀疏图
    path, _, _ = astar(grid, start, goal)  # 指引线
    dim = params.WINDOW * params.WINDOW + 4  # 维
    agent = DQNAgent(dim, params.DQN_ACTIONS)  # 网
    mem = Replay(params.DQN_MEMORY, dim)  # 池
    ok_n = 0  # 近 20 局成功
    hist = []  # 成功标记
    for ep in range(1, params.TRAIN_EPISODES + 1):  # 局
        frac = min(1.0, ep / params.DQN_EPS_DECAY)  # 衰减进度
        eps = params.DQN_EPS_START + (params.DQN_EPS_END - params.DQN_EPS_START) * frac
        ret, ok = episode(grid, start, goal, path, agent, mem, eps)  # 一局
        hist.append(1 if ok else 0)  # 记录
        if len(hist) > 20:  # 窗口
            hist.pop(0)  # 丢旧
        if ep % 25 == 0 or ep == 1:  # 打印
            print(f"ep={ep} ret={ret:.1f} eps={eps:.2f} win20={sum(hist)}/20")
    wp = ROOT / params.DQN_WEIGHT  # 权重路径
    agent.save(str(wp))  # 保存
    print(f"saved {wp}")  # 提示


if __name__ == "__main__":
    np.random.seed(2026)  # 复现
    main()  # 训练
