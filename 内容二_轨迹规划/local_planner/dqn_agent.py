# 两层 MLP 的 DQN（纯 numpy，不依赖 torch）
from pathlib import Path  # 权重路径
import numpy as np  # 矩阵
import params  # 超参
from env.grid_map import local_window, manhattan  # 观测
from local_planner.dwa import ACTIONS, lookahead  # 与 DWA 同一动作集


def state_vec(grid, dyn, pos, path, goal):
    # 局部窗展平 + 终点/路径方向
    win = local_window(grid, dyn, pos, params.WINDOW)  # 7x7
    look = lookahead(path, pos, params.LOOKAHEAD)  # 前瞻
    h, w = grid.shape  # 尺寸
    extra = np.array(  # 4 维几何
        [
            (goal[0] - pos[0]) / max(h, 1),  # 相对终点行
            (goal[1] - pos[1]) / max(w, 1),  # 相对终点列
            (look[0] - pos[0]) / max(h, 1),  # 相对路径行
            (look[1] - pos[1]) / max(w, 1),  # 相对路径列
        ],
        dtype=np.float32,
    )
    return np.concatenate([win.reshape(-1), extra])  # 状态向量


class Replay:
    # 循环回放池
    def __init__(self, cap, dim):
        self.s = np.zeros((cap, dim), np.float32)  # s
        self.a = np.zeros((cap,), np.int32)  # a
        self.r = np.zeros((cap,), np.float32)  # r
        self.ns = np.zeros((cap, dim), np.float32)  # s'
        self.d = np.zeros((cap,), np.float32)  # done
        self.n = 0  # 写入次数
        self.cap = cap  # 容量

    def add(self, s, a, r, ns, done):
        i = self.n % self.cap  # 下标
        self.s[i] = s  # 存 s
        self.a[i] = a  # 存 a
        self.r[i] = r  # 存 r
        self.ns[i] = ns  # 存 s'
        self.d[i] = float(done)  # 存结束
        self.n += 1  # 计数

    def sample(self, batch):
        m = min(self.n, self.cap)  # 有效数
        idx = np.random.randint(0, m, size=batch)  # 随机
        return self.s[idx], self.a[idx], self.r[idx], self.ns[idx], self.d[idx]  # 批


class DQNAgent:
    # Q(s) ≈ W2 relu(W1 s + b1) + b2
    def __init__(self, dim, n_act):
        rng = np.random.default_rng(2026)  # 可复现
        h = params.DQN_HIDDEN  # 隐层
        self.w1 = rng.normal(0, 0.15, (dim, h)).astype(np.float32)  # W1
        self.b1 = np.zeros((h,), np.float32)  # b1
        self.w2 = rng.normal(0, 0.15, (h, n_act)).astype(np.float32)  # W2
        self.b2 = np.zeros((n_act,), np.float32)  # b2
        self.n_act = n_act  # 动作数
        self.dim = dim  # 状态维
        self.lr = params.DQN_LR  # 学习率

    def _q(self, s):
        # 前向：支持 (D,) 或 (B,D)
        x = np.atleast_2d(s)  # 批
        h = np.maximum(0.0, x @ self.w1 + self.b1)  # ReLU
        q = h @ self.w2 + self.b2  # 线性
        return q, h, x  # Q 与中间量

    def q_values(self, s):
        # 只取 Q
        q, _, _ = self._q(s)  # 前向
        return q[0] if np.ndim(s) == 1 else q  # 还原形状

    def act(self, s, eps):
        # ε-贪心
        if np.random.rand() < eps:  # 探索
            return int(np.random.randint(self.n_act))  # 随机
        return int(np.argmax(self.q_values(s)))  # 利用

    def train_batch(self, mem, batch):
        # 一步 TD 更新
        if mem.n < batch:  # 样本不足
            return  # 跳过
        s, a, r, ns, d = mem.sample(batch)  # 取样
        q, h, x = self._q(s)  # 当前 Q
        nq, _, _ = self._q(ns)  # 下一 Q
        target = q.copy()  # 复制
        y = r + (1.0 - d) * params.DQN_GAMMA * np.max(nq, axis=1)  # Bellman
        rows = np.arange(batch)  # 行号
        target[rows, a] = y  # 只改动作维
        dq = (q - target) / batch  # dL/dQ
        dw2 = h.T @ dq  # dW2
        db2 = dq.sum(axis=0)  # db2
        dh = dq @ self.w2.T  # dh
        dh[h <= 0] = 0  # ReLU 反传
        dw1 = x.T @ dh  # dW1
        db1 = dh.sum(axis=0)  # db1
        self.w2 -= self.lr * dw2  # 更新
        self.b2 -= self.lr * db2
        self.w1 -= self.lr * dw1
        self.b1 -= self.lr * db1

    def save(self, path):
        # 存权重
        Path(path).parent.mkdir(parents=True, exist_ok=True)  # 建目录
        np.savez(path, w1=self.w1, b1=self.b1, w2=self.w2, b2=self.b2)  # 写盘

    def load(self, path):
        # 读权重
        z = np.load(path)  # 加载
        self.w1, self.b1 = z["w1"], z["b1"]  # 恢复
        self.w2, self.b2 = z["w2"], z["b2"]


def action_to_delta(a):
    # 动作编号 -> (dr,dc)
    return ACTIONS[int(a)]  # 查表


def follow_or_dqn(grid, dyn, pos, path, goal, agent, eps=0.0):
    # 混合：前方无动态则跟路径，否则 DQN（PPT 混合系统）
    look = lookahead(path, pos, 1)  # 下一步指引
    front = look  # 前方格
    if front not in dyn and front != pos:  # 前方安全且需走
        return (front[0] - pos[0], front[1] - pos[1])  # 跟 A*
    s = state_vec(grid, dyn, pos, path, goal)  # 状态
    a = agent.act(s, eps)  # DQN
    return action_to_delta(a)  # 增量
