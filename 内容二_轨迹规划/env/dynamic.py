# 动态障碍：往复 / 循环（不写入全局地图，A* 看不见）


class MovingObstacle:
    # 单个动态障碍
    def __init__(self, cells, kind="pingpong"):
        self.cells = list(cells)  # 路径格子
        if len(self.cells) < 1:  # 空路径
            raise ValueError("动态障碍路径为空")  # 报错
        self.kind = kind  # pingpong 或 loop
        self.idx = 0  # 当前下标
        self.sign = 1  # 往复方向

    def pos(self):
        # 当前格子
        return self.cells[self.idx]  # (r,c)

    def step(self, static_grid):
        # 前进一步；撞静障碍则反向
        n = len(self.cells)  # 路径长
        if n == 1:  # 定点
            return self.pos()  # 不动
        if self.kind == "loop":  # 循环
            nxt = (self.idx + 1) % n  # 下一环
        else:  # 往复
            nxt = self.idx + self.sign  # 下一步
            if nxt < 0 or nxt >= n:  # 到头
                self.sign *= -1  # 掉头
                nxt = self.idx + self.sign  # 反方向
                nxt = max(0, min(n - 1, nxt))  # 夹紧
        r, c = self.cells[nxt]  # 候选格
        h, w = static_grid.shape  # 尺寸
        if 0 <= r < h and 0 <= c < w and static_grid[r, c] == 0:  # 空闲
            self.idx = nxt  # 走上去
        elif self.kind != "loop":  # 往复撞墙
            self.sign *= -1  # 掉头
        return self.pos()  # 新位置


def build_obstacles(map_name, spec_dict):
    # 按地图名生成障碍列表
    cfgs = spec_dict.get(map_name, [])  # 该图配置
    obs = []  # 实例列表
    for cfg in cfgs:  # 每条
        obs.append(MovingObstacle(cfg["cells"], cfg["type"]))  # 创建
    return obs  # 列表


def dyn_set(obstacles):
    # 当前动态占格集合
    return {o.pos() for o in obstacles}  # set of (r,c)
