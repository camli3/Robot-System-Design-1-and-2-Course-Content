# Dijkstra：无启发，用于对比扩展范围
import heapq  # 优先队列
from env.grid_map import neighbors4  # 四邻


def dijkstra(grid, start, goal):
    # 返回路径、扩展顺序、关闭集
    heap = []  # (g, 计数, 节点)
    count = 0  # 并列计数
    g = {start: 0}  # 代价
    parent = {start: None}  # 父指针
    heapq.heappush(heap, (0, count, start))  # 起点
    closed = set()  # 关闭集
    expanded = []  # 扩展序

    while heap:  # 循环
        _, _, cur = heapq.heappop(heap)  # 最小 g
        if cur in closed:  # 重复
            continue  # 跳过
        closed.add(cur)  # 关闭
        expanded.append(cur)  # 记录
        if cur == goal:  # 到终点
            break  # 停
        for nxt in neighbors4(grid, cur[0], cur[1]):  # 邻居
            ng = g[cur] + 1  # 新代价
            if nxt not in g or ng < g[nxt]:  # 更优
                g[nxt] = ng  # 更新
                parent[nxt] = cur  # 父
                count += 1  # 计数
                heapq.heappush(heap, (ng, count, nxt))  # 入堆

    path = []  # 路径
    if goal in parent or goal == start:  # 可达
        node = goal  # 回溯
        while node is not None:  # 直到空
            path.append(node)  # 加点
            node = parent.get(node)  # 父
        path.reverse()  # 正序
    return path, expanded, closed  # 结果
