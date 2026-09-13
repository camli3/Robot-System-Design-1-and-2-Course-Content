# A*：f = g + h，记录逐步扩展过程
import heapq  # 优先队列
from env.grid_map import neighbors4, manhattan  # 邻居与启发


def astar(grid, start, goal):
    # 返回路径、扩展顺序、关闭集
    open_heap = []  # (f, 计数, 节点)
    count = 0  # 打破并列
    g = {start: 0}  # 起点代价
    parent = {start: None}  # 回溯
    heapq.heappush(open_heap, (manhattan(start, goal), count, start))  # 入堆
    in_open = {start}  # 开放集
    closed = set()  # 关闭集
    expanded = []  # 扩展顺序（动画用）

    while open_heap:  # 未空
        _, _, cur = heapq.heappop(open_heap)  # 取 f 最小
        if cur in closed:  # 过时项
            continue  # 跳过
        in_open.discard(cur)  # 移出开放
        closed.add(cur)  # 关闭
        expanded.append(cur)  # 记录
        if cur == goal:  # 到达
            break  # 结束
        for nxt in neighbors4(grid, cur[0], cur[1]):  # 四邻
            ng = g[cur] + 1  # 新 g
            if nxt in closed:  # 已关闭
                continue  # 跳过
            if nxt not in g or ng < g[nxt]:  # 更好
                g[nxt] = ng  # 更新 g
                parent[nxt] = cur  # 记父
                count += 1  # 计数
                f = ng + manhattan(nxt, goal)  # f=g+h
                heapq.heappush(open_heap, (f, count, nxt))  # 入堆
                in_open.add(nxt)  # 开放

    path = []  # 路径
    if goal in parent or goal == start:  # 可达
        node = goal  # 从终点回溯
        while node is not None:  # 直到起点
            path.append(node)  # 收集
            node = parent.get(node)  # 父节点
        path.reverse()  # 正序
    return path, expanded, closed  # 三件套
