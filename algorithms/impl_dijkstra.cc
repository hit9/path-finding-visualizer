#include "impl_dijkstra.h"

#include <spdlog/spdlog.h>

void AlgorithmImplDijkstra::Setup(Blackboard &b, const Options &options) {
  // 清理黑板
  setupBlackboard(b);
  // 建图
  setupEdges(options.use_4directions);
  // 清理 f, 到无穷大
  memset(f, 0x3f, sizeof(f));
  // 清理 queue
  while (q.size()) q.pop();
  // 设置初始坐标 (or重设)
  s = pack(options.start);
  f[s] = 0;
  from[s] = s;
  q.push({f[s], s});
  // 设置结束
  t = pack(options.target);
}

int AlgorithmImplDijkstra ::Update(Blackboard &b) {
  while (!q.empty()) {
    auto [_, x] = q.top();
    q.pop();
    // 访问过的要忽略
    int i = unpack_i(x), j = unpack_j(x);
    // x 已经不算待扩展了, 恢复到 -1
    b.exploring[i][j] = -1;
    if (b.visited[i][j]) continue;
    b.visited[i][j] = true;
    // 到达目标, 及时退出 (将 return 0)
    if (t == x) break;
    // 添加邻居节点进入待扩展
    for (const auto &[w, y] : edges[x]) {
      if (f[y] > f[x] + w) {
        f[y] = f[x] + w;
        from[y] = x;
        q.push({f[y], y});
        b.exploring[unpack_i(y)][unpack_j(y)] = f[y];
      }
    }
    // 每次 Update 只考察一个点, 不算结束
    return -1;
  }
  // 已经结束,需要计算最短路
  if (from[t] == inf) return -2;  // 失败
  buildShortestPathResult(b);
  return 0;
}

void AlgorithmImplDijkstra::HandleMapChanges(
    Blackboard &b, const Options &options,
    const std::vector<Point> &to_become_obstacles,
    const std::vector<Point> &to_remove_obstacles) {
  if (to_become_obstacles.empty() && to_remove_obstacles.empty()) return;
  // dijkstra 不支持增量计算, 只可以重新计算
  spdlog::info("dijkstra 算法不支持增量计算, 将重新计算");
  Setup(b, options);
}

void AlgorithmImplDijkstra::HandleStartPointChange(Blackboard &b,
                                                   const Options &options) {
  spdlog::info("dijkstra 算法不支持动态变更起始点, 将重新计算");
  Setup(b, options);
}
