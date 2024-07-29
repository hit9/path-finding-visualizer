#include "impl_astar.h"

#include <spdlog/spdlog.h>

void AlgorithmImplAStar::Setup(Blackboard &b, const Options &options) {
  heuristic_weight = options.astar_heuristic_weight;
  if (options.astar_heuristic_method == "euclidean") {
    heuristic_method = 2;
    spdlog::info("astar 选用欧式距离");
  } else {
    heuristic_method = 1;
    spdlog::info("astar 选用曼哈顿距离");
  }
  // 复用 dijkstra 的 Setup 即可
  AlgorithmImplDijkstra::Setup(b, options);
}

int AlgorithmImplAStar ::Update(Blackboard &b) {
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
      auto g = f[x] + w;           // s 到 y 的实际代价
      auto h = future_cost(y, t);  // y 到目标的未来代价的估计
      auto cost = g + heuristic_weight * h;  // 总代价 = 实际 + 权重*未来
      if (f[y] > g) {  // 如果当前实际代价比之前计算的更优
        f[y] = g;      // 维护 y 的实际代价
        b.exploring[unpack_i(y)][unpack_j(y)] = f[y];
        q.push({cost, y});
        from[y] = x;  // 最短路来源
      }
    }
    return -1;
  }
  // 每次 Update 只考察一个点, 不算结束
  if (from[t] == inf) return -2;  // 失败
  // 已经结束,需要计算最短路
  buildShortestPathResult(b);
  return 0;
}

int AlgorithmImplAStar::future_cost(int y, int t) {
  auto ti = unpack_i(t), tj = unpack_j(t);
  // y 的坐标
  auto yi = unpack_i(y), yj = unpack_j(y);
  // 注意乘以 10
  // 对于 y 的未来代价预估, 曼哈顿距离
  if (heuristic_method == 1) return (abs(ti - yi) + abs(tj - yj)) * COST_UNIT;
  // 欧式距离
  return std::floor(std::hypot(abs(ti - yi), abs(tj - yj))) * COST_UNIT;
}

void AlgorithmImplAStar::HandleMapChanges(
    Blackboard &b, const Options &options,
    const std::vector<Point> &to_become_obstacles,
    const std::vector<Point> &to_remove_obstacles) {
  if (to_become_obstacles.empty() && to_remove_obstacles.empty()) return;
  // astar 不支持增量计算, 只可以重新计算
  spdlog::info("astar 算法不支持增量计算, 将重新计算");
  Setup(b, options);
}

void AlgorithmImplAStar::HandleStartPointChange(Blackboard &b,
                                                const Options &options) {
  // astar 不支持增量计算, 只可以重新计算
  spdlog::info("astar 算法不支持增量计算, 将重新计算");
  Setup(b, options);
}
