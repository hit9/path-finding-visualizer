#include "impl_bidirectional_astar.h"

#include <spdlog/spdlog.h>

// 和原生 A* 的完全一样
int AlgorithmImplBidirectionalAStar::future_cost(int y, int t) {
  auto ti = unpack_i(t), tj = unpack_j(t);
  // y 的坐标
  auto yi = unpack_i(y), yj = unpack_j(y);
  // 注意乘以 10
  // 对于 y 的未来代价预估, 曼哈顿距离
  if (heuristic_method == 1) return (abs(ti - yi) + abs(tj - yj)) * COST_UNIT;
  // 欧式距离
  return std::floor(std::hypot(abs(ti - yi), abs(tj - yj))) * COST_UNIT;
}

std::pair<int, int> AlgorithmImplBidirectionalAStar::extend(
    decltype(q1) &q, int f[n], int from[n], bool vis[n], bool vis_other[n],
    int t, Blackboard &b) {
  int k = q.size();
  while (k--) {
    auto [_, x] = q.top();
    q.pop();
    if (x == t) break;  // 到达目标
    if (vis[x]) continue;
    vis[x] = true;
    b.visited[unpack_i(x)][unpack_j(x)] = true;
    // 判断重合
    if (vis_other[x]) return {0, x};
    // 对于 x 的每个邻居 y 和 边权
    for (const auto &[w, y] : edges[x]) {
      auto g = f[x] + w;           // s 到 y 的实际代价
      auto h = future_cost(y, t);  // y 到目标的未来代价的估计
      auto cost = g + h;           // 总代价 = 实际 + 未来
      if (f[y] > g) {  // 如果当前实际代价比之前计算的更优
        f[y] = g;      // 维护 y 的实际代价
        q.push({cost, y});
        from[y] = x;  // 最短路来源
        b.exploring[unpack_i(y)][unpack_j(y)] = f[y];
      }
    }
    // 每一帧只扩展一个点
    return {-1, 0};
  }
  // 本轮没有相遇
  return {0, inf};
}

void AlgorithmImplBidirectionalAStar::Setup(Blackboard &b,
                                            const Options &options) {
  heuristic_weight = options.astar_heuristic_weight;
  if (options.astar_heuristic_method == "euclidean") {
    heuristic_method = 2;
    spdlog::info("astar-bi 选用欧式距离");
  } else {
    heuristic_method = 1;
    spdlog::info("astar-bi 选用曼哈顿距离");
  }
  // 复用 dijkstra 的 Setup 即可
  AlgorithmImplBidirectionalDijkstra::Setup(b, options);
}

int AlgorithmImplBidirectionalAStar ::Update(Blackboard &b) {
  while (!q1.empty() && !q2.empty()) {
    // 优先扩展更小的
    std::pair<int, int> p;
    if (q1.size() < q2.size()) {
      // 扩展 1, 走向目标 t
      p = extend(q1, f1, from1, vis1, vis2, t, b);
    } else {
      // 扩展 2, 走向模板 s
      p = extend(q2, f2, from2, vis2, vis1, s, b);
    }
    if (p.first == -1) return -1;  // 仍未结束
    int x = p.second;              // 否则, 已经结束, x 是相遇点
    if (x != inf) {
      // 寻路成功
      std::vector<int> path;
      collect(x, path);
      b.path.clear();
      for (auto y : path) b.path.push_back({unpack_i(y), unpack_j(y)});
      b.isStopped = true;
      return 0;
    }
  }
  return -2;  // 寻路失败了
}

void AlgorithmImplBidirectionalAStar::HandleMapChanges(
    Blackboard &b, const Options &options,
    const std::vector<Point> &to_become_obstacles,
    const std::vector<Point> &to_remove_obstacles) {
  // 不支持增量计算, 只可以重新计算
  spdlog::info("astar-bi 算法不支持增量计算, 将重新计算");
  Setup(b, options);
}

void AlgorithmImplBidirectionalAStar::HandleStartPointChange(
    Blackboard &b, const Options &options) {
  spdlog::info("astar-bi 算法不支持动态调整起始点, 将重新计算");
  Setup(b, options);
}
