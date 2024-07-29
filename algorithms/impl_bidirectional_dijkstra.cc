#include "impl_bidirectional_dijkstra.h"

#include <spdlog/spdlog.h>

std::pair<int, int> AlgorithmImplBidirectionalDijkstra::extend(
    decltype(q1) &q, int f[n], int from[n], bool vis[n], bool vis_other[n],
    Blackboard &b) {
  int k = q.size();
  while (k--) {
    auto [_, x] = q.top();
    q.pop();
    if (vis[x]) continue;
    vis[x] = true;
    b.visited[unpack_i(x)][unpack_j(x)] = true;
    // 判断重合
    if (vis_other[x]) return {0, x};
    for (const auto &[w, y] : edges[x]) {
      if (f[y] > f[x] + w) {
        f[y] = f[x] + w;
        q.push({f[y], y});
        from[y] = x;
        b.exploring[unpack_i(y)][unpack_j(y)] = f[y];
      }
    }
    // 每一帧只扩展一个点
    return {-1, 0};
  }
  // 本层没有找到相遇点
  return {0, inf};
}

void AlgorithmImplBidirectionalDijkstra::Setup(Blackboard &b,
                                               const Options &options) {
  // 清理黑板
  setupBlackboard(b);
  // 建图
  setupEdges(options.use_4directions);
  // 清理 f, 到无穷大
  memset(f1, 0x3f, sizeof(f1));
  memset(f2, 0x3f, sizeof(f2));
  memset(vis1, 0, sizeof vis1);
  memset(vis2, 0, sizeof vis2);
  memset(from1, 0x3f, sizeof from1);
  memset(from2, 0x3f, sizeof from2);
  // 清理 queue
  while (q1.size()) q1.pop();
  while (q2.size()) q2.pop();
  // 设置初始坐标, 目标坐标
  s = pack(options.start);
  t = pack(options.target);
  // 初始化
  f1[s] = 0;
  f2[t] = 0;
  q1.push({f1[s], s});
  q2.push({f2[t], t});
}

int AlgorithmImplBidirectionalDijkstra ::Update(Blackboard &b) {
  // 优先扩展点更少的
  while (!q1.empty() && !q2.empty()) {
    // 优先扩展更小的
    std::pair<int, int> p;
    if (q1.size() < q2.size()) {
      // 扩展 1
      p = extend(q1, f1, from1, vis1, vis2, b);
    } else {
      // 扩展 2
      p = extend(q2, f2, from2, vis2, vis1, b);
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

void AlgorithmImplBidirectionalDijkstra::collect(int x,
                                                 std::vector<int> &path) {
  path.push_back(x);
  int y1 = x;
  while (y1 != s) {
    y1 = from1[y1];
    path.push_back(y1);
  }
  // 翻转 [x..t]
  for (int l = 0, r = path.size() - 1; l < r; l++, r--)
    std::swap(path[l], path[r]);
  // 收集 x ==> t 的路径
  int y2 = x;
  while (y2 != t) {
    y2 = from2[y2];
    path.push_back(y2);
  }
}

void AlgorithmImplBidirectionalDijkstra::HandleMapChanges(
    Blackboard &b, const Options &options,
    const std::vector<Point> &to_become_obstacles,
    const std::vector<Point> &to_remove_obstacles) {
  // dijkstra 不支持增量计算, 只可以重新计算
  spdlog::info("dijkstra 算法不支持增量计算, 将重新计算");
  Setup(b, options);
}

void AlgorithmImplBidirectionalDijkstra::HandleStartPointChange(
    Blackboard &b, const Options &options) {
  spdlog::info("dijkstra 算法不支持动态调整起始点, 将重新计算");
  Setup(b, options);
}
