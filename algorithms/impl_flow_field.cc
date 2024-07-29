#include "impl_flow_field.h"

#include <spdlog/spdlog.h>

void AlgorithmImplFlowField::Setup(Blackboard &b, const Options &options) {
  // 清理黑板
  setupBlackboard(b);
  // 支持流场
  memset(b.flows, -1, sizeof b.flows);
  b.isSupportedFlowField = true;
  // 建图
  // 注意!!! NOTE: 实际应该反向建图, 但是这里是方格图,
  // 正反向是对称的 (边总是双向的).
  setupEdges(options.use_4directions);
  // 清理距离场, 到无穷大
  memset(dist, 0x3f, sizeof(dist));
  // 清理 queue
  while (q.size()) q.pop();
  // 设置目标和起始点
  s = pack(options.start);
  t = pack(options.target);
  // 重设 is_flow_calc_done 标记
  is_flow_calc_done = false;
  use_4directions = options.use_4directions;
  // 初始化目标的 dist
  dist[t] = 0;
  q.push({dist[t], t});
}

int AlgorithmImplFlowField::Update(Blackboard &b) {
  // 计算距离场: dijkstra 算法
  while (!q.empty()) {
    // q 不空, 说明还没计算完毕距离场
    auto [_, x] = q.top();
    q.pop();
    int i = unpack_i(x), j = unpack_j(x);
    b.exploring[i][j] = -1;
    if (b.visited[i][j]) continue;
    b.visited[i][j] = true;
    for (const auto &[w, y] : edges[x]) {
      if (dist[y] > dist[x] + w) {
        dist[y] = dist[x] + w;
        q.push({dist[y], y});
        b.exploring[unpack_i(y)][unpack_j(y)] = dist[y];
      }
    }
    // 每次 Update 只考察一个点, 不算结束
    return -1;
  }

  if (!is_flow_calc_done) {
    calc_flow(b);
    is_flow_calc_done = true;
    return -1;
  }

  // 收集路径
  if (b.path.empty() && find(unpack_i(s), unpack_j(s), b.path, b)) {
    b.isStopped = true;
    return 0;  // 寻路成功
  }
  return -2;  // 失败
}

void AlgorithmImplFlowField::calc_flow(Blackboard &b) {
  spdlog::info("计算流场中");
  // 计算 flow 场
  int max_directions = use_4directions ? 4 : 8;
  for (int i = 0; i < M; i++) {
    for (int j = 0; j < N; j++) {
      // 记录最小的 dist 的邻居
      int min_dist = inf;
      // 默认情况下, 方向值是 -1
      b.flows[i][j] = -1;
      // 如果当前格子是障碍物, 则不考虑流向
      if (GRID_MAP[i][j]) continue;
      for (int k = 0; k < max_directions; k++) {
        const auto &[_, d] = DIRECTIONS[k];
        auto i1 = i + d.first, j1 = j + d.second;
        if (ValidatePoint(i1, j1) and !GRID_MAP[i1][j1]) {
          // 确保邻居方格是合法的, 且不是障碍物
          int y = pack(i1, j1);
          if (min_dist > dist[y]) {
            min_dist = dist[y];
            b.flows[i][j] = k;
          }
        }
      }
    }
  }
  spdlog::info("计算流场完毕!");
}

bool AlgorithmImplFlowField::find(int i, int j, std::vector<Point> &path,
                                  const Blackboard &b) {
  P x = {i, j};
  path.push_back(x);
  // 目标
  int ti = unpack_i(t), tj = unpack_j(t);
  while (!(i == ti && j == tj)) {
    if (b.flows[i][j] == -1) return false;
    const auto &[_, d] = DIRECTIONS[b.flows[i][j]];
    i += d.first;
    j += d.second;
    x = {i, j};
    path.push_back(x);
  }
  return true;
}

void AlgorithmImplFlowField::HandleMapChanges(
    Blackboard &b, const Options &options,
    const std::vector<Point> &to_become_obstacles,
    const std::vector<Point> &to_remove_obstacles) {
  // 不支持增量计算, 只可以重新计算
  spdlog::info("flow-field 算法不支持增量计算, 将重新计算");
  Setup(b, options);
}

void AlgorithmImplFlowField::HandleStartPointChange(Blackboard &b,
                                                    const Options &options) {
  spdlog::info("flow-field 支持动态调整起始点, 无需重新计算!");
  // 重设起点
  s = pack(options.start);
  b.path.clear();
  b.isStopped = false;
  spdlog::info("flow-field 已修改完起始点");
}
