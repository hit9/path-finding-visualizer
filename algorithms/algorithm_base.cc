#include "algorithm_base.h"

#include <vector>

/////////////////////////////////////
/// 实现 AlgorithmImplBase
/////////////////////////////////////

void AlgorithmImplBase::setupBlackboard(Blackboard &b) {
  // 清理黑板
  b.isStopped = false;
  memset(b.visited, 0, sizeof(b.visited));
  for (int i = 0; i < M; i++)
    for (int j = 0; j < N; j++) b.exploring[i][j] = -1;
  // 默认情况下, 都不支持流场 (除了流场寻路)
  b.isSupportedFlowField = false;
  b.path.clear();
}

/////////////////////////////////////
/// 实现 AlgorithmImplGraphBase
/////////////////////////////////////

void AlgorithmImplGraphBase::setupEdges(bool use_4directions) {
  memset(from, 0x3f, sizeof(from));
  // 构造 edges
  edges.clear();
  edges.resize(n);
  // 4 方向是取前 4 个.
  int K = use_4directions ? 4 : 8;
  for (int i = 0; i < M; i++) {
    for (int j = 0; j < N; j++) {
      if (GRID_MAP[i][j])  // 不可从障碍物出发
        continue;
      int x = pack(i, j);
      // 都可以走, 除非是有障碍物
      for (int k = 0; k < K; k++) {
        const auto &[w, d] = DIRECTIONS[k];
        auto i1 = i + d.first, j1 = j + d.second;
        // 不可到达障碍物, 不可越过边界
        if (ValidatePoint(i1, j1) && !GRID_MAP[i1][j1])
          edges[x].push_back({w, pack(i1, j1)});
      }
    }
  }
}

void AlgorithmImplGraphBase::buildShortestPathResult(Blackboard &b) {
  std::vector<int> path;
  path.push_back(t);
  int x = t;
  while (x != s) {
    x = from[x];
    path.push_back(x);
  }
  // 反向求最短路
  for (int i = path.size() - 1; i >= 0; --i) {
    auto x = path[i];
    b.path.push_back({unpack_i(x), unpack_j(x)});
  }
  b.isStopped = true;
}
