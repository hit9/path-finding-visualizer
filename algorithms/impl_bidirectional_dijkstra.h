#ifndef PATH_FINDING_VISUALIZER_ALGORITHM_DIJKSTRA_BI_H
#define PATH_FINDING_VISUALIZER_ALGORITHM_DIJKSTRA_BI_H

#include <queue>

#include "algorithm_base.h"

// -- 双向 Dijkstra 算法
class AlgorithmImplBidirectionalDijkstra : public AlgorithmImplGraphBase {
 public:
  virtual void Setup(Blackboard &b, const Options &options) override;
  virtual int Update(Blackboard &b) override;
  virtual void HandleMapChanges(
      Blackboard &b, const Options &options,
      const std::vector<Point> &to_become_obstacles,
      const std::vector<Point> &to_remove_obstacles) override;
  virtual void HandleStartPointChange(Blackboard &b,
                                      const Options &options) override;

 protected:
  // 1 是出发点正向, 2 是目标点反向
  std::priority_queue<P, std::vector<P>, std::greater<P>> q1, q2;
  // 最短路结果是相遇点 x 的 f1[x] + f2[x]
  int f1[n], f2[n];
  // from 保存最短路来源
  int from1[n], from2[n];
  // 访问数组
  bool vis1[n], vis2[n];

  // 扩展一次队列 q (是扩展一层)
  // vis 是自己的访问数组, vis_other 是对方的访问数组, 如果出现重合,
  // 代表可以搜索结束, 返回 {0, 相遇点}, 如果没有相遇点, 返回 {0, inf};
  // 如果仍未结束, 返回 {-1, anything}
  std::pair<int, int> extend(decltype(q1) &q, int f[n], int from[n],
                             bool vis[n], bool vis_other[n], Blackboard &b);
  // 收集路径到给的参数 path 中, 其中 x 是相遇点
  void collect(int x, std::vector<int> &path);
};

#endif
