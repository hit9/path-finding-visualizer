#ifndef PATH_FINDING_VISUALIZER_ALGORITHM_ASTAR_BI_H
#define PATH_FINDING_VISUALIZER_ALGORITHM_ASTAR_BI_H

#include "impl_bidirectional_dijkstra.h"

// -- 算法 双向 astar
class AlgorithmImplBidirectionalAStar
    : public AlgorithmImplBidirectionalDijkstra {
  void Setup(Blackboard &b, const Options &options) override;
  int Update(Blackboard &b) override;
  void HandleMapChanges(Blackboard &b, const Options &options,
                        const std::vector<Point> &to_become_obstacles,
                        const std::vector<Point> &to_remove_obstacles) override;
  void HandleStartPointChange(Blackboard &b, const Options &options) override;

 protected:
  int heuristic_weight = 1;
  int heuristic_method = 1;  // 1 曼哈顿, 2 欧式

  // 扩展一次队列 q (是扩展一层)
  // vis 是自己的访问数组, vis_other 是对方的访问数组
  // t 是本次搜索的模板
  // 如果出现重合, 代表可以搜索结束,返回 {0, 相遇点}, 如果没有相遇点, 返回 {0,
  // inf}; 如果仍未结束, 返回 {-1, anything}
  std::pair<int, int> extend(decltype(q1) &q, int f[n], int from[n],
                             bool vis[n], bool vis_other[n], int t,
                             Blackboard &b);
  // 代价估算的启发式函数
  // 反向和正向的时候传入的目标不一样
  int future_cost(int x, int t);
};

#endif
