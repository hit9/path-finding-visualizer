#ifndef PATH_FINDING_VISUALIZER_ALGORITHM_DIJKSTRA_H
#define PATH_FINDING_VISUALIZER_ALGORITHM_DIJKSTRA_H

#include <queue>

#include "algorithm_base.h"

class AlgorithmImplDijkstra : public AlgorithmImplGraphBase {
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
  // 小根堆, 实际是按第一项 f[y] 作为比较
  std::priority_queue<P, std::vector<P>, std::greater<P>> q;
  // f[x] 保存出发点 s 到 x 的最短路
  int f[n];
};

#endif
