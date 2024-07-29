#ifndef PATH_FINDING_VISUALIZER_ALGORITHM_ASTAR_H
#define PATH_FINDING_VISUALIZER_ALGORITHM_ASTAR_H

#include "impl_dijkstra.h"

// 算法实现 -- A star
// 事实上, A star 是一种优化的 dijkstra
class AlgorithmImplAStar : public AlgorithmImplDijkstra {
 public:
  // Setup 其实可以直接复用 dijkstra 的
  void Setup(Blackboard &b, const Options &options) override;
  int Update(Blackboard &b) override;
  void HandleMapChanges(Blackboard &b, const Options &options,
                        const std::vector<Point> &to_become_obstacles,
                        const std::vector<Point> &to_remove_obstacles) override;
  virtual void HandleStartPointChange(Blackboard &b,
                                      const Options &options) override;

 private:
  int heuristic_weight = 1;
  int heuristic_method = 1;  // 1 曼哈顿, 2 欧式
  // 计算节点 y 到目标 t 的未来预估代价, 曼哈顿距离
  int future_cost(int y, int t);
};

#endif
