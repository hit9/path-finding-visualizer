#ifndef PATH_FINDING_VISUALIZER_ALGORITHM_GREEDY_H
#define PATH_FINDING_VISUALIZER_ALGORITHM_GREEDY_H

#include <queue>

#include "algorithm_base.h"

// 算法实现 -- Greedy  贪心
// 严格来说, 贪心的方法不会计算出来最短路, 但是也算作一种寻路方法,
// 作为演示对比目的存在
class AlgorithmImplGreedy : public AlgorithmImplGraphBase {
 public:
  void Setup(Blackboard &b, const Options &options) override;
  int Update(Blackboard &b) override;
  void HandleMapChanges(Blackboard &b, const Options &options,
                        const std::vector<Point> &to_become_obstacles,
                        const std::vector<Point> &to_remove_obstacles) override;
  virtual void HandleStartPointChange(Blackboard &b,
                                      const Options &options) override;

 private:
  int heuristic_method = 1;  // 1 曼哈顿, 2 欧式
  // 小根堆, 实际是按 cost 进行比较
  std::priority_queue<P, std::vector<P>, std::greater<P>> q;
  // f[x] 保存出发点 s 到 x 的最短路
  int f[n];
  // 计算节点 y 到目标 t 的未来预估代价, 曼哈顿距离
  int future_cost(int y, int t);
};

#endif
