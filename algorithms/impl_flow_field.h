#ifndef PATH_FINDING_VISUALIZER_ALGORITHM_FLOW_FIELD_H
#define PATH_FINDING_VISUALIZER_ALGORITHM_FLOW_FIELD_H

#include <queue>

#include "algorithm_base.h"

// 算法实现 - FlowField
class AlgorithmImplFlowField : public AlgorithmImplGraphBase {
 public:
  void Setup(Blackboard &b, const Options &options) override;
  int Update(Blackboard &b) override;
  void HandleMapChanges(Blackboard &b, const Options &options,
                        const std::vector<Point> &to_become_obstacles,
                        const std::vector<Point> &to_remove_obstacles) override;
  void HandleStartPointChange(Blackboard &b, const Options &options) override;

 protected:
  // dijkstra 的小根堆
  std::priority_queue<P, std::vector<P>, std::greater<P>> q;
  // 距离场, 按标号
  int dist[n];
  // 流场直接写到黑板

  // 是否已经计算完毕流场
  bool is_flow_calc_done = false;

  // 是否支持四个方向
  bool use_4directions = false;

  // 寻找从点 (i,j) 出发的路径
  // 返回 false 表示失败
  bool find(int i, int j, std::vector<Point> &path, const Blackboard &b);
  // 计算流程
  void calc_flow(Blackboard &b);
};

#endif
