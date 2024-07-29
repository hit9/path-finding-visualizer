#ifndef PATH_FINDING_VISUALIZER_ALGORITHM_H
#define PATH_FINDING_VISUALIZER_ALGORITHM_H

#include "../base.h"

// 算法实现的虚类
class Algorithm {
 public:
  // 初始化算法的准备项, 需要支持重复执行(幂等)
  virtual void Setup(Blackboard &b, const Options &options) = 0;
  // 每一帧会被调用一次, 算作走一步.
  // 在这里, Update 要把结果写到黑板上.
  // 如果已经结束, 则返回 0, 没结束返回 -1, 失败返回 -2
  // 在结束返回 0 的时候必须要保证 blackboard 上的最短路结果 path 被填写.
  virtual int Update(Blackboard &b) = 0;
  // 处理地图变化 (目前就是新增和删除障碍物点)
  // 如果算法支持增量计算, 就直接增量重新规划路径
  // 否则就需要完全重新计算, 相当于 Setup 要重新执行一次
  virtual void HandleMapChanges(
      Blackboard &b, const Options &options,
      const std::vector<Point> &to_become_obstacles,
      const std::vector<Point> &to_remove_obstacles) = 0;
  // 处理起始点变化
  virtual void HandleStartPointChange(Blackboard &b,
                                      const Options &options) = 0;
  // makes unique_ptr happy
  virtual ~Algorithm() {}
};

// 算法实现的基础类, 可选择性继承
class AlgorithmImplBase : public Algorithm {
 protected:
  // (距离 OR 边权, 节点号)
  using P = std::pair<int, int>;
  // 起始点标号 s, 结束点标号 t
  int s, t;
  // 设置(清理) 黑板, 允许重复执行
  virtual void setupBlackboard(Blackboard &b);
};

// 基于图的寻路算法的基础类, 可选择性继承
class AlgorithmImplGraphBase : public AlgorithmImplBase {
 protected:
  // edges[x] => {{w, y}}, 每一项是: {边权, 邻接点}
  std::vector<std::vector<P>> edges;
  // from[x] 保存 x 最短路的上一步由哪个节点而来
  // 默认是 inf, 如果最终算法结束仍然是 inf, 则表示算法失败
  int from[n];

  // 初始化建图, 允许重复执行
  virtual void setupEdges(bool use_4directions = false);
  // 从 from 数组反向收集最短路结果
  virtual void buildShortestPathResult(Blackboard &b);
};

#endif
