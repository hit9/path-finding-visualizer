#ifndef PATH_FINDING_VISUALIZER_ALGORITHM_LPASTAR_H
#define PATH_FINDING_VISUALIZER_ALGORITHM_LPASTAR_H

#include <map>

#include "algorithm_base.h"

// 算法实现 - LPAstar
class AlgorithmImplLPAStar : public AlgorithmImplBase {
 public:
  void Setup(Blackboard &b, const Options &options) override;
  int Update(Blackboard &b) override;
  void HandleMapChanges(Blackboard &b, const Options &options,
                        const std::vector<Point> &to_become_obstacles,
                        const std::vector<Point> &to_remove_obstacles) override;
  void HandleStartPointChange(Blackboard &b, const Options &options) override;

 private:
  int heuristic_weight = 1;
  int heuristic_method = 1;  // 1 曼哈顿, 2 欧式
  // 默认情况下, 传播是按照估价终止的
  // 如果这个 force_stop_until_target 设置到 true
  // 那么会强制传播到目标才终止(或者q空的时候).
  // 这个用来预防不良的, 高估的估价函数 (比如8方向的情况下的曼哈顿函数)
  bool force_stop_until_target = false;
  // { 标号, 边权 }
  using P = std::pair<int, int>;
  // { 键值k1, 键值k2, 标号 }
  using K = std::tuple<int, int, int>;
  // 前继 {标号, 边权}
  std::vector<std::vector<P>> pred;
  // 后继
  std::vector<std::vector<int>> succ;

  // g 值: 起点到当前点的实际代价 (旧值)
  // rhs 值: 起点到当前点的实际代价的临时值, 由前继节点更新而来
  int g[n], rhs[n];
  // 优先级队列 (因为要支持 update 操作, 所以用 map)
  std::map<K, int> q;

  // 启发函数
  int h(int x);
  // 计算 queue 的 key 的函数
  K k(int x);
  // 初始化
  void init();
  // 更新节点
  void update(int x);
  // 收集最短路, 结果存储在入参 path (必须是空的)
  // 收集失败, 返回 -1, 否则返回 0
  int collect(std::vector<int> &path);
  // 新增障碍物
  void add_obstacle(int i, int j);
  // 清理障碍物
  void remove_obstacle(int i, int j);
};

#endif
