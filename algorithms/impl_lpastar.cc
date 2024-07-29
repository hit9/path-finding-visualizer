#include "impl_lpastar.h"

#include <spdlog/spdlog.h>

#include <bitset>

int AlgorithmImplLPAStar::h(int x) {
  // 方格内的坐标
  auto ti = unpack_i(t), tj = unpack_j(t);
  auto xi = unpack_i(x), xj = unpack_j(x);
  // 曼哈顿
  if (heuristic_method == 1) return (abs(ti - xi) + abs(tj - xj)) * COST_UNIT;
  // 欧式距离
  return std::floor(std::hypot(abs(ti - xi), abs(tj - xj))) * COST_UNIT;
}

AlgorithmImplLPAStar::K AlgorithmImplLPAStar::k(int x) {
  return {std::min(g[x], rhs[x]) + heuristic_weight * h(x),
          std::min(g[x], rhs[x]), x};
}

void AlgorithmImplLPAStar::init() {
  memset(g, 0x3f, sizeof g);
  memset(rhs, 0x3f, sizeof rhs);
  rhs[s] = 0;
  q.insert({k(s), s});
}

void AlgorithmImplLPAStar::update(int x) {
  // 其实点不可更新, g[s] 和 rhs[s] 恒等于 0
  if (x == s) return;
  // 从队列中删除
  q.erase(k(x));
  // 根据 x 的前继节点的实际代价 g, 加上边权 w, 取最小.
  // 来获取 x 处的新代价 rhs
  rhs[x] = inf;
  for (auto [y, w] : pred[x]) rhs[x] = std::min(rhs[x], g[y] + w);
  // 如果 x 的 g 和 rhs 没有对齐, 则重新加入队列等待更新
  if (g[x] != rhs[x]) q.insert({k(x), x});
}

int AlgorithmImplLPAStar::collect(std::vector<int> &path) {
  path.push_back(t);
  // st 用来判环, 如果检测到, 则即时终止, 以防死循环
  std::bitset<n> st;
  int x = t;
  while (x != s) {
    if (st[x]) {
      spdlog::warn(
          "得到的路径存在环, 可能启发函数设计不良存在高估, "
          "将强制继续传播一轮以恢复");
      force_stop_until_target = true;
      return -1;
    }
    st[x] = 1;
    // 找到 g + w 最小的前继邻居
    int y1 = inf;
    int g1 = inf;
    for (const auto &[y, w] : pred[x]) {
      if (g1 >= g[y] + w) {
        g1 = g[y] + w;
        y1 = y;
      }
    }
    if (y1 >= inf) break;
    x = y1;
    path.push_back(x);
  }
  // 原地反转
  for (int l = 0, r = path.size() - 1; l < r; l++, r--)
    std::swap(path[l], path[r]);
  return 0;
}

void AlgorithmImplLPAStar::add_obstacle(int i, int j) {
  int x = pack(i, j);
  // 到达 x 的边权全部无穷大
  for (auto &[y, w] : pred[x]) w = inf;

  // x 到达后继邻居的边权全部无穷大
  for (auto y : succ[x])
    for (auto &[x1, w] : pred[y])
      if (x1 == x) w = inf;

  // update x 和 后继邻居
  // 原则是:  update 边权有变化的边的末端节点
  update(x);
  for (auto y : succ[x]) update(y);
}

void AlgorithmImplLPAStar::remove_obstacle(int i, int j) {
  int x = pack(i, j);

  // 到达 x 的边权全部恢复
  for (auto &[y, w] : pred[x]) {
    int i1 = unpack_i(y), j1 = unpack_j(y);
    int di = i - i1, dj = j - j1;
    if (di != 0 && dj != 0) {  // 斜边邻居
      w = DIAGONAL_COST;
    } else {  // 水平竖直邻居
      w = COST_UNIT;
    }
  }
  // x 到达后继邻居的边权全部恢复
  for (auto y : succ[x]) {  // 对每个 x 的后继 y
    int i1 = unpack_i(y), j1 = unpack_j(y);
    int di = i - i1, dj = j - j1;
    for (auto &[x1, w] : pred[y])
      // 找到 y 的前继 x 的边权 w
      if (x1 == x) {
        if (di != 0 && dj != 0) {  // 斜边邻居
          w = DIAGONAL_COST;
        } else {  // 水平竖直邻居
          w = COST_UNIT;
        }
      }
  }
  // update x 和 后继邻居
  // 原则是:  update 边权有变化的边的末端节点
  update(x);
  for (auto y : succ[x]) update(y);
}

// 支持增量计算
void AlgorithmImplLPAStar::HandleMapChanges(
    Blackboard &b, const Options &options,
    const std::vector<Point> &to_become_obstacles,
    const std::vector<Point> &to_remove_obstacles) {
  if (to_become_obstacles.empty() && to_remove_obstacles.empty()) return;
  spdlog::info("LAPStar 支持增量计算, 将进行增量寻路修正");
  // 清理一下黑板  (以完全重新渲染)
  setupBlackboard(b);
  // 新增障碍物
  for (const auto &p : to_become_obstacles) {
    add_obstacle(p.first, p.second);
  }
  // 移除障碍物
  for (const auto &p : to_remove_obstacles) {
    remove_obstacle(p.first, p.second);
  }
  spdlog::info("LAPStar 增量修改完毕");
}

void AlgorithmImplLPAStar::HandleStartPointChange(Blackboard &b,
                                                  const Options &options) {
  spdlog::info("LAPStar 不支持动态变更起始点, 将重新计算");
  Setup(b, options);
}

void AlgorithmImplLPAStar::Setup(Blackboard &b, const Options &options) {
  heuristic_weight = options.astar_heuristic_weight;
  if (heuristic_weight > 1) {
    spdlog::warn(
        "选用 LAPStar 时的启发权重设置为 > 1, 这可能会引起代价高估, "
        "导致增量计算不充分");
  }
  if (options.astar_heuristic_method == "euclidean") {
    heuristic_method = 2;
    spdlog::info("LPAStar 选用欧式距离");
  } else {
    heuristic_method = 1;
    spdlog::info("LPAStar 选用曼哈顿距离");
    if (!options.use_4directions)
      spdlog::warn("LPAStar 在8方向上采用曼哈顿可能会引起代价高估");
  }
  // 清理黑板
  setupBlackboard(b);
  // 建图
  pred.clear();
  succ.clear();
  pred.resize(n);
  succ.resize(n);
  // 初始化 pred 和 succ
  for (int i = 0; i < M; i++) {
    for (int j = 0; j < N; j++) {
      int x = pack(i, j);
      int di_max = options.use_4directions ? 4 : 8;
      for (int di = 0; di < di_max; di++) {
        const auto &[w, d] = DIRECTIONS[di];
        auto i1 = i + d.first, j1 = j + d.second;
        auto y = pack(i1, j1);
        if (ValidatePoint(i1, j1)) {
          // 现在考虑的边 (x => y)
          // 后继
          succ[x].push_back(y);
          // 前继, 从障碍物出发或者到达障碍物都算作无穷大的边权
          pred[y].push_back(
              {x, (GRID_MAP[i][j] || GRID_MAP[i1][j1]) ? inf : w});
        }
      }
    }
  }

  // 清理 q
  q.clear();
  // 设置初始坐标
  s = pack(options.start);
  // 设置结束点
  t = pack(options.target);
  // 初始化
  init();
}

int AlgorithmImplLPAStar::Update(Blackboard &b) {
  while (q.size()) {
    auto it = q.begin();
    if ((!force_stop_until_target) && it->first >= k(t) && rhs[t] == g[t])
      break;

    // 弹出队头
    int x = it->second;
    q.erase(it);

    int i = unpack_i(x), j = unpack_j(x);
    b.visited[unpack_i(x)][unpack_j(x)] = true;

    if (g[x] > rhs[x]) {
      // 局部过一致
      g[x] = rhs[x];
    } else {
      // 局部欠一致, 通常说明新增了障碍物
      g[x] = inf;
      update(x);
    }
    // 向后继节点传播, 更新邻域
    for (auto y : succ[x]) {
      update(y);
      b.exploring[unpack_i(y)][unpack_j(y)] = g[y];
    }

    // 无论如何, 已经扩散到目标, 都可以终止
    if (x == t) break;

    // 每一次 Update 都只考察一个点, 不算结束
    return -1;
  }

  // 已经结束, 需要计算最短路
  if (g[t] >= inf) return -2;  // 失败

  std::vector<int> path;
  if (collect(path) != 0)
    // 收集失败, 继续重试 Update
    return -1;
  // 收集成功, 重置 force_stop_until_target
  force_stop_until_target = false;
  // 输出到 blackboard
  b.path.clear();
  for (auto x : path) b.path.push_back({unpack_i(x), unpack_j(x)});
  b.isStopped = true;
  return 0;
}
