#include <spdlog/spdlog.h>

#include <argparse/argparse.hpp>

#include "algorithms/registry.h"
#include "base.h"
#include "visualizer.h"

int main(int argc, char *argv[]) {
  // 解析命令行参数到给定的 options 结构.
  Options options;
  if (ParseOptionsFromCommandline(argc, argv, options) != 0) std::exit(1);

  // 加载地图
  if (LoadMap(options.map_file_path) != 0) return -1;
  spdlog::info("地图加载成功 ({})", options.map_file_path);

  // 结合地图检查下 起始点 和 终点
  if (ValidateStartAndTarget(options) != 0) return -1;
  spdlog::info("支持的方向数量 => {}", options.use_4directions ? 4 : 8);

  // 选用算法
  if (AlgorithmMakers.find(options.algorithm) == AlgorithmMakers.end()) {
    spdlog::error("找不到算法实现:  {}", options.algorithm);
    std::exit(1);
  }

  // 构造 algorithm handler
  auto algo = AlgorithmMakers[options.algorithm]();
  spdlog::info("选用了算法 {}", options.algorithm);

  // 构造 Visualizer
  Blackboard b;
  Visualizer visualizer(options, b, algo.get());

  // 启动 Visualizer
  if (visualizer.Init() != 0) return -1;
  visualizer.Start();
  visualizer.Destroy();
  return 0;
}
