#include "base.h"

#include <spdlog/spdlog.h>

#include <argparse/argparse.hpp>
#include <fstream>
#include <string>

int GRID_MAP[M][N];
bool CHANGED_GRIDS[M][N];

Point ParsePointString(const std::string &s) {
  std::string sx, sy;
  // flag 的含义: 0 时输出给 sx, 1 时输出给 sy
  int flag = 0;
  for (const auto ch : s) {
    if (ch == ',')
      flag = 1;  // 切换到输入给 sy
    else
      flag == 0 ? sx.push_back(ch) : sy.push_back(ch);
  }
  if (!flag) return {-1, -1};  // Invalid
  return {std::stoi(sx), std::stoi(sy)};
}

bool ValidatePoint(const Point &p) {
  return p.first >= 0 && p.first < M && p.second >= 0 && p.second < N;
}

bool ValidatePoint(int x, int y) { return x >= 0 && x < M && y >= 0 && y < N; }

int LoadMap(const std::string &filepath) {
  std::fstream f;
  f.open(filepath);
  std::string line;
  int x = 0;
  while (std::getline(f, line)) {
    // 检查行数
    if (x >= M) {
      spdlog::error("地图: 必须恰好 {} 行", M);
      f.close();
      return -1;
    }
    // 检查每行字符个数
    if (line.size() != N + N - 1) {  // 算进去空格
      spdlog::error(
          "地图: 每行必须是 {} 个 0 或者 1, 目前第 {} 行是 {} 个字符 "
          "(包含空格计算在内)",
          N, x, line.size());
      f.close();
      return -1;
    }
    // 读取每个字符
    int y = 0;
    for (auto ch : line) {
      if (ch == ' ') continue;
      int value = static_cast<int>(ch - '0');
      if (value != 0 && value != 1) {
        spdlog::error("地图: 每个字符要么是0要么是1, 发现了一个 '{}'", ch);
        f.close();
        return -2;
      }
      GRID_MAP[x][y++] = value;
    }
    x++;
  }
  f.close();
  return 0;
}

int ValidateStartAndTarget(const Options &options) {
  if (!ValidatePoint(options.start)) {
    spdlog::error("非法的 start {},{}", options.start.first,
                  options.start.second);
    return -1;
  }
  if (!ValidatePoint(options.target)) {
    spdlog::error("非法的 target {},{}", options.target.first,
                  options.target.second);
    return -1;
  }
  if (GRID_MAP[options.start.first][options.start.second]) {
    spdlog::error("start 选在了障碍物上");
    return -1;
  }
  if (GRID_MAP[options.target.first][options.target.second]) {
    spdlog::error("target 选在了障碍物上");
    return -1;
  }
  return 0;
}

int ParseOptionsFromCommandline(int argc, char *argv[], Options &options) {
  argparse::ArgumentParser program("shortest-path-visulization-sdl");
  program.add_argument("-m", "--map")
      .help("地图文件")
      .default_value(std::string("map.txt"))
      .store_into(options.map_file_path);
  program.add_argument("--enable-screenshot")
      .help("是否启用每一步的窗口截图")
      .default_value(false)
      .store_into(options.enable_screenshot);
  program.add_argument("--screenshot-directory")
      .help("截图文件的保存目录")
      .default_value(std::string("screenshots"))
      .store_into(options.screenshot_directory);
  program.add_argument("-d", "--delay-ms")
      .help("帧之间的毫秒间隔")
      .default_value(50)
      .store_into(options.delay_ms);
  program.add_argument("algorithm")
      .help("算法名称")
      .metavar("ALGORITHM")
      .choices("dijkstra", "astar", "lpastar", "dijkstra-bi", "astar-bi",
               "flow-field")
      .default_value(std::string("dijkstra"))
      .store_into(options.algorithm);
  program.add_argument("-d4", "--use-4-directions")
      .help("是否只采用4方向,默认是8方向")
      .default_value(false)
      .store_into(options.use_4directions);
  program.add_argument("-astar-w", "--astar-heuristic-weight")
      .help("AStar/LPAStar 算法的启发式未来估价的权重倍数, 自然数")
      .default_value(1)
      .store_into(options.astar_heuristic_weight);
  program.add_argument("-astar-m", "--astar-heuristic-method")
      .help(
          "AStar/LPAStar 算法的启发式方法, 曼哈顿 manhattan 或者 欧式距离 "
          "euclidean; 对于4方向默认是曼哈顿, "
          "8方向时默认是欧式")
      .default_value(std::string(""))
      .store_into(options.astar_heuristic_method);
  program.add_argument("-s", "--start").help("起始点").default_value("0,0");
  program.add_argument("-t", "--target").help("起始点").default_value("11,14");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &e) {
    spdlog::error(e.what());
    return 1;
  }

  // 起始点 和 终点
  options.start = ParsePointString(program.get<std::string>("--start"));
  options.target = ParsePointString(program.get<std::string>("--target"));

  // 处理启发式函数的选用
  if (options.astar_heuristic_method.empty()) {
    if (options.use_4directions) {
      spdlog::info("默认对于4方向移动时,选用曼哈顿距离作为启发函数");
      options.astar_heuristic_method = "manhattan";
    } else {
      spdlog::info("默认对于8方向移动时,选用欧式距离作为启发函数");
      options.astar_heuristic_method = "euclidean";
    }
  }
  return 0;
}
