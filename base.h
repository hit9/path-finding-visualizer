#ifndef PATH_FINDING_VISUALIZER_BASE_H
#define PATH_FINDING_VISUALIZER_BASE_H

#include <string>
#include <vector>

// 坐标 (i, j)
using Point = std::pair<int, int>;

// 全局设置
const int GRID_SIZE = 40;                 // 绘制每个网格 Pixel 的个数
const int M = 12;                         // 方格行数, 迭代变量 i
const int N = 15;                         // 方格列数, 迭代变量 j
const int WINDOW_HEIGHT = M * GRID_SIZE;  // 窗口高度 600
const int WINDOW_WIDTH = N * GRID_SIZE;   // 窗口宽度 800
const int COST_UNIT = 10;  // 默认边长 10, 即代价单位, 水平和垂直方向代价
const int DIAGONAL_COST = 14;  // 对角成本是 14 (根号2 x 10)
const int inf = 0x3f3f3f3f;

// 网格地图: 0 表示空白方格 (白色), 1 表示有障碍物 (灰色)
extern int GRID_MAP[M][N];

// 记录下被修改过的位置(只为渲染), 主要 for Visualizer
extern bool CHANGED_GRIDS[M][N];

// 方向 和 成本
const std::pair<int, std::pair<int, int>> DIRECTIONS[8] = {
    // 前 4 个是水平和竖直
    {COST_UNIT, {0, 1}},   // 右
    {COST_UNIT, {0, -1}},  // 左
    {COST_UNIT, {-1, 0}},  // 上
    {COST_UNIT, {1, 0}},   // 下
    // 后 4 个是斜向
    {DIAGONAL_COST, {-1, -1}},  // 左上
    {DIAGONAL_COST, {1, -1}},   // 左下
    {DIAGONAL_COST, {-1, 1}},   // 右上
    {DIAGONAL_COST, {1, 1}},    // 右下
};

// 字体 Arrow 中的字符
const char DIRECTIONS_CHAR[9] = "ABCDEFGH";

// 命令行选项
struct Options {
  // 地图文件地址
  std::string map_file_path = "map.txt";
  // 是否启用每一步的窗口截图, 保存在当前目录下的 screenshot 目录中
  bool enable_screenshot = false;
  // 保存窗口截图的目录, 文件的保存格式是 "{step-number}.PNG"
  // 不带 / 结尾
  std::string screenshot_directory = "screenshots";
  // 两帧之间的时间间隔
  int delay_ms = 50;
  // 要演示的算法
  std::string algorithm = "dijkstra";
  // 起始点, 终点
  Point start = {0, 0}, target = {M - 1, N - 1};
  // 是否只采用 4 方向, 默认是 8 方向
  bool use_4directions = false;
  // astar/lpastar 的启发式权重, 默认是 1 倍权重, 0 时退化到 dijkstra
  int astar_heuristic_weight = 1;
  // astar 的启发式方法, 可选两种: 曼哈顿距离 'manhattan' 和 欧式距离
  // 'euclidean' 对于 4 方向, 默认是曼哈顿; 对于 8 方向默认是欧式
  std::string astar_heuristic_method = "";
};

// 黑板, 算法实现者要把寻路中的数据写到这里, Visualizer
// 可视化器会从这个黑板上去读.
struct Blackboard {
  // 是否计算结束? 算法结束时需要维护这个字段.
  bool isStopped = false;
  // 历史考察过的点, 即 访问数组
  // 有的也叫做 closed_set
  bool visited[M][N];
  // 当前候选的待扩展的点的代价值
  // 不在待扩展列表中的, 标记 -1
  // 有的也叫做 open_set
  int exploring[M][N];
  // 从出发到目标的一条最短路径 (包含 start 和 target)
  std::vector<Point> path;
  // 是否支持 flow 流场展示?
  bool isSupportedFlowField = false;
  // 如果支持流场展示的话, 这里设置方向标号
  // 设置为 -1 表示没有流
  int flows[M][N];
};

// 加载地图, 成功则返回 0
int LoadMap(const std::string &filepath);
// 解析命令行参数, 成功返回 0
int ParseOptionsFromCommandline(int argc, char *argv[], Options &options);

// Util 函数

// 一个编码规则 i*N+j => 标号, 注意, 因为我们这里 N 比 M 大, 所以采用 N
inline int pack(int i, int j) { return i * N + j; }
inline int pack(const Point &p) { return p.first * N + p.second; }
inline int unpack_i(int x) { return x / N; }
inline int unpack_j(int x) { return x % N; }

static const int n = M * N;  // 总的节点数量

// 一个切割类似 "x,y" 的字符串到 Point 的 util 函数
Point ParsePointString(const std::string &s);

// 检查点是否在地图中
bool ValidatePoint(const Point &p);
bool ValidatePoint(int x, int y);

// 检查选项 start 和 target, 成功返回 0
int ValidateStartAndTarget(const Options &options);

#endif
