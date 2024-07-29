#ifndef PATH_FINDING_VISUALIZER_VISUALIZER_H
#define PATH_FINDING_VISUALIZER_VISUALIZER_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>

#include "algorithms/algorithm_base.h"
#include "base.h"

// 主要的程序 Visualizer
class Visualizer {
 public:
  Visualizer(Options &options, Blackboard &b, Algorithm *algo);
  // 初始化工作, 包括 SDL 的各种初始化, 需要显式调用
  // 返回 0 时表示成功
  int Init();
  // 启动主循环, 直到退出
  void Start();
  // 释放 SDL 的资源, 需要显式调用
  void Destroy();

 protected:
  // 保存一次屏幕截图
  void saveScreenShot();
  // 绘制一帧的情况
  void draw();
  // 处理输入, 返回 -1 表示要退出
  int handleInputs();
  // 处理最短路径结果的播放状态
  void handleShortestPathPalyStates();
  // 处理地图变化 (障碍物添加和清除)
  void handleMapChanges();
  // 处理起始点变更
  void handleStartPointChange();

 private:
  Options &options;
  Blackboard &blackboard;
  Algorithm *algo;
  // 拷贝一份 enable_screenshot (因为要修改)
  bool enable_screenshot = false;
  // SDL
  SDL_Window *window = nullptr;
  SDL_Renderer *renderer = nullptr;
  // SDL 加载好的 arrow font
  TTF_Font *arrow_font = nullptr;
  // 箭头的 texture
  SDL_Texture *arrow_texture;
  // 渲染时每个箭头字符的宽度 和 offset
  int arrow_w[8];
  int arrow_offset[8];
  // 字体宽度
  int arrow_h;
  // 帧号
  int seq = 0;
  // 绘制最短路时的临时状态
  // shortest_grids[i][j] 是 true 表示 (i,j) 是最短路结果的一员
  bool shortest_grids[M][N];
  // 当前播放到第几个最短路点?
  int shortest_grid_no = 0;
  // 第一次绘制完毕最短路后变为 true
  bool is_shortest_path_ever_rendered = false;
  // 需要新增成为障碍物的坐标点
  std::vector<Point> to_become_obstacles, to_remove_obstacles;
  // 将变更到的起始点
  Point new_start = {-1, -1};
};

#endif
