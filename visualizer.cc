#include "visualizer.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_ttf.h>
#include <spdlog/spdlog.h>

#include "base.h"

Visualizer::Visualizer(Options &options, Blackboard &b, Algorithm *algo)
    : options(options),
      blackboard(b),
      algo(algo),
      enable_screenshot(options.enable_screenshot) {
  memset(shortest_grids, 0, sizeof(shortest_grids));
}

int Visualizer::Init() {
  memset(CHANGED_GRIDS, 0, sizeof CHANGED_GRIDS);

  // 初始化 SDL
  if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
    spdlog::error("SDL 初始化错误 {}", SDL_GetError());
    return -1;
  }
  // 初始化 SDL_image
  if (!(IMG_Init(IMG_INIT_PNG) & IMG_INIT_PNG)) {
    spdlog::error("SDL_image 初始化错误: {}", IMG_GetError());
    SDL_Quit();
    return -2;
  }
  // 初始化 font
  if (TTF_Init() == -1) {
    spdlog::error("SDL_ttf 初始化错误: {}", SDL_GetError());
    SDL_Quit();
    return -1;
  }
  arrow_font = TTF_OpenFont("fonts/Arrows.ttf", 21);  // TODO: from commandline
  if (arrow_font == nullptr) {
    spdlog::error("无法打开字体 Arrows.ttf: {}", SDL_GetError());
    TTF_Quit();
    IMG_Quit();
    SDL_Quit();
    return -1;
  }

  // 创建 Arrows 的 texture
  SDL_Surface *ts =
      TTF_RenderUTF8_Solid(arrow_font, DIRECTIONS_CHAR, {0, 0, 0, 255});
  if (!ts) {
    spdlog::error("无法创建字体 SDL_Surface: {}", TTF_GetError());
    TTF_CloseFont(arrow_font);
    TTF_Quit();
    IMG_Quit();
    SDL_Quit();
    return -1;
  }

  // 创建窗口
  window = SDL_CreateWindow("shortest-path-visulization-sdl",
                            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                            WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
  if (window == nullptr) {
    spdlog::error("无法创建窗口: {}", SDL_GetError());
    SDL_DestroyTexture(arrow_texture);
    SDL_FreeSurface(ts);
    TTF_CloseFont(arrow_font);
    TTF_Quit();
    IMG_Quit();
    SDL_Quit();
    return -3;
  }

  // 创建渲染器
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
  if (renderer == nullptr) {
    spdlog::error("创建渲染器失败: {}", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_FreeSurface(ts);
    TTF_CloseFont(arrow_font);
    TTF_Quit();
    IMG_Quit();
    SDL_Quit();
    return -1;
  }

  arrow_texture = SDL_CreateTextureFromSurface(renderer, ts);
  if (arrow_texture == nullptr) {
    spdlog::error("无法创建箭头的 texture: {}", SDL_GetError());
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_FreeSurface(ts);
    TTF_CloseFont(arrow_font);
    TTF_Quit();
    IMG_Quit();
    SDL_Quit();
    return -1;
  }
  SDL_FreeSurface(ts);

  // 计算每个箭头的宽度
  int offset = 0;
  for (int i = 0; i < 8; i++) {
    int minx, maxx, miny, maxy, advance;
    TTF_GlyphMetrics(arrow_font, DIRECTIONS_CHAR[i], &minx, &maxx, &miny, &maxy,
                     &advance);
    arrow_w[i] = advance;
    arrow_offset[i] = offset;
    offset += advance;
  }
  arrow_h = TTF_FontHeight(arrow_font);

  spdlog::info("初始化 SDL 成功");

  // 初始化算法设置
  algo->Setup(blackboard, options);

  spdlog::info("初始化算法成功");
  return 0;
}

void Visualizer::Destroy() {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_DestroyTexture(arrow_texture);
  TTF_CloseFont(arrow_font);
  TTF_Quit();
  IMG_Quit();
  SDL_Quit();
}

void Visualizer::Start() {
  while (true) {
    // 处理输入
    if (handleInputs() == -1) break;

    // 更新算法步骤
    seq++;

    // 地图是否有变化?
    handleMapChanges();
    // 起始点是否有变化?
    handleStartPointChange();

    // 继续进行一步 Update
    int code = -1;
    if (!blackboard.isStopped) {  // 只有没有结束时才执行一次 Update
      code = algo->Update(blackboard);
      if (code == 0) {
        spdlog::info("算法已结束, Ctrl-C 即可退出");
      } else if (code == -2) {
        spdlog::info("算法已失败, Ctrl-C 即可退出");
      }
    } else {
      // 否则, 需要设定当前需要绘制的最短路径点
      handleShortestPathPalyStates();
    }
    // 清理屏幕,并绘制一次
    // 背景颜色: 白色
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    draw();
    SDL_RenderPresent(renderer);
    if ((code == -2 || is_shortest_path_ever_rendered) && enable_screenshot) {
      enable_screenshot = false;
      spdlog::info("算法已结束, 已关闭自动截图");
    } else if ((!enable_screenshot && options.enable_screenshot) &&
               !is_shortest_path_ever_rendered) {
      // 自动恢复录制
      enable_screenshot = true;
      spdlog::info("自动恢复截图");
    }
    // 自动截图一次
    if (enable_screenshot) {
      saveScreenShot();
    }
    // 睡眠 (不严格的 delay)
    SDL_Delay(options.delay_ms);
  }
}

void Visualizer::handleMapChanges() {
  for (const auto &[i, j] : to_become_obstacles) {
    GRID_MAP[i][j] = 1;
    CHANGED_GRIDS[i][j] ^= 1;  // 两次修改相当于没修改
  }
  for (const auto &[i, j] : to_remove_obstacles) {
    GRID_MAP[i][j] = 0;
    CHANGED_GRIDS[i][j] ^= 1;  // 两次修改相当于没修改
  }
  if (to_become_obstacles.size() > 0 || to_remove_obstacles.size() > 0) {
    algo->HandleMapChanges(blackboard, options, to_become_obstacles,
                           to_remove_obstacles);
    // 注意清理当前最短路的播放
    memset(shortest_grids, 0, sizeof shortest_grids);
    shortest_grid_no = 0;
    is_shortest_path_ever_rendered = false;
  }
  to_become_obstacles.clear();
  to_remove_obstacles.clear();
}

void Visualizer::handleStartPointChange() {
  if (new_start.first >= 0 && new_start.second >= 0) {
    // 需要设置
    options.start = new_start;
    new_start = {-1, -1};
    algo->HandleStartPointChange(blackboard, options);
    // 注意清理当前最短路的播放
    memset(shortest_grids, 0, sizeof shortest_grids);
    shortest_grid_no = 0;
    is_shortest_path_ever_rendered = false;
  }
}

int Visualizer::handleInputs() {
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_QUIT:
        return -1;
      case SDL_KEYDOWN:
        // ESC 退出
        if (e.key.keysym.sym == SDLK_ESCAPE) {
          spdlog::info("监听到 ESC : 即将退出...");
          return -1;
        }
        // Ctrl-C 退出
        if (e.key.keysym.sym == SDLK_c && SDL_GetModState() & KMOD_CTRL) {
          spdlog::info("监听到 Ctrl-C : 即将退出...");
          return -1;
        }
        // Ctrl-S 手动截图一次
        if (e.key.keysym.sym == SDLK_s && SDL_GetModState() & KMOD_CTRL) {
          spdlog::info("监听到 Ctrl-S : 即将截图一次...");
          saveScreenShot();
        }
        break;
      case SDL_MOUSEBUTTONDOWN:
        // 单击翻转地图元素, 新增或者删除一个障碍物, 更新地图
        if (e.button.button == SDL_BUTTON_LEFT) {
          Point p{e.button.y / GRID_SIZE, e.button.x / GRID_SIZE};
          int flag = 0;                       // for 日志
          if (GRID_MAP[p.first][p.second]) {  // 消除障碍物
            to_remove_obstacles.push_back(p);
          } else {  // 新增障碍物
            to_become_obstacles.push_back(p);
            flag = 1;
          }
          spdlog::info("监听到鼠标左键点击 {},{}, {}一个障碍物", p.first,
                       p.second, flag ? "新增" : "消除");
        }
        if (e.button.button == SDL_BUTTON_RIGHT) {
          Point p{e.button.y / GRID_SIZE, e.button.x / GRID_SIZE};
          if (p != options.start) {
            spdlog::info("监听到鼠标右键点击 {},{}, 变更起始点", p.first,
                         p.second);
            new_start = p;
          }
        }
        break;
    }
  }
  return 0;
}

void Visualizer::draw() {
  // 绘制方格 (x,y) 表示绘制坐标, (i, j) 表示方格坐标
  for (int i = 0, y = 0; i < M; i++, y += GRID_SIZE) {
    for (int j = 0, x = 0; j < N; j++, x += GRID_SIZE) {
      // 正方形 rect, 内侧是 inner (边框宽度 1)
      SDL_Rect rect = {x, y, GRID_SIZE, GRID_SIZE};
      SDL_Rect inner = {x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2};
      // 绘制外层正方形, 边框是黑色
      SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
      // 对于修改过的正方形, 边框是红色
      if (CHANGED_GRIDS[i][j]) SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
      SDL_RenderDrawRect(renderer, &rect);
      // 选用内层正方形的填充颜色
      if (GRID_MAP[i][j] == 1)
        // 障碍物: 灰色
        SDL_SetRenderDrawColor(renderer, 64, 64, 64, 255);
      else if ((i == options.start.first && j == options.start.second) ||
               (i == options.target.first && j == options.target.second))
        // 起始点: 绿色
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
      else if (shortest_grids[i][j])
        // 最短路径点: 绿色
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
      else if (blackboard.visited[i][j])
        // 访问过的路径点: 蓝色
        SDL_SetRenderDrawColor(renderer, 0, 150, 255, 255);
      else if (blackboard.exploring[i][j] >= 0)
        // 将要扩展的点: 浅蓝色
        SDL_SetRenderDrawColor(renderer, 173, 216, 230, 255);
      else
        // 默认是白色
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
      // 绘制内侧正方形
      SDL_RenderFillRect(renderer, &inner);
      // 如果支持流场箭头展示. 背景色不变, 黑色字体
      if (blackboard.isSupportedFlowField) {
        auto flow = blackboard.flows[i][j];
        if (0 <= flow && flow < 8) {
          // 方格的中心位置
          int x1 = x + GRID_SIZE / 2, y1 = y + GRID_SIZE / 2;
          // 获取字符宽度 和 宽度
          int w = arrow_w[flow], h = arrow_h, offset = arrow_offset[flow];
          SDL_Rect dst = {x1 - w / 2, y1 - h / 2, w, h};
          SDL_Rect src = {offset, 0, w, h};
          // 目标位置
          SDL_RenderCopy(renderer, arrow_texture, &src, &dst);
        }
      }
    }
  }
}

void Visualizer::handleShortestPathPalyStates() {
  if (blackboard.path.empty()) return;
  // 播放到下一个路径点, 到尾部则循环
  if (shortest_grid_no == blackboard.path.size() - 1) {
    shortest_grid_no = 0;
    memset(shortest_grids, 0, sizeof shortest_grids);
    is_shortest_path_ever_rendered = true;
  } else {
    shortest_grid_no++;
  }
  auto &p = blackboard.path[shortest_grid_no];
  shortest_grids[p.first][p.second] = true;
}

void Visualizer::saveScreenShot() {
  SDL_Surface *saveSurface = nullptr;
  SDL_Surface *infoSurface = SDL_GetWindowSurface(window);
  if (infoSurface == nullptr) {
    spdlog::warn("saveScreenShot: 创建 surface 失败 {}", SDL_GetError());
    return;
  }

  auto *pixels = new unsigned char[infoSurface->w * infoSurface->h *
                                   infoSurface->format->BytesPerPixel];
  if (pixels == nullptr) {
    spdlog::warn(
        "saveScreenShot: Unable to allocate memory for screenshot pixel data "
        "buffer!");
    return;
  }

  if (SDL_RenderReadPixels(
          renderer, nullptr, infoSurface->format->format, pixels,
          infoSurface->w * infoSurface->format->BytesPerPixel) != 0) {
    spdlog::warn("saveScreenShot: 无法从 renderer 读取像素数据 {}",
                 SDL_GetError());
    delete[] pixels;
    return;
  }

  saveSurface = SDL_CreateRGBSurfaceFrom(
      pixels, infoSurface->w, infoSurface->h, infoSurface->format->BitsPerPixel,
      infoSurface->w * infoSurface->format->BytesPerPixel,
      infoSurface->format->Rmask, infoSurface->format->Gmask,
      infoSurface->format->Bmask, infoSurface->format->Amask);
  if (saveSurface == nullptr) {
    spdlog::warn("saveScreenShot: 无法创建 saveSurface: {}", SDL_GetError());
    delete[] pixels;
    return;
  }

  auto filename =
      options.screenshot_directory + "/" + std::to_string(seq) + ".PNG";

  if (IMG_SavePNG(saveSurface, filename.c_str()) != 0) {
    spdlog::warn("saveScreenShot: 保存截图失败 {} => {}", filename,
                 IMG_GetError());
    SDL_FreeSurface(saveSurface);
    delete[] pixels;
    return;
  }

  spdlog::info("saveScreenShot: 保存截图成功 => {}", filename);

  SDL_FreeSurface(saveSurface);
  delete[] pixels;
  return;
}
