#ifndef PATH_FINDING_VISUALIZER_ALGORITHM_REGISTRY_H
#define PATH_FINDING_VISUALIZER_ALGORITHM_REGISTRY_H

#include <functional>
#include <unordered_map>

#include "algorithm_base.h"

// 算法 Handler 构造器表格, 每新增一个算法, 需要到这里注册一下
extern std::unordered_map<std::string,
                          std::function<std::unique_ptr<Algorithm>()>>
    AlgorithmMakers;
#endif
