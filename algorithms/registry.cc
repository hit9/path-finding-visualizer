#include "registry.h"

#include <memory>
#include <unordered_map>

#include "impl_astar.h"
#include "impl_bidirectional_astar.h"
#include "impl_bidirectional_dijkstra.h"
#include "impl_dijkstra.h"
#include "impl_flow_field.h"
#include "impl_greedy.h"
#include "impl_lpastar.h"

decltype(AlgorithmMakers) AlgorithmMakers = {
    {"dijkstra", []() { return std::make_unique<AlgorithmImplDijkstra>(); }},
    {"greedy", []() { return std::make_unique<AlgorithmImplGreedy>(); }},
    {"astar", []() { return std::make_unique<AlgorithmImplAStar>(); }},
    {"lpastar", []() { return std::make_unique<AlgorithmImplLPAStar>(); }},
    {"dijkstra-bi",
     []() { return std::make_unique<AlgorithmImplBidirectionalDijkstra>(); }},
    {"astar-bi",
     []() { return std::make_unique<AlgorithmImplBidirectionalAStar>(); }},
    {"flow-field", []() { return std::make_unique<AlgorithmImplFlowField>(); }},
};
