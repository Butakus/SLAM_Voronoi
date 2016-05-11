#ifndef ASTAR_H
#define ASTAR_H

#include "Node.h"
#include "Heuristics.h"
#include <set>
#include <list>
#include <iterator>
#include <tuple>

std::tuple<std::list<Node*>, std::vector<Node*>, std::vector<Node*>> calculateAStar(Node*, Node*,std::function<float(Node*, Node*)>);

#endif
