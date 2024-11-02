/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_COMMON_HPP
#define CHDR_COMMON_HPP

#include "CHDR_Version.hpp"

#include "mazes/base/IMaze.hpp"
#include "mazes/base/IGraph.hpp"
#include "mazes/Graph.hpp"
#include "mazes/Grid.hpp"

#include "solvers/base/BSolver.hpp"
#include "solvers/AStar.hpp"
#include "solvers/BStar.hpp"
#include "solvers/BFS.hpp"
#include "solvers/DFS.hpp"
#include "solvers/Dijkstra.hpp"
#include "solvers/ESMGStar.hpp"
#include "solvers/FloodFill.hpp"
#include "solvers/GBFS.hpp"
#include "solvers/GDFS.hpp"
#include "solvers/GStar.hpp"
#include "solvers/JPS.hpp"

#include "types/Coord.hpp"
#include "mazes/nodes/WeightedNode.hpp"

#include "utils/Heuristics.hpp"
#include "utils/Utils.hpp"

#endif //CHDR_COMMON_HPP