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
#include "mazes/base/igraph.hpp"
#include "mazes/graph.hpp"
#include "mazes/Grid.hpp"

#include "solvers/base/bsolver.hpp"
#include "solvers/astar.hpp"
#include "solvers/bstar.hpp"
#include "solvers/bfs.hpp"
#include "solvers/dfs.hpp"
#include "solvers/dijkstra.hpp"
#include "solvers/esmgstar.hpp"
#include "solvers/floodfill.hpp"
#include "solvers/gbfs.hpp"
#include "solvers/gdfs.hpp"
#include "solvers/GStar.hpp"
#include "solvers/JPS.hpp"

#include "types/Coord.hpp"
#include "mazes/nodes/WeightedNode.hpp"

#include "utils/Heuristics.hpp"
#include "utils/Utils.hpp"

#endif //CHDR_COMMON_HPP