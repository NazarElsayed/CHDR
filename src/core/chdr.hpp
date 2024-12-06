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

#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "mazes/base/igraph.hpp"
#include "mazes/base/imaze.hpp"
#include "mazes/nodes/weighted_node.hpp"

#include "solvers/astar.hpp"
#include "solvers/bfs.hpp"
#include "solvers/bstar.hpp"
#include "solvers/dfs.hpp"
#include "solvers/dijkstra.hpp"
#include "solvers/esmgstar.hpp"
#include "solvers/floodfill.hpp"
#include "solvers/gbfs.hpp"
#include "solvers/gdfs.hpp"
#include "solvers/gstar.hpp"
#include "solvers/jps.hpp"
#include "solvers/base/bsolver.hpp"

#include "types/coord.hpp"

#include "utils/heuristics.hpp"
#include "utils/utils.hpp"

#endif //CHDR_COMMON_HPP