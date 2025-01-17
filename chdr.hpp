/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_COMMON_HPP
#define CHDR_COMMON_HPP

// NOLINTBEGIN(*-include-cleaner)

#include "chdr_version.hpp"

#include "include/mazes/graph.hpp"
#include "include/mazes/grid.hpp"
#include "include/mazes/nodes/id_node.hpp"
#include "include/mazes/nodes/weighted_node.hpp"
#include "include/solvers/astar.hpp"
#include "include/solvers/base/bnode.hpp"
#include "include/solvers/base/solver.hpp"
#include "include/solvers/bfs.hpp"
#include "include/solvers/bstar.hpp"
#include "include/solvers/dfs.hpp"
#include "include/solvers/dijkstra.hpp"
#include "include/solvers/eidastar.hpp"
#include "include/solvers/eidbstar.hpp"
#include "include/solvers/eiddfs.hpp"
#include "include/solvers/floodfill.hpp"
#include "include/solvers/fstar.hpp"
#include "include/solvers/gbfs.hpp"
#include "include/solvers/gdfs.hpp"
#include "include/solvers/gjps.hpp"
#include "include/solvers/gstar.hpp"
#include "include/solvers/idastar.hpp"
#include "include/solvers/idbstar.hpp"
#include "include/solvers/iddfs.hpp"
#include "include/solvers/jps.hpp"
#include "include/types/containers/coord.hpp"
#include "include/types/containers/existence_set.hpp"
#include "include/types/containers/heap.hpp"
#include "include/types/containers/queue.hpp"
#include "include/types/containers/stack.hpp"
#include "include/types/pmr/growing_monotonic_resource.hpp"
#include "include/types/pmr/pool_memory_resource.hpp"
#include "include/utils/heuristics.hpp"
#include "include/utils/intrinsics.hpp"
#include "include/utils/utils.hpp"

// NOLINTEND(*-include-cleaner)

#endif //CHDR_COMMON_HPP