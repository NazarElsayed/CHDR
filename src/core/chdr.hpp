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

#include "CHDR_Version.hpp"

#include "mazes/base/igraph.hpp"
#include "mazes/base/imaze.hpp"
#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "mazes/nodes/base/inode.hpp"
#include "mazes/nodes/id_node.hpp"
#include "mazes/nodes/weighted_node.hpp"

#include "solvers/astar.hpp"
#include "solvers/base/bnode.hpp"
#include "solvers/base/solver.hpp"
#include "solvers/bfs.hpp"
#include "solvers/bstar.hpp"
#include "solvers/dfs.hpp"
#include "solvers/dijkstra.hpp"
#include "solvers/esmgstar.hpp"
#include "solvers/floodfill.hpp"
#include "solvers/fstar.hpp"
#include "solvers/gbfs.hpp"
#include "solvers/gdfs.hpp"
#include "solvers/gstar.hpp"
#include "solvers/idastar.hpp"
#include "solvers/iddfs.hpp"
#include "solvers/jps.hpp"

#include "types/circular_queue.hpp"
#include "types/coord.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "types/queue.hpp"
#include "types/stable_forward_buf.hpp"
#include "types/stack.hpp"
#include "types/stack_allocator.hpp"

#include "utils/heuristics.hpp"
#include "utils/intrinsics.hpp"
#include "utils/simd_extensions.hpp"

#include "utils/utils.hpp"

// NOLINTEND(*-include-cleaner)

#endif //CHDR_COMMON_HPP