/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_COMMON_HPP
#define CHDR_COMMON_HPP

/**
 * @file chdr.hpp
 *
 * @details Include this header in your project to access the CHDR library.
 *
 * @code
 * #include <chdr.hpp>
 * @endcode
 *
 * @note Please refer to the [manual](docs/manual/getting_started.md) for more information.
 */

/** @brief Full version string of the CHDR library. */
#define CHDR_VERSION          "0.1.0-dev+c7954f52"

/** @brief CHDR major version number. */
#define CHDR_VERSION_MAJOR     0

/** @brief CHDR minor version number. */
#define CHDR_VERSION_MINOR     1

/** @brief CHDR patch version number. */
#define CHDR_VERSION_PATCH     0

/** @brief CHDR version tweak string. */
#define CHDR_VERSION_TWEAK    "dev"

/** @brief CHDR version metadata string. */
#define CHDR_VERSION_METADATA "c7954f52"

/* ReSharper enable CppUnusedIncludeDirective */
// NOLINTBEGIN(*-include-cleaner)

#include "include/mazes/graph.hpp"
#include "include/mazes/grid.hpp"
#include "include/mazes/nodes/id_node.hpp"
#include "include/mazes/nodes/weighted_node.hpp"
#include "include/solvers/astar.hpp"
#include "include/solvers/base/bnode.hpp"
#include "include/solvers/base/managed_node.hpp"
#include "include/solvers/base/solver.hpp"
#include "include/solvers/base/unmanaged_node.hpp"
#include "include/solvers/best_first.hpp"
#include "include/solvers/bfs.hpp"
#include "include/solvers/dfs.hpp"
#include "include/solvers/dijkstra.hpp"
#include "include/solvers/eidastar.hpp"
#include "include/solvers/eidbest_first.hpp"
#include "include/solvers/eiddfs.hpp"
#include "include/solvers/flood.hpp"
#include "include/solvers/fstar.hpp"
#include "include/solvers/gbest_first.hpp"
#include "include/solvers/gbfs.hpp"
#include "include/solvers/gdfs.hpp"
#include "include/solvers/gjps.hpp"
#include "include/solvers/gstar.hpp"
#include "include/solvers/idastar.hpp"
#include "include/solvers/idbest_first.hpp"
#include "include/solvers/iddfs.hpp"
#include "include/solvers/jps.hpp"
#include "include/types/containers/coord.hpp"
#include "include/types/containers/existence_set.hpp"
#include "include/types/containers/heap.hpp"
#include "include/types/containers/queue.hpp"
#include "include/types/containers/stack.hpp"
#include "include/types/pmr/heterogeneous_pool.hpp"
#include "include/types/pmr/homogeneous_pool.hpp"
#include "include/types/pmr/monotonic_pool.hpp"
#include "include/utils/heuristics.hpp"
#include "include/utils/intrinsics.hpp"
#include "include/utils/utils.hpp"

// NOLINTEND(*-include-cleaner)
/* ReSharper disable CppUnusedIncludeDirective */

#endif //CHDR_COMMON_HPP
