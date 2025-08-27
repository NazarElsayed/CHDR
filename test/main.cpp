/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <chdr.hpp>
#include <debug.hpp>

#include "core/application.hpp"

namespace test {

    class cli final {

        static constexpr size_t SOLVER             = 1U;
        static constexpr size_t MAZE_WEIGHT        = 2U;
        //static constexpr size_t CLOSED_COMPRESSION = 3U;
        static constexpr size_t X                  = 3U;
        static constexpr size_t Y                  = 4U;
        static constexpr size_t Z                  = 5U;
        static constexpr size_t W                  = 6U;

        static void help() {

            std::cout << "Usage: chdr <command> [arguments]\n"
                      << "\nCommands:\n"
                      << "  <solver> <maze_weight_type> <x>              Process with 1-dimensional coordinate.\n"
                      << "  <solver> <maze_weight_type> <x> <y>          Process with 2-dimensional coordinates.\n"
                      << "  <solver> <maze_weight_type> <x> <y> <z>      Process with 3-dimensional coordinates.\n"
                      << "  <solver> <maze_weight_type> <x> <y> <z> <w>  Process with 4-dimensional coordinates.\n"
                      << "\nSolvers:\n"
                      << "  astar          A*\n"
                      << "  bfs            Breadth-First Search\n"
                      << "  best_first     Best-First Search\n"
                      << "  dfs            Depth-First Search\n"
                      << "  dijkstra       Dijkstra's Algorithm\n"
                      << "  eidastar       Enhanced Iterative-Deepening A*\n"
                      << "  eidbest_first  Enhanced Iterative-Deepening Best-First Search\n"
                      << "  eiddfs         Enhanced Iterative-Deepening Depth-First Search\n"
                      << "  flood          Flood Fill\n"
                      << "  fringe         Fringe Search F*\n"
                      << "  gbest_first    Graveyard Best-First Search\n"
                      << "  gbfs           Graveyard Breadth-First Search\n"
                      << "  gdfs           Graveyard Depth-First Search\n"
                      << "  gjps           Graveyard Jump-Point Search\n"
                      << "  mgstar         Memory-Bounded Graveyard Search\n"
                      << "  smastar        Simplified Memory-Bounded A* Search\n"
                      << "  osmastar       Optimising Simplified Memory-Bounded A* Search\n"
                      << "  gstar          Graveyard Search (G*)\n"
                      << "  idastar        Iterative-Deepening A*\n"
                      << "  idbest_first   Iterative-Deepening Best-First Search\n"
                      << "  iddfs          Iterative-Deepening Depth-First Search\n"
                      << "  jps            Jump-Point Search\n"
                      << "\nMaze Weight Type:\n"
                      << "  bit            Search space represented using 1-bit values.\n"
                      << "  byte           Search space represented using 4-bit values.\n"
                      << "\nExample:\n"
                      << "  chdr astar byte 10\n"
                      << "  chdr astar byte 10 10\n"
                      << "  chdr astar byte 10 10 10\n"
                      << "  chdr astar byte 10 10 10 10\n"
                      << "\nNote: The dimensions must be integers representing coordinates.\n";
        }

        template <template <typename params_t> typename solver_t, typename params_t>
        static int invoke(const params_t& _params) {
            return application::main<solver_t, params_t>(_params);
        }

        template <typename weight_t, typename coord_t>
        static int deduce_solver(const std::string_view& _solver, const coord_t& _size) {

            auto result = EXIT_FAILURE;

            using scalar_t = uint32_t;
            using  index_t = uint32_t;

            /* GENERATE MAZE */
            constexpr size_t seed { 0U };

            const coord_t start {};
                  coord_t end;

            for (size_t i = 0U; i < _size.size(); ++i) {
                end[i] = _size[i] - 1U;
            }

            // Empty grid:
            // const std::vector<weight_t> nodes(chdr::utils::product<size_t>(_size), std::numeric_limits<weight_t>::lowest());
            // const auto grid = chdr::mazes::grid<coord_t, weight_t>(_size, nodes);
            //debug::log(_size[0] + _size[1] - 2U);

            // Random grid:
            //const auto grid = generator::obstacles::generate<weight_t, index_t, coord_t, scalar_t>(start, end, _size, 0.1, std::numeric_limits<size_t>::max(), seed);

            // Maze grid:
            const auto grid = generator::grid::generate<weight_t, coord_t, scalar_t>(start, end, _size, 0.0, 0.0, seed);

            const auto& test = grid;
            // auto graph_pool = chdr::heterogeneous_pool(); const auto test = chdr::mazes::graph<index_t, scalar_t>(grid, &graph_pool);
            // const auto test = generator::graph::generate<weight_t, index_t, coord_t, scalar_t>(start, end, size, seed);

            auto     monotonic = chdr::monotonic_pool();
            auto heterogeneous = chdr::heterogeneous_pool();
            auto   homogeneous = chdr::homogeneous_pool();

            struct params {

                using  weight_type [[maybe_unused]] = weight_t;
                using  scalar_type [[maybe_unused]] = scalar_t;
                using   index_type [[maybe_unused]] =  index_t;
                using   coord_type [[maybe_unused]] =  coord_t;

                using lazy_sorting [[maybe_unused]] = std::false_type;
                using   no_cleanup [[maybe_unused]] = std:: true_type;

                const decltype(test)& maze;
                const     coord_type  start;
                const     coord_type  end;
                const     coord_type  size;
                scalar_type  (*h)(const coord_type&, const coord_type&) noexcept;

                decltype(    monotonic)*     monotonic_pmr;
                decltype(heterogeneous)* heterogeneous_pmr;
                decltype(  homogeneous)*   homogeneous_pmr;

                const scalar_type weight       =  1U;
                const      size_t capacity     =  0U;
                const      size_t memory_limit = 90U;
            };

            const params args { test, start, end, _size, chdr::heuristics::manhattan_distance<scalar_t, coord_t>, &monotonic, &heterogeneous, &homogeneous };

                 if (_solver == "astar"        ) { result = invoke<chdr::solvers::        astar, params>(args); }
            else if (_solver == "best_first"   ) { result = invoke<chdr::solvers::   best_first, params>(args); }
            else if (_solver == "bfs"          ) { result = invoke<chdr::solvers::          bfs, params>(args); }
            else if (_solver == "dfs"          ) { result = invoke<chdr::solvers::          dfs, params>(args); }
            else if (_solver == "dijkstra"     ) { result = invoke<chdr::solvers::     dijkstra, params>(args); }
            else if (_solver == "eidastar"     ) { result = invoke<chdr::solvers::     eidastar, params>(args); }
            else if (_solver == "eidbest_first") { result = invoke<chdr::solvers::eidbest_first, params>(args); }
            else if (_solver == "eiddfs"       ) { result = invoke<chdr::solvers::       eiddfs, params>(args); }
            else if (_solver == "flood"        ) { result = invoke<chdr::solvers::        flood, params>(args); }
            else if (_solver == "fringe"       ) { result = invoke<chdr::solvers::       fringe, params>(args); }
            else if (_solver == "gbest_first"  ) { result = invoke<chdr::solvers::  gbest_first, params>(args); }
            else if (_solver == "gbfs"         ) { result = invoke<chdr::solvers::         gbfs, params>(args); }
            else if (_solver == "gdfs"         ) { result = invoke<chdr::solvers::         gdfs, params>(args); }
            else if (_solver == "gjps"         ) { result = invoke<chdr::solvers::         gjps, params>(args); }
            else if (_solver == "gstar"        ) { result = invoke<chdr::solvers::        gstar, params>(args); }
            else if (_solver == "idastar"      ) { result = invoke<chdr::solvers::      idastar, params>(args); }
            else if (_solver == "idbest_first" ) { result = invoke<chdr::solvers:: idbest_first, params>(args); }
            else if (_solver == "iddfs"        ) { result = invoke<chdr::solvers::        iddfs, params>(args); }
            else if (_solver == "jps"          ) { result = invoke<chdr::solvers::          jps, params>(args); }
            else if (_solver == "mgstar"       ) { result = invoke<chdr::solvers::       mgstar, params>(args); }
            else if (_solver == "osmastar"     ) { result = invoke<chdr::solvers::     osmastar, params>(args); }
            else if (_solver == "smastar"      ) { result = invoke<chdr::solvers::      smastar, params>(args); }
            else {
                debug::log("ERROR: Unknown solver \"" + std::string(_solver) + "\"!", error);
            }

#if CHDR_DIAGNOSTICS == 1

            auto peak_memory_usage = monotonic.__get_diagnostic_data().peak_allocated +
                                 heterogeneous.__get_diagnostic_data().peak_allocated +
                                   homogeneous.__get_diagnostic_data().peak_allocated;

            std::cout << peak_memory_usage << "\n";

#endif //CHDR_DIAGNOSTICS == 1

            return result;
        }

        template <typename coord_t>
        static int deduce_weight(int _argc, const char* const _argv[], const coord_t& _coord) {

            auto result = EXIT_FAILURE;

            if (_argc > 0 && static_cast<size_t>(_argc) > MAZE_WEIGHT) {

                const std::string maze_format { _argv[MAZE_WEIGHT] };

                     if (maze_format == "bit"   ) { result = deduce_solver<bool>    ( { _argv[SOLVER] }, _coord); }
                else if (maze_format == "byte"  ) { result = deduce_solver<char>    ( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "int32" ) { result = deduce_solver<int32_t> ( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "uint32") { result = deduce_solver<uint32_t>( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "int64" ) { result = deduce_solver<int32_t> ( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "uint64") { result = deduce_solver<uint32_t>( { _argv[SOLVER] }, _coord); }
                else {
                    debug::log("ERROR: Unknown maze format \"" + std::string(maze_format) + "\"!", error);
                }
            }

            return result;
        }

        static int deduce_coord(int _argc, const char* const _argv[]) {

            auto result = EXIT_FAILURE;

            using index_t = uint32_t;

                 // if (static_cast<size_t>(_argc - 1) == X) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 1U> { static_cast<index_t>(std::stoul(_argv[X])) }); }
            if (static_cast<size_t>(_argc - 1) == Y) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 2U> { static_cast<index_t>(std::stoul(_argv[X])), static_cast<index_t>(std::stoul(_argv[Y])) }); }
            // else if (static_cast<size_t>(_argc - 1) == Z) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 3U> { static_cast<index_t>(std::stoul(_argv[X])), static_cast<index_t>(std::stoul(_argv[Y])), static_cast<index_t>(std::stoul(_argv[Z])) }); }
            // else if (static_cast<size_t>(_argc - 1) == W) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 4U> { static_cast<index_t>(std::stoul(_argv[X])), static_cast<index_t>(std::stoul(_argv[Y])), static_cast<index_t>(std::stoul(_argv[Z])), static_cast<index_t>(std::stoul(_argv[W])) }); }
            else {
                debug::log("ERROR: Invalid Dimensionality!", error);
            }

            return result;
        }

    public:

        static int main(const int _argc, const char* const _argv[]) {

            const auto result = deduce_coord(_argc, _argv);

            if (result != EXIT_SUCCESS) {
                help();
            }

            return result;
        }
    };

}

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main(const int _argc, const char* const _argv[]) noexcept {

    auto result = EXIT_FAILURE;

    try {
        debug::log("CHDR " CHDR_VERSION);
        debug::log("Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson");
        debug::log("Licensed under CC BY-NC-ND 4.0");

        debug::log("main()", info);

        result = test::cli::main(_argc, _argv);
    }
    catch(...) {} //NOLINT(*-empty-catch)

    return result;
}