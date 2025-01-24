/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <chdr.hpp>
#include <debug.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>

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
		              << "  astar      A*\n"
		              << "  bfs        Breadth-First Search\n"
		              << "  bstar      Best-First Search\n"
		              << "  dfs        Depth-First Search\n"
		              << "  dijkstra   Dijkstra's Algorithm\n"
		              << "  eidastar   Enhanced Iterative-Deepening A*\n"
		              << "  eidbstar   Enhanced Iterative-Deepening Best-First Search\n"
		              << "  eiddfs     Enhanced Iterative-Deepening Depth-First Search\n"
		              << "  floodfill  Flood Fill\n"
		              << "  fstar      Fringe Search F*\n"
		              << "  gbfs       Graveyard Best-First Search\n"
		              << "  gdfs       Graveyard Depth-First Search\n"
		              << "  gjps       Graveyard Jump-Point Search\n"
		              << "  gstar      Graveyard Search (G*)\n"
		              << "  idastar    Iterative-Deepening A*\n"
		              << "  idbstar    Iterative-Deepening Best-First Search\n"
		              << "  iddfs      Iterative-Deepening Depth-First Search\n"
		              << "  jps        Jump-Point Search\n"
				      << "\nMaze Weight Type:\n"
					  << "  bit        Search space is represented with 1-bit values.\n"
					  << "  byte       Search space is represented with 4-bit values.\n"
		              << "\nExample:\n"
		              << "  chdr astar 10\n"
		              << "  chdr astar 10 10\n"
		              << "  chdr astar 10 10 10\n"
		              << "  chdr astar 10 10 10 10\n"
		              << "\nNote: The dimensions must be integers representing coordinates.\n";
		}

		template <template <typename params_t> typename solver_t, typename params_t>
		static int invoke(const params_t& _params) {
			return application::main<solver_t, params_t>(_params);
		}

		template <typename weight_t, typename coord_t>
		static int deduce_solver(const std::string_view& _solver, const coord_t& _size) {

			int result = EXIT_FAILURE;

			using scalar_t = uint32_t;
			using  index_t = uint32_t;

			/* GENERATE MAZE */
			constexpr size_t seed { 0U };

			constexpr coord_t start {};
			          coord_t end;

			const auto grid = generator::grid::generate<weight_t>(start, end, _size, 0.0, 0.0, seed);

			const auto& test = grid;
			//const auto test = chdr::mazes::graph<index_t, scalar_t>(grid);
			//const auto test = generator::graph::generate<weight_t, index_t, coord_t, scalar_t>(start, end, size, seed);

			auto  monotonic = chdr::monotonic_pool();
			auto* polytonic = std::pmr::new_delete_resource();
			auto  pool      = chdr::polytonic_pool();

			struct params {

		        using  weight_type [[maybe_unused]] = weight_t;
		        using  scalar_type [[maybe_unused]] = scalar_t;
		        using   index_type [[maybe_unused]] =  index_t;
		        using   coord_type [[maybe_unused]] =  coord_t;

				using lazy_sorting [[maybe_unused]] = std::true_type;

		        const decltype(test)& maze;
		        const     coord_type  start;
		        const     coord_type  end;
		        const     coord_type  size;
		                 scalar_type  (*h)(const coord_type&, const coord_type&) noexcept;

				decltype(monotonic)* monotonic_pmr;
				decltype(polytonic)  polytonic_pmr;
				decltype(     pool)* pool_pmr;

		        const scalar_type  weight       = 1U;
		        const      size_t  capacity     = 0U;
		        const      size_t  memoryLimit  = static_cast<size_t>(-1U);
			};

			const params args { test, start, end, _size, chdr::heuristics::manhattan_distance<scalar_t, coord_t>, &monotonic, polytonic, &pool };

                 if (_solver == "astar"    ) { result = invoke<chdr::solvers::    astar, params>(args); }
            else if (_solver == "bfs"      ) { result = invoke<chdr::solvers::      bfs, params>(args); }
            else if (_solver == "bstar"    ) { result = invoke<chdr::solvers::    bstar, params>(args); }
            else if (_solver == "dfs"      ) { result = invoke<chdr::solvers::      dfs, params>(args); }
            else if (_solver == "dijkstra" ) { result = invoke<chdr::solvers:: dijkstra, params>(args); }
            else if (_solver == "eidastar" ) { result = invoke<chdr::solvers:: eidastar, params>(args); }
            else if (_solver == "eidbstar" ) { result = invoke<chdr::solvers:: eidbstar, params>(args); }
            else if (_solver == "eiddfs"   ) { result = invoke<chdr::solvers::   eiddfs, params>(args); }
            else if (_solver == "floodfill") { result = invoke<chdr::solvers::floodfill, params>(args); }
            else if (_solver == "fstar"    ) { result = invoke<chdr::solvers::    fstar, params>(args); }
            else if (_solver == "gbfs"     ) { result = invoke<chdr::solvers::     gbfs, params>(args); }
            else if (_solver == "gdfs"     ) { result = invoke<chdr::solvers::     gdfs, params>(args); }
            else if (_solver == "gjps"     ) { result = invoke<chdr::solvers::     gjps, params>(args); }
            else if (_solver == "gstar"    ) { result = invoke<chdr::solvers::    gstar, params>(args); }
            else if (_solver == "idastar"  ) { result = invoke<chdr::solvers::  idastar, params>(args); }
            else if (_solver == "idbstar"  ) { result = invoke<chdr::solvers::  idbstar, params>(args); }
            else if (_solver == "iddfs"    ) { result = invoke<chdr::solvers::    iddfs, params>(args); }
            else if (_solver == "jps"      ) { result = invoke<chdr::solvers::      jps, params>(args); }
            else {
		        debug::log("ERROR: Unknown solver \"" + std::string(_solver) + "\"!", error);
		    }

			return result;
		}

		template <typename coord_t>
		static int deduce_weight(int _argc, const char* const _argv[], const coord_t& _coord) {

			int result = EXIT_FAILURE;

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

			int result = EXIT_FAILURE;

			using index_t = unsigned long;

			     if (static_cast<size_t>(_argc - 1) == X) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 1U> { std::stoul(_argv[X]) }); }
			else if (static_cast<size_t>(_argc - 1) == Y) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 2U> { std::stoul(_argv[X]), std::stoul(_argv[Y]) }); }
			else if (static_cast<size_t>(_argc - 1) == Z) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 3U> { std::stoul(_argv[X]), std::stoul(_argv[Y]), std::stoul(_argv[Z]) }); }
			else if (static_cast<size_t>(_argc - 1) == W) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 4U> { std::stoul(_argv[X]), std::stoul(_argv[Y]), std::stoul(_argv[Z]), std::stoul(_argv[W]) }); }
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

	int result = EXIT_FAILURE;

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