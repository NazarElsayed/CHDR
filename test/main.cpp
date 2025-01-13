/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <chdr.hpp>
#include <debug.hpp>

#include <iostream>
#include <string>
#include <string_view>

#include "core/application.hpp"

namespace test {

	class cli final {

		static void help() {

		    std::cout << "Usage: chdr <command> [arguments]\n"
		              << "\nCommands:\n"
		              << "  <solver> <x>              Process with 1-dimensional coordinate.\n"
		              << "  <solver> <x> <y>          Process with 2-dimensional coordinates.\n"
		              << "  <solver> <x> <y> <z>      Process with 3-dimensional coordinates.\n"
		              << "  <solver> <x> <y> <z> <w>  Process with 4-dimensional coordinates.\n"
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
		              << "\nExample:\n"
		              << "  chdr astar 10\n"
		              << "  chdr astar 10 10\n"
		              << "  chdr astar 10 10 10\n"
		              << "  chdr astar 10 10 10 10\n"
		              << "\nNote: The dimensions must be integers representing coordinates.\n";

		}

		template <typename coord_t>
		static int deduce_solver(const std::string_view& _solver, const coord_t& _size) {

			int result = EXIT_FAILURE;

			using weight_t = char;
			using scalar_t = uint32_t;
			using  index_t = uint32_t;

			/* GENERATE MAZE */
			constexpr auto seed { 0U };

			constexpr coord_t start {};
			          coord_t end;

			const auto grid = test::generator::grid::generate<weight_t>({}, end, _size, 0.0, 0.0, seed);

			const auto test = grid;
			//const auto test = chdr::mazes::graph<index_t, scalar_t>(grid);
			//const auto test = generator::graph::generate<weight_t, index_t, coord_t, scalar_t>(start, end, size, seed);

			struct params {

		        using weight_type [[maybe_unused]] = weight_t;
		        using scalar_type [[maybe_unused]] = scalar_t;
		        using  index_type [[maybe_unused]] =  index_t;
		        using  coord_type [[maybe_unused]] =  coord_t;

		        const decltype(test)& maze;
		        const     coord_type start;
		        const     coord_type end;
		        const     coord_type size;
		                 scalar_type (*h)(const coord_type&, const coord_type&) noexcept;
		        const    scalar_type weight      =  1U;
		        const         size_t capacity    =  0U;
		        const         size_t memoryLimit = -1U;
		    };

			const params args { test, start, end, _size, chdr::heuristics::manhattan_distance<scalar_t, coord_t> };

		         if (_solver == "astar"    ) { result = test::application::main<chdr::solvers::    astar, params>(args); }
		    else if (_solver == "bfs"      ) { result = test::application::main<chdr::solvers::      bfs, params>(args); }
		    else if (_solver == "bstar"    ) { result = test::application::main<chdr::solvers::    bstar, params>(args); }
		    else if (_solver == "dfs"      ) { result = test::application::main<chdr::solvers::      dfs, params>(args); }
		    else if (_solver == "dijkstra" ) { result = test::application::main<chdr::solvers:: dijkstra, params>(args); }
		    else if (_solver == "eidastar" ) { result = test::application::main<chdr::solvers:: eidastar, params>(args); }
		    else if (_solver == "eidbstar" ) { result = test::application::main<chdr::solvers:: eidbstar, params>(args); }
		    else if (_solver == "eiddfs"   ) { result = test::application::main<chdr::solvers::   eiddfs, params>(args); }
		    else if (_solver == "floodfill") { result = test::application::main<chdr::solvers::floodfill, params>(args); }
		    else if (_solver == "fstar"    ) { result = test::application::main<chdr::solvers::    fstar, params>(args); }
		    else if (_solver == "gbfs"     ) { result = test::application::main<chdr::solvers::     gbfs, params>(args); }
		    else if (_solver == "gdfs"     ) { result = test::application::main<chdr::solvers::     gdfs, params>(args); }
		    else if (_solver == "gjps"     ) { result = test::application::main<chdr::solvers::     gjps, params>(args); }
		    else if (_solver == "gstar"    ) { result = test::application::main<chdr::solvers::    gstar, params>(args); }
		    else if (_solver == "idastar"  ) { result = test::application::main<chdr::solvers::  idastar, params>(args); }
		    else if (_solver == "idbstar"  ) { result = test::application::main<chdr::solvers::  idbstar, params>(args); }
		    else if (_solver == "iddfs"    ) { result = test::application::main<chdr::solvers::    iddfs, params>(args); }
		    else if (_solver == "jps"      ) { result = test::application::main<chdr::solvers::      jps, params>(args); }
		    else {
		        debug::log("ERROR: Unknown solver \"" + std::string(_solver) + "\"!", error);
		    }

			return result;
		}

		static int deduce_coord(const int& _argc, const char* const _argv[]) {

			int result = EXIT_FAILURE;

			using index_t = unsigned long;

			     if (_argc == 3U) { result = deduce_solver( { _argv[1U] }, chdr::coord<index_t, 1U> { std::stoul(_argv[2U]) }); }
			else if (_argc == 4U) { result = deduce_solver( { _argv[1U] }, chdr::coord<index_t, 2U> { std::stoul(_argv[2U]), std::stoul(_argv[3U]) }); }
			else if (_argc == 5U) { result = deduce_solver( { _argv[1U] }, chdr::coord<index_t, 3U> { std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]) }); }
			else if (_argc == 6U) { result = deduce_solver( { _argv[1U] }, chdr::coord<index_t, 4U> { std::stoul(_argv[2U]), std::stoul(_argv[3U]), std::stoul(_argv[4U]), std::stoul(_argv[5U]) }); }
			else {
				debug::log("ERROR: Invalid Dimensionality!", error);
			}

			return result;
		}

	public:

		static int execute(const int& _argc, const char* const _argv[]) {

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
int main(int _argc, const char* const _argv[]) noexcept {

	int result = EXIT_FAILURE;

	try {
		debug::log("CHDR " CHDR_VERSION);
		debug::log("Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson");
		debug::log("Licensed under CC BY-NC-ND 4.0");

		debug::log("main()", info);

		result = test::cli::execute(_argc, _argv);
	}
	catch(...) { /* ignored */ }

	return result;
}