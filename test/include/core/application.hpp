/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_APPLICATION_HPP
#define TEST_APPLICATION_HPP

#include <chdr.hpp>
#include <debug.hpp>

#include <atomic>
#include <cstdlib>
#include <string>

#include "../units/solver.hpp"

namespace test {

	/**
	 * @class application
	 * @brief Represents the application.
	 *
	 * The application class is responsible for managing the main execution flow of the program and handling the termination of the application.
	 */
	class application final {

	private:

		inline static std::atomic<bool> s_quit        { false }; // Is Application scheduled to quit?
		inline static std::atomic<bool> s_initialised { false }; // Is Application initialised?

		inline static std::unique_ptr<std::byte[]> s_contingency_block;			// NOLINT(*-avoid-c-arrays)

		static void reinforce_contingent_memory() noexcept {
			try {
				if (s_contingency_block == nullptr) {
					s_contingency_block = std::make_unique<std::byte[]>(16384U);	// NOLINT(*-avoid-c-arrays)
				}
			}
			catch (...) {
				/*
				 * There may not be enough memory to print or do just about anything.
				 * So allow the exception to fall through, and hope the failed allocation
				 * is picked up by the critical new handler before the OS kills the app.
				 */
			}
		}

		/**
		 * @brief Finalises the application.
		 *
		 * Releases allocated resources.
		 *
		 * @note This function should only be called when the application is about to terminate.
		 */
		static void finalise() noexcept {

			try {

				debug::log("application::finalise()", info);

				try {
				}
				catch (const std::exception& e) {
					debug::log(e, critical);
				}

				debug::flush();
			}
			catch (...) {}
		}

		/**
		 * @brief Called when the application unexpectedly terminates.
		 *
		 * Perform finalisation and then exit the application.
		 *
		 * @see debug::log(const std::string_view&, const LogType&, const bool&)
		 * @see application::finalise()
		 * @see std::exit()
		 */
		static void on_terminate() noexcept {

			try {
				try {

					std::string reason = "NULL";

					if (const auto critical_error = std::current_exception(); critical_error != nullptr) {

						try {
							std::rethrow_exception(critical_error);
						}
						catch (const std::exception &e) {
							reason = e.what();
						}
						catch (...) {
							reason = "UNKNOWN\"";
						}
					}

					debug::log("application::on_terminate()! [REASON]: \"" + reason + "\"", critical);
				}
				catch (...) {}

                finalise();

				debug::log("Finalised.", trace);
			}
			catch (...) {}

			std::exit(343);
			// ReSharper disable once CppDFAUnreachableCode
		}

		/**
		 * @brief Custom handler for memory allocation failures.
		 *
		 * This function is invoked when the system is unable to allocate memory.
		 * It logs the error and terminates the application gracefully.
		 */
		static void critical_new_handler() noexcept {

			bool has_lifeline = s_contingency_block != nullptr;
			s_contingency_block.reset();

			try {
				debug::log("application::critical_new_handler(): Memory allocation failure!", critical);
			}
			catch(...) {
			}

			if (!has_lifeline) {
				on_terminate();
			}
		}
		
	public:

		 application()                           = delete;
		 application(const application&  _other) = delete;
		 application(      application&& _other) = delete;
		~application()                           = delete;

		application& operator = (const application&  _other) = delete;
		application& operator =       (application&& _other) = delete;

		/**
		 * @brief Entry point of the application.
		 *
		 * This function is the main entry point of the application, and contains the main loop.
		 *
		 * @return An integer error code (0 for successful execution)
		 */
		template<typename coord_t>
		[[nodiscard]] static int main(std::string_view _solver, const coord_t& _size) noexcept {

			debug::log("application::main()", info);

			// Restrict main() to one instance.
			if (s_initialised) {
				debug::log("Attempted to call application::main() while it is already running! Do you have multiple instances?", warning);
			}
			else {

				try {

					s_quit        = false;
					s_initialised =  true;

					reinforce_contingent_memory();

					// Set custom termination behaviour:
					std::set_terminate(on_terminate);
					std::set_new_handler(critical_new_handler);

					/* INIT */

					// ...

					debug::log("Application Initialised.", info);

					/* LOOP */
					while (!s_quit) {

						try {

							try {

								using weight_t = char;
								using scalar_t = uint32_t;
								using  index_t = uint32_t;

								/* GENERATE MAZE */
								constexpr auto seed { 0U };

								constexpr coord_t start {};
								          coord_t end;

								const auto grid = generator::grid::generate<weight_t>({}, end, _size, 0.0, 0.0, seed);

								const auto test = grid;
								//const auto test = chdr::mazes::graph<index_t, scalar_t>(grid);
								//const auto test = generator::graph::generate<weight_t, index_t, coord_t, scalar_t>(start, end, size, seed);

								struct params {

							        using weight_type = weight_t;
							        using scalar_type = scalar_t;
							        using  index_type =  index_t;
							        using  coord_type =  coord_t;

							        const decltype(test) maze;
							        const     coord_type start;
							        const     coord_type end;
							        const     coord_type size;
							                 scalar_type (*h)(const coord_type&, const coord_type&) noexcept;
							        const    scalar_type weight      =  1U;
							        const     index_type capacity    =  0U;
							        const     index_type memoryLimit = -1U;
							    };

								const params args { test, start, end, _size, chdr::heuristics::manhattan_distance<scalar_t, coord_t> };

							         if (_solver == "astar"    ) { tests::solver::run<chdr::solvers::    astar, params>(args); }
							    else if (_solver == "bfs"      ) { tests::solver::run<chdr::solvers::      bfs, params>(args); }
							    else if (_solver == "bstar"    ) { tests::solver::run<chdr::solvers::    bstar, params>(args); }
							    else if (_solver == "dfs"      ) { tests::solver::run<chdr::solvers::      dfs, params>(args); }
							    else if (_solver == "dijkstra" ) { tests::solver::run<chdr::solvers:: dijkstra, params>(args); }
							    else if (_solver == "eidastar" ) { tests::solver::run<chdr::solvers:: eidastar, params>(args); }
							    else if (_solver == "eidbstar" ) { tests::solver::run<chdr::solvers:: eidbstar, params>(args); }
							    else if (_solver == "eiddfs"   ) { tests::solver::run<chdr::solvers::   eiddfs, params>(args); }
							    else if (_solver == "floodfill") { tests::solver::run<chdr::solvers::floodfill, params>(args); }
							    else if (_solver == "fstar"    ) { tests::solver::run<chdr::solvers::    fstar, params>(args); }
							    else if (_solver == "gbfs"     ) { tests::solver::run<chdr::solvers::     gbfs, params>(args); }
							    else if (_solver == "gdfs"     ) { tests::solver::run<chdr::solvers::     gdfs, params>(args); }
							    else if (_solver == "gjps"     ) { tests::solver::run<chdr::solvers::     gjps, params>(args); }
							    else if (_solver == "gstar"    ) { tests::solver::run<chdr::solvers::    gstar, params>(args); }
							    else if (_solver == "idastar"  ) { tests::solver::run<chdr::solvers::  idastar, params>(args); }
							    else if (_solver == "idbstar"  ) { tests::solver::run<chdr::solvers::  idbstar, params>(args); }
							    else if (_solver == "iddfs"    ) { tests::solver::run<chdr::solvers::    iddfs, params>(args); }
							    else if (_solver == "jps"      ) { tests::solver::run<chdr::solvers::      jps, params>(args); }
							    else {
							        debug::log("ERROR: Unknown solver " + std::string(_solver) + "!", error);
							    }
							}
							catch (const std::exception& e) {
								debug::log(e, error);
							}

							reinforce_contingent_memory();
						}
						catch (const std::exception& e) {
							debug::log(e, critical);
						}

                        quit();
					}
				}
				catch (const std::exception& e) {
					debug::log(e, critical);
				}

				/* FINALISE */
                finalise();

				debug::log("Application Terminated Normally.", info);
			}

			return 0;
		}

		/**
		 * @brief quit function.
		 *
		 * This function is responsible for quitting the application.
		 *
		 * @see application::s_Quit
		 * @see debug::log(const std::string_view&, const LogType&, const bool&)
		 */
		static void quit() noexcept {

			debug::log("application::quit()", info);

            s_quit = true;
		}
	};

} //test

#endif //TEST_APPLICATION_HPP