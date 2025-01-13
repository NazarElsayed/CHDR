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
		template <template <typename params_t> typename solver_t, typename params_t>
		[[nodiscard]] static int main(const params_t& _params) noexcept {

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
								tests::solver::run<solver_t, params_t>(_params);
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

			return EXIT_SUCCESS;
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