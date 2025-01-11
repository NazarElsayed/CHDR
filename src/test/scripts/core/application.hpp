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

#include "../tests/astar.hpp"

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

		/**
		 * @brief Finalises the application.
		 *
		 * Releases allocated resources.
		 *
		 * @note This function should only be called when the application is about to terminate.
		 */
		static void finalise() noexcept {

			try {

				debug::log("Application::finalise()", info);

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
		 * @see Application::finalise()
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

					debug::log("onTerminate()! [REASON]: \"" + reason + "\"", critical);
				}
				catch (...) {}

                finalise();

				debug::log("Finalised.", trace);
			}
			catch (...) {}

			std::exit(343);
			// ReSharper disable once CppDFAUnreachableCode
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
		template<size_t Kd>
		static int main(const chdr::coord<size_t, Kd> _dimensions) {

			debug::log("Application::main()", info);

			// Restrict main() to one instance.
			if (s_initialised) {
				debug::log("Attempted to call Application::main() while it is already running! Do you have multiple instances?", warning);
			}
			else {
                s_quit        = false;
                s_initialised =  true;

				// Set custom termination behaviour:
				std::set_terminate(&on_terminate);

				try {

					/* INIT */

					// ...

					debug::log("Application Initialised.", info);

					/* LOOP */
					while (!s_quit) {

						try {

							/* Put tests here */

							try {
                                tests::astar::run<char>(_dimensions);
							}
							catch (const std::exception& e) {
								debug::log(e, error);
							}

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
		 * @see Application::s_Quit
		 * @see debug::log(const std::string_view&, const LogType&, const bool&)
		 */
		static void quit() noexcept {

			debug::log("Application::quit()", info);

            s_quit = true;
		}
	};

} //test

#endif //TEST_APPLICATION_HPP