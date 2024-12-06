/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_APPLICATION_HPP
#define TEST_APPLICATION_HPP

#include <atomic>

#include <Debug.hpp>

#include "../tests/astar.hpp"

namespace test {

	/**
	 * @class Application
	 * @brief Represents the application.
	 *
	 * The Application class is responsible for managing the main execution flow of the program and handling the termination of the application.
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

				Debug::Log("Application::finalise()", Info);

				try {
				}
				catch (const std::exception& e) {
					Debug::Log(e, Critical);
				}

				Debug::Flush();
			}
			catch (...) {}
		}

		/**
		 * @brief Called when the application unexpectedly terminates.
		 *
		 * Perform finalisation and then exit the application.
		 *
		 * @see Debug::Log(const std::string_view&, const LogType&, const bool&)
		 * @see Application::finalise()
		 * @see std::exit()
		 */
		static void onTerminate() noexcept {

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

					Debug::Log("onTerminate()! [REASON]: \"" + reason + "\"", Critical);
				}
				catch (...) {}

                finalise();

				Debug::Log("Finalised.", Trace);
			}
			catch (...) {}

			std::exit(343);
		}

	public:

		 application()                          = delete;
		 application(const application& _other) = delete;
		~application()                          = delete;

		application& operator = (const application&  _other) = delete;
		application& operator =       (application&& _other) = delete;

		/**
		 * @brief Entry point of the application.
		 *
		 * This function is the main entry point of the application, and contains the main loop.
		 *
		 * @return An integer error code (0 for successful execution)
		 */
		template<const size_t Kd>
		static int main(const chdr::coord_t<size_t, Kd> _dimensions) {

			Debug::Log("Application::main()", Info);

			// Restrict main() to one instance.
			if (s_initialised) {
				Debug::Log("Attempted to call Application::main() while it is already running! Do you have multiple instances?", Warning);
			}
			else {
                s_quit        = false;
                s_initialised =  true;

				// Set custom termination behaviour:
				std::set_terminate(&onTerminate);

				try {

					/* INIT */

					// ...

					Debug::Log("Application Initialised.", Info);

					/* LOOP */
					while (!s_quit) {

						try {

							/* Put tests here */

							try {
                                tests::astar::run<bool>(_dimensions);
							}
							catch (const std::exception& e) {
								Debug::Log(e);
							}

						}
						catch (const std::exception& e) {
							Debug::Log(e, Critical);
						}

                        quit();
					}
				}
				catch (const std::exception& e) {
					Debug::Log(e, Critical);
				}

				/* FINALISE */
                finalise();

				Debug::Log("Application Terminated Normally.", Info);
			}

			return 0;
		}

		/**
		 * @brief quit function.
		 *
		 * This function is responsible for quitting the application.
		 *
		 * @see Application::s_Quit
		 * @see Debug::Log(const std::string_view&, const LogType&, const bool&)
		 */
		static void quit() noexcept {

			Debug::Log("Application::quit()", Info);

            s_quit = true;
		}
	};

} // LouiEriksson::Engine

#endif //TEST_APPLICATION_HPP