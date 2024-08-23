#ifndef TEST_APPLICATION_HPP
#define TEST_APPLICATION_HPP

#include <atomic>

#include <Debug.hpp>

#include "../tests/AStar.hpp"

namespace Test {

	/**
	 * @class Application
	 * @brief Represents the application.
	 *
	 * The Application class is responsible for managing the main execution flow of the program and handling the termination of the application.
	 */
	class Application final {

	private:

		inline static std::atomic<bool> s_Quit        { false }; // Is Application scheduled to quit?
		inline static std::atomic<bool> s_Initialised { false }; // Is Application initialised?

		/**
		 * @brief Finalises the application.
		 *
		 * Releases allocated resources.
		 *
		 * @note This function should only be called when the application is about to terminate.
		 */
		static void Finalise() noexcept {

			try {

				Debug::Log("Application::Finalise()", Info);

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
		 * @see Application::Finalise()
		 * @see std::exit()
		 */
		static void OnTerminate() noexcept {

			try {
				try {

					std::string reason = "NULL";

					auto critical_error = std::current_exception();
					if (critical_error != nullptr) {

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

					Debug::Log("OnTerminate()! [REASON]: \"" + reason + "\"", Critical);
				}
				catch (...) {}

				Finalise();

				Debug::Log("Finalised.", Trace);
			}
			catch (...) {}

			std::exit(343);
		}

	public:

		 Application()                          = delete;
		 Application(const Application& _other) = delete;
		~Application()                          = delete;

		Application& operator = (const Application&  _other) = delete;
		Application& operator =       (Application&& _other) = delete;

		/**
		 * @brief Entry point of the application.
		 *
		 * This function is the main entry point of the application, and contains the main loop.
		 *
		 * @return An integer error code (0 for successful execution)
		 */
		static int Main(const std::array<long unsigned, 2U> _dimensions) {

		    //Print Version
		    Debug::Log("CHDR Version: v" CHDR_VERSION, Info);

			Debug::Log("Application::Main()", Info);

			// Restrict Main() to one instance.
			if (s_Initialised) {
				Debug::Log("Attempted to call Application::Main() while it is already running! Do you have multiple instances?", Warning);
			}
			else {
				s_Quit        = false;
				s_Initialised =  true;

				// Set custom termination behaviour:
				std::set_terminate(&OnTerminate);

				try {

					/* INIT */

					// ...

					Debug::Log("Application Initialised.", Info);

					/* LOOP */
					while (!s_Quit) {

						try {

							/* Put tests here */

							try {
								Tests::AStar::Run<bool, 2U>(std::array<long unsigned, 2U>(_dimensions));
							}
							catch (const std::exception& e) {
								Debug::Log(e);
							}

						}
						catch (const std::exception& e) {
							Debug::Log(e, Critical);
						}

						Quit();
					}
				}
				catch (const std::exception& e) {
					Debug::Log(e, Critical);
				}

				/* FINALISE */
				Finalise();

				Debug::Log("Application Terminated Normally.", Info);
			}

			return 0;
		}

		/**
		 * @brief Quit function.
		 *
		 * This function is responsible for quitting the application.
		 *
		 * @see Application::s_Quit
		 * @see Debug::Log(const std::string_view&, const LogType&, const bool&)
		 */
		static void Quit() noexcept {

			Debug::Log("Application::Quit()", Info);

			s_Quit = true;
		}
	};

} // LouiEriksson::Engine

#endif //TEST_APPLICATION_HPP