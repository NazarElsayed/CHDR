#include <chdr.hpp>

#include <Debug.hpp>

void AStarTest() {

    try {

        auto maze = CHDR::Mazes::Grid<2>(32, 32);
        auto solver = CHDR::Solvers::AStar();
        solver.Solve();

        const auto& size = maze.Size();
        Debug::Log(std::to_string(size[0]) + ", " + std::to_string(size[1]));

        auto& node1 = maze.At(0, 0);
        node1.Value(true);

        const auto& node2 = maze.At(0, 0);
        Debug::Log(node2.Value() ? "True" : "False");

        node1.Value(false);
        Debug::Log(node2.Value() ? "True" : "False");
    }
    catch (const std::exception& e){
        Debug::Log("ERROR: AStarTest() failed! REASON: " + std::string(e.what()), LogType::Error);
    }
}

/**
 * @brief Finalises the application.
 *
 * Releases allocated resources.
 *
 * @note This function should only be called when the application is about to terminate.
 */
static void Finalise() noexcept {

	try {

		Debug::Log("Finalise()", LogType::Info);

	    try {
		}
		catch (const std::exception& e) {
			Debug::Log(e, LogType::Critical);
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

			Debug::Log("OnTerminate()! [REASON]: \"" + reason + "\"", LogType::Critical);
		}
		catch (...) {}

		Finalise();

		Debug::Log("Finalised.", LogType::Trace);
	}
	catch (...) {}

	std::exit(343);
}

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] int _argc, [[maybe_unused]] char* _argv[]) {

    try {

    	Debug::Log("main()", LogType::Info);

    	// Set custom termination behaviour:
    	std::set_terminate(&OnTerminate);

    	AStarTest();

    	Finalise();
    }
    catch (const std::exception& e) {
        Debug::Log(e, LogType::Critical);
    }

    return 0;
}