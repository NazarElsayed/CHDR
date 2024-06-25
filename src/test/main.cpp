#include <chdr.hpp>

#include <Debug.hpp>

#include "Ellers.hpp"

template <typename T, typename... Args>
static constexpr auto GenerateMaze(const size_t& _seed = -1, const Args&... _sizes) {

	static_assert(std::is_integral_v<T>, "Type T must be an integer type.");

	std::array sizes { _sizes... };

	auto layout = Ellers::Generate(_seed, sizes[0], sizes[1]);

	std::vector<CHDR::Node<T>> result;
	result.reserve(layout.size());

	std::ostringstream oss;

	for (size_t i = 0; i < layout.size(); ++i) {
		auto& node = result[i];

		bool nodeVal = layout[i];
		node.Value(nodeVal);

		oss << (node.Value() ? "##" : "  ");

		if (i % sizes[0] == 0) {
			oss << '\n';
		}
	}

	std::cout << oss.str() << std::endl;

	return result;
}

static void AStarTest() {

    try {
        auto maze = CHDR::Mazes::Grid<2>(12, 12);
        auto solver = CHDR::Solvers::AStar();

		auto vals = GenerateMaze<bool>(0, maze.Size());

    	//solver.Solve(maze);
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