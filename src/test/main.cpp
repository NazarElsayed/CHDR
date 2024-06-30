#include <chdr.hpp>

#include <Debug.hpp>

#include "RecursiveBacktrack.hpp"

#include <iomanip>

template <typename T, size_t Kd>
static constexpr void DisplayGridMaze(const CHDR::Coord<size_t, Kd>& _start, const CHDR::Coord<size_t, Kd>& _end, const CHDR::Coord<size_t, Kd>& _size, const CHDR::Mazes::Grid<Kd, T>& _maze) {

	static_assert(              Kd < 3U, "This maze renderer does not support dimensions higher than 2.");
	static_assert(std::is_integral_v<T>, "Maze type must be an integral type."                          );

	std::ostringstream oss;

	constexpr auto* path_str = "  ";
	constexpr auto* wall_str = "██";
	constexpr auto  line_brk = '\n';

	const bool even_width = _size[0U] % 2U == 0U;

	// Add an upper boundary:
	{
		const auto columns = _size[0U] + (even_width ? 1U : 2U);

		for (size_t i = 0U; i < columns; ++i) {
			oss << wall_str;
		}
		oss << line_brk;
	}

	const auto& nodes = _maze.Nodes();
	for (size_t i = 0U; i < nodes.size(); ++i) {

		if (i % _size[0U] == 0U) { oss << wall_str; }

		auto s = CHDR::Utils::To1D(_start, _size);
		auto e = CHDR::Utils::To1D(_end,   _size);

		     if (i == s) { oss << "St"; }
		else if (i == e) { oss << "En"; }
		else {

			auto val = nodes[i].Value();

			if constexpr (std::is_same_v<T, bool>) {
				oss << (val ? wall_str : path_str);
			}
			else {
				oss << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(val);
			}

			// Handle end of line:
			const bool end_of_line = (Kd == 1 && i == _size[0U] - 1U) || ((i + 1U) % _size[0U] == 0U);

			if (end_of_line) {
			    if (!even_width) {
			        oss << wall_str;
			    }
			    oss << line_brk;
			}

		}
	}

	// Handle the addition of a lower boundary:
	{
		const bool even_height = Kd > 1U && _size[1U] % 2U == 0U;
		if (!even_height) {

			const auto columns = _size[0U] + (even_width ? 1U : 2U);
			for (size_t i = 0U; i < columns; ++i) {
				oss << wall_str;
			}
			oss << line_brk;
		}
	}

	std::cout << oss.str();
}

template <typename T, size_t Kd, typename... Args>
static constexpr auto GenerateMaze(const CHDR::Coord<size_t, Kd>& _start, CHDR::Coord<size_t, Kd>& _end, const size_t& _seed = -1, const Args&... _size) {

	static_assert(std::is_integral_v<T>, "Type T must be an integral type.");

	std::array size { _size... };

	auto layout = RecursiveBacktrack<Kd>::Generate(_start, _end, size, _seed);

	std::vector<CHDR::Node<T>> result;
	result.reserve(layout.size());
	for (size_t i = 0U; i < layout.size(); ++i) {
		result.emplace_back(layout[i] == RecursiveBacktrack<Kd>::WALL);
	}

	return result;
}

template <typename T, size_t Kd>
static void AStarTest() {

	static_assert(std::is_integral_v<T>, "Type T must be an integral type.");

    try {
		using coord_t = CHDR::Coord<size_t, Kd>;

    	coord_t start {};
    	coord_t end   {};
		coord_t size  { 33U, 33U };

        auto maze = CHDR::Mazes::Grid<Kd, T>(size);
    	maze.Nodes(GenerateMaze<T>(start, end, 0U, size));

		DisplayGridMaze(start, end, size, maze);

    	auto solver = CHDR::Solvers::AStar();
    	//solver.Solve(maze);
    }
    catch (const std::exception& e){
        Debug::Log("ERROR: AStarTest() failed! REASON: " + std::string(e.what()), Error);
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

		Debug::Log("Finalise()", Info);

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

/**
 * @file main.cpp
 * @brief Global entry point.
 */
int main([[maybe_unused]] int _argc, [[maybe_unused]] char* _argv[]) {

    try {

    	Debug::Log("main()", Info);

    	// Set custom termination behaviour:
    	std::set_terminate(&OnTerminate);

    	AStarTest<bool, 2U>();

    	Finalise();
    }
    catch (const std::exception& e) {
        Debug::Log(e, Critical);
    }

    return 0;
}