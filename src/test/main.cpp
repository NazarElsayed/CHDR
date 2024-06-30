#include <chdr.hpp>

#include <Debug.hpp>

#include "RecursiveBacktrack.hpp"

#include <iomanip>

template <typename T, size_t Kd>
static constexpr void DisplayMaze(const CHDR::Coord<size_t, Kd>& _start, const CHDR::Coord<size_t, Kd>& _end, const CHDR::Coord<size_t, Kd>& _size, const std::vector<CHDR::Node<T>>& _maze) {

	static_assert(               Kd < 3, "This maze renderer does not support dimensions higher than 2.");
	static_assert(std::is_integral_v<T>, "Template parameter T must be an integral type."               );

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

	for (size_t i = 0U; i < _maze.size(); ++i) {

		if (i % _size[0U] == 0U) { oss << wall_str; }

		auto s = CHDR::Utils::To1D(_start, _size);
		auto e = CHDR::Utils::To1D(_end,   _size);

		     if (i == s) { oss << "St"; }
		else if (i == e) { oss << "En"; }
		else {

			auto val = _maze[i].Value();

			if constexpr (std::is_same_v<T, bool>) {
				oss << (val ? wall_str : path_str);
			}
			else {
				oss << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(val);
			}

			// Handle end of line:
			if constexpr (Kd == 1) {

				const bool end_of_line = i == _size[0U] - 1U;
				if (end_of_line) {

					if (!even_width) {
						oss << wall_str;
					}
					oss << line_brk;
				}
			}
			else {

				const bool end_of_line = (i + 1U) % _size[0U] == 0U;
				if (end_of_line) {

					if (!even_width) {
						oss << wall_str;
					}
					oss << line_brk;
				}
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

	static_assert(std::is_integral_v<T>, "Type T must be an integer type.");

	std::array size { _size... };

	auto layout = RecursiveBacktrack<Kd>::Generate(_start, _end, size, _seed);

	std::vector<CHDR::Node<T>> result;
	result.reserve(layout.size());
	for (size_t i = 0U; i < layout.size(); ++i) {
		result.emplace_back(layout[i] == RecursiveBacktrack<Kd>::WALL);
	}

	return result;
}

static void AStarTest() {

    try {
		using coord_t = CHDR::Coord<std::size_t, 2U>;

		coord_t dimensions { 33U, 33U };

        auto maze = CHDR::Mazes::Grid(dimensions);
        auto solver = CHDR::Solvers::AStar();

    	coord_t start{};
		coord_t end  {};

		auto vals = GenerateMaze<bool>(start, end, 0U, maze.Size());

		DisplayMaze(start, end, dimensions, vals);

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