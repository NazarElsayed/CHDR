/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#include <chdr.hpp>
#include <debug.hpp>

#include <cstddef>
#include <cstdint>
#include <sys/wait.h>
#include <filesystem>
#include <iostream>
#include <string_view>
#include <omp.h>
#include <variant>

#include "core/application.hpp"

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#elif defined(__linux__) || defined(__unix__)
#include <pthread.h>
#include <sched.h>
#endif

namespace test {

    class cli final {

        static constexpr size_t SOLVER             = 1U;
        static constexpr size_t MAZE_WEIGHT        = 2U;
        //static constexpr size_t CLOSED_COMPRESSION = 3U;
        static constexpr size_t X                  = 3U;
        static constexpr size_t Y                  = 4U;
        static constexpr size_t Z                  = 5U;
        static constexpr size_t W                  = 6U;

        static void help() {

            std::cout << "Usage: chdr <command> [arguments]\n"
                      << "\nCommands:\n"
                      << "  <solver> <maze_weight_type> <x>              Process with 1-dimensional coordinate.\n"
                      << "  <solver> <maze_weight_type> <x> <y>          Process with 2-dimensional coordinates.\n"
                      << "  <solver> <maze_weight_type> <x> <y> <z>      Process with 3-dimensional coordinates.\n"
                      << "  <solver> <maze_weight_type> <x> <y> <z> <w>  Process with 4-dimensional coordinates.\n"
                      << "\nSolvers:\n"
                      << "  astar          A*\n"
                      << "  bfs            Breadth-First Search\n"
                      << "  best_first     Best-First Search\n"
                      << "  dfs            Depth-First Search\n"
                      << "  dijkstra       Dijkstra's Algorithm\n"
                      << "  eidastar       Enhanced Iterative-Deepening A*\n"
                      << "  eidbest_first  Enhanced Iterative-Deepening Best-First Search\n"
                      << "  eiddfs         Enhanced Iterative-Deepening Depth-First Search\n"
                      << "  flood          Flood Fill\n"
                      << "  fringe         Fringe Search F*\n"
                      << "  gbest_first    Graveyard Best-First Search\n"
                      << "  gbfs           Graveyard Breadth-First Search\n"
                      << "  gdfs           Graveyard Depth-First Search\n"
                      << "  gjps           Graveyard Jump-Point Search\n"
                      << "  mgstar         Memory-Bounded Graveyard Search\n"
                      << "  smastar        Simplified Memory-Bounded A* Search\n"
                      << "  osmastar       Optimising Simplified Memory-Bounded A* Search\n"
                      << "  gstar          Graveyard Search (G*)\n"
                      << "  idastar        Iterative-Deepening A*\n"
                      << "  idbest_first   Iterative-Deepening Best-First Search\n"
                      << "  iddfs          Iterative-Deepening Depth-First Search\n"
                      << "  jps            Jump-Point Search\n"
                      << "\nMaze Weight Type:\n"
                      << "  bit            Search space represented using 1-bit values.\n"
                      << "  byte           Search space represented using 4-bit values.\n"
                      << "\nExample:\n"
                      << "  chdr astar byte 10\n"
                      << "  chdr astar byte 10 10\n"
                      << "  chdr astar byte 10 10 10\n"
                      << "  chdr astar byte 10 10 10 10\n"
                      << "\nNote: The dimensions must be integers representing coordinates.\n";
        }

        template <template <typename params_t> typename solver_t, typename params_t>
        static int invoke(const params_t& _params) {
            return application::main<solver_t, params_t>(_params);
        }

        template <typename weight_t, typename coord_t, typename scalar_t, typename index_t>
        static chdr::mazes::grid<coord_t, weight_t> make_solvable_random_grid_maze(const coord_t& _start, const coord_t& _end, const coord_t& _size, size_t _seed = 0U, size_t _iterations = std::numeric_limits<size_t>::max()) {

            auto     monotonic = chdr::monotonic_pool();
            auto heterogeneous = chdr::heterogeneous_pool();
            auto   homogeneous = chdr::homogeneous_pool();

            struct params {

                using  weight_type [[maybe_unused]] = weight_t;
                using  scalar_type [[maybe_unused]] = scalar_t;
                using   index_type [[maybe_unused]] =  index_t;
                using   coord_type [[maybe_unused]] =  coord_t;

                using lazy_sorting [[maybe_unused]] = std::true_type;
                using   no_cleanup [[maybe_unused]] = std::false_type;

                const chdr::mazes::grid<coord_t, weight_t>& maze;
                const     coord_type  start;
                const     coord_type  end;
                const     coord_type  size;
                scalar_type  (*h)(const coord_type&, const coord_type&) noexcept;

                decltype(    monotonic)*     monotonic_pmr;
                decltype(heterogeneous)* heterogeneous_pmr;
                decltype(  homogeneous)*   homogeneous_pmr;

                const scalar_type weight       = 1U;
                const      size_t capacity     = 0U;
                const      size_t memoryLimit  = static_cast<size_t>(-1U);
            };

            if (_size.size() >= 2U && _size[0U] > 2U && _size[1U] > 2U) {

                // Generate random 2-KD mazes:
                for (size_t i = 0U; i < _iterations; ++i) {

                    // Seed the random number generator.
                    generator::utils::lcg lcg { _seed + i };

                    constexpr size_t obstacle_threshold = std::numeric_limits<size_t>::max() / 10UL; // 1/10 chance.

                    // Create a random maze using the given threshold for obstacles:
                    std::vector<weight_t> nodes(chdr::utils::product<size_t>(_size));
                    for (size_t j = 1U; j < nodes.size() - 1UL; ++j) {
                        nodes[j] = lcg.operator()() < obstacle_threshold ?
                                       std::numeric_limits<weight_t>::max() :
                                       std::numeric_limits<weight_t>::lowest();
                    }

                    auto result = chdr::mazes::grid<coord_t, weight_t>(_size, nodes);

                    // Check if maze is solvable:
                    auto path = chdr::solvers::solver<chdr::solvers::gbest_first, params>::solve({ result, _start, _end, _size, chdr::heuristics::manhattan_distance<scalar_t, coord_t>, &monotonic, &heterogeneous, &homogeneous });
                    if (!(path.empty())) {
                        std::cout << "Solution depth (d) = " << path.size() << "\n";
                        return result; // Return if solvable.
                    }
                }
            }
            else {
                // Edge case for 0D & 1D mazes.
                return chdr::mazes::grid<coord_t, weight_t>(_size, std::vector<weight_t>(chdr::utils::product<size_t>(_size)));
            }

            throw std::runtime_error("ERROR: Could not create a solvable maze!");
        }

        static void set_highest_thread_priority() {
#if defined(_WIN32) || defined(_WIN64)
            // Windows implementation
            HANDLE thread = GetCurrentThread();
            SetThreadPriority(thread, THREAD_PRIORITY_TIME_CRITICAL);
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__)
            // POSIX (Linux/Unix) implementation
            int policy;
            sched_param param;

            auto thread = pthread_self();
            if (pthread_getschedparam(thread, &policy, &param) != 0) {
                return;
            }

            policy = SCHED_FIFO;
            param.sched_priority = sched_get_priority_max(policy);
            pthread_setschedparam(thread, policy, &param);
#endif
        }

        template <typename instanced_solver_t, typename params_t>
        static auto invoke_benchmark(const params_t& _params) {

            auto min_duration = std::numeric_limits<long double>::max();
            auto path_length = 0UL;

            try {

                set_highest_thread_priority();

                /* TEST SAMPLES */
    #ifndef NDEBUG
                constexpr size_t base_samples = 1UL;
    #else //!NDEBUG
                constexpr size_t base_samples = 100000000UL;
    #endif //!NDEBUG

                size_t test_samples = chdr::utils::max(base_samples / _params.maze.count(), static_cast<size_t>(1U));

                // Graphs are generally more sparse than grids, require fewer samples:
                if constexpr (std::is_same_v<
                    std::decay_t<decltype(_params.maze)>,
                    chdr::mazes::graph<typename params_t::index_type, typename params_t::scalar_type>>
                ) {
                    test_samples = chdr::utils::sqrt(test_samples);
                }

                /* CAPTURE SYSTEM NOISE */
                auto noise_floor_min = std::numeric_limits<long double>::max();
                for (size_t i = 0U; i < test_samples; ++i) {

                    const auto sw_start = std::chrono::high_resolution_clock::now();

                    noise_floor_min = chdr::utils::min(
                        noise_floor_min,
                        std::chrono::duration_cast<std::chrono::duration<long double>>(
                            std::chrono::high_resolution_clock::now() - sw_start
                        ).count()
                    );
                }

                /* TEST ALGORITHM */
                chdr::malloc_consolidate();

                decltype(instanced_solver_t::solve(_params)) path;

                for (size_t i = 0U; i < test_samples; ++i) {

                    const auto sw_start = std::chrono::high_resolution_clock::now();

                    /* INVOKE SOLVE */
                    path = instanced_solver_t::solve(_params);

                    min_duration = chdr::utils::min(
                        min_duration,
                        std::chrono::duration_cast<std::chrono::duration<long double>>(
                            std::chrono::high_resolution_clock::now() - sw_start
                        ).count()
                    );

                    if (i != test_samples - 1U) {
                        _params.    monotonic_pmr->reset();
                        _params.heterogeneous_pmr->reset();
                        _params.  homogeneous_pmr->reset();
                    }
                }

                min_duration = chdr::utils::max(min_duration - noise_floor_min, std::numeric_limits<long double>::epsilon());
                path_length  = path.size();
            }
            catch (const std::exception& e) {
                min_duration = std::numeric_limits<long double>::max();

                debug::log(e);
            }

            return std::make_pair(min_duration, path_length);
        }

        template <typename weight_t, typename coord_t, typename scalar_t, typename index_t>
        static auto run_gppc_benchmarks() {

            struct params {

                using weight_type [[maybe_unused]] = weight_t;
                using scalar_type [[maybe_unused]] = scalar_t;
                using  index_type [[maybe_unused]] = index_t;
                using  coord_type [[maybe_unused]] = coord_t;

                using lazy_sorting [[maybe_unused]] = std::false_type;
                using   no_cleanup [[maybe_unused]] = std::true_type;

                const decltype(generator::gppc::map<weight_t, coord_t>::maze)& maze;

                const coord_type start;
                const coord_type end;
                const coord_type size;

                scalar_type (*h)(const coord_type&, const coord_type&) noexcept;

                chdr::    monotonic_pool<>*     monotonic_pmr;
                chdr::heterogeneous_pool<>* heterogeneous_pmr;
                chdr::  homogeneous_pool<>*   homogeneous_pmr;

                const scalar_type weight       = 1U;
                const size_t      capacity     = 0U;
                const size_t      memory_limit = 0U;
            };

            using variant_t = std::variant<
                chdr::solvers::solver<chdr::solvers::        astar, params>,
                chdr::solvers::solver<chdr::solvers::   best_first, params>,
                chdr::solvers::solver<chdr::solvers::          bfs, params>,
                chdr::solvers::solver<chdr::solvers::          dfs, params>,
                chdr::solvers::solver<chdr::solvers::     dijkstra, params>,
                chdr::solvers::solver<chdr::solvers::     eidastar, params>,
                chdr::solvers::solver<chdr::solvers::eidbest_first, params>,
                chdr::solvers::solver<chdr::solvers::       eiddfs, params>,
                chdr::solvers::solver<chdr::solvers::        flood, params>,
                chdr::solvers::solver<chdr::solvers::       fringe, params>,
                chdr::solvers::solver<chdr::solvers::  gbest_first, params>,
                chdr::solvers::solver<chdr::solvers::         gbfs, params>,
                chdr::solvers::solver<chdr::solvers::         gdfs, params>,
                chdr::solvers::solver<chdr::solvers::         gjps, params>,
                chdr::solvers::solver<chdr::solvers::        gstar, params>,
                chdr::solvers::solver<chdr::solvers::      idastar, params>,
                chdr::solvers::solver<chdr::solvers:: idbest_first, params>,
                chdr::solvers::solver<chdr::solvers::        iddfs, params>,
                chdr::solvers::solver<chdr::solvers::          jps, params>,
                chdr::solvers::solver<chdr::solvers::       mgstar, params>,
                chdr::solvers::solver<chdr::solvers::     osmastar, params>,
                chdr::solvers::solver<chdr::solvers::      smastar, params>>;

            struct test {
                const std::string name;
                const variant_t   variant;
            };

            #define MAKE_TEST_VARIANT(name) test { #name, variant_t { std::in_place_type<chdr::solvers::solver<chdr::solvers::name, params>> } }

            // Comment or uncomment as needed:
            const std::array tests {
                MAKE_TEST_VARIANT(        astar),
                // MAKE_TEST_VARIANT(   best_first),
                // MAKE_TEST_VARIANT(          bfs),
                // MAKE_TEST_VARIANT(          dfs),
                // MAKE_TEST_VARIANT(     dijkstra),
                MAKE_TEST_VARIANT(     eidastar),
                // MAKE_TEST_VARIANT(eidbest_first),
                // MAKE_TEST_VARIANT(       eiddfs),
                // MAKE_TEST_VARIANT(        flood),
                // MAKE_TEST_VARIANT(       fringe),
                // MAKE_TEST_VARIANT(  gbest_first),
                // MAKE_TEST_VARIANT(         gbfs),
                // MAKE_TEST_VARIANT(         gdfs),
                // MAKE_TEST_VARIANT(         gjps),
                // MAKE_TEST_VARIANT(        gstar),
                // MAKE_TEST_VARIANT(      idastar),
                // MAKE_TEST_VARIANT( idbest_first),
                // MAKE_TEST_VARIANT(        iddfs),
                MAKE_TEST_VARIANT(          jps),
                MAKE_TEST_VARIANT(       mgstar),
                // MAKE_TEST_VARIANT(     osmastar),
                MAKE_TEST_VARIANT(      smastar)
            };

            #undef MAKE_TEST_VARIANT

            // Find the longest solver's name (for text alignment in readouts):
            size_t longest_solver_name { 0U };
            for (const auto& [name, variant] : tests) {
                longest_solver_name = std::max(longest_solver_name, name.size());
            }

            const auto gppc_dir = std::filesystem::current_path() / "gppc";
            if (std::filesystem::exists(gppc_dir)) {

                const auto log_dir = std::filesystem::current_path() / "log" / CHDR_VERSION;
                if (!std::filesystem::exists(log_dir)) {
                    std::filesystem::create_directories(log_dir);
                }

                size_t tests_completed { 0U };
                size_t total_tests     { 0U };

                // Identify all benchmarking maps in advance:
                std::vector<std::pair<generator::gppc::map<weight_t, coord_t>, std::vector<generator::gppc::scenario<coord_t, scalar_t>>>> maps;

                // Ensure the directory exists.
                if (std::filesystem::exists(gppc_dir)) {
                    for (const auto& entry : std::filesystem::recursive_directory_iterator(gppc_dir, std::filesystem::directory_options::skip_permission_denied)) {

                        if (entry.is_regular_file() && entry.path().extension() == ".map") {
                            const auto scenarios_path = std::filesystem::path{ entry.path().string() + ".scen" };
                            maps.push_back(generator::gppc::generate<weight_t, coord_t, scalar_t>(entry, scenarios_path));

                            total_tests += maps.back().second.size() * tests.size();
                        }
                    }
                }
                else {
                    throw std::filesystem::filesystem_error("Directory not found.", gppc_dir, std::error_code());
                }

                std::cout << "~ Running Diagnostics (GPPC) ~\n"
                          << "OMP: "
                #ifdef _OPENMP
                          << "ENABLED.\n";
                #else //!_OPENMP
                          << "DISABLED.\n";
                #endif //!_OPENMP

                constexpr auto log_interval = std::chrono::seconds(3UL);
                auto next_log = std::chrono::high_resolution_clock::now() + log_interval;

                constexpr auto local_window_size = std::chrono::seconds(10UL);
                auto  global_window_begin = std::chrono::high_resolution_clock::now();
                auto   local_window_begin = std::chrono::high_resolution_clock::now();
                size_t local_window_tests { 0U };

                struct test_data {
                    long double duration;
                    size_t      memory;
                };

                std::pmr::synchronized_pool_resource pmr;
                std::pmr::unordered_map<std::string, std::pmr::map<long double, std::pmr::vector<test_data>>> global_averages(&pmr);

                constexpr auto timeout = std::chrono::seconds(30UL);
                std::pmr::unordered_map<std::string, scalar_t> max_distances(&pmr);
                for (const auto& [name, variant] : tests) {
                    max_distances[name] = 2048UL;
                }

                /* PARALLELISED BENCHMARKING */
                for (const auto& [map, scenarios] : maps) {

                    try {

                        std::ofstream log(log_dir / (map.metadata.name + ".log"));
                        log << std::fixed << std::setprecision(9)
                            << "~ Running Diagnostics (GPPC) ~\n"
                            << "FORMAT: [SECONDS, BYTES]\n"
                            << "MAP: \"" << map.metadata.name << "\":\n";

                        try {

                            std::array<test_data, tests.size()> test_averages{};

                            #pragma omp parallel for schedule(guided) // Parallelise per-scenario (inner loop).
                            for (size_t index = 0U; index < scenarios.size(); ++index) {

                                size_t i = index; //(scenarios.size() - 1U) - index;

                                const auto& scenario = scenarios[i];

                                // Each thread builds an in-memory log BEFORE writing.
                                std::ostringstream thread_log;
                                thread_log << std::fixed << std::setprecision(9)
                                           << "Scenario " << (i + 1U) << " (Length: " << scenario.distance << "):\n";

                                for (size_t j = 0U; j < tests.size(); ++j)
                                {
                                    const auto& [name, variant] = tests[j];

                                    // Right-aligned output:
                                    thread_log << "\t";
                                    for (size_t k = 0U; k < longest_solver_name - name.size(); ++k) {
                                        thread_log << " ";
                                    }
                                    thread_log << name << ": ";

                                    if (scenario.distance >= max_distances[name]) {
                                        thread_log << "SKIPPED\n";
                                    }
                                    else {

                                        struct result_data {
                                            std::pair<long double, size_t> path_runtime;
                                            size_t peak_memory;
                                        };

                                        result_data data{{ -1.0L, 0U }, { 0U }};
                                        {
                                            int pipefd[2];
                                            if (pipe(pipefd) == -1) {
                                                perror("pipe");
                                            }

                                            pid_t child_pid = fork();
                                            if (child_pid == 0) {
                                                close(pipefd[0]);

                                                auto monotonic     = chdr::monotonic_pool();
                                                auto heterogeneous = chdr::heterogeneous_pool();
                                                auto homogeneous   = chdr::homogeneous_pool();

                                    auto [duration, length] = std::visit([&](const auto& _t) {
                                        return invoke_benchmark<std::decay_t<decltype(_t)>>(
                                            params{
                                                map.maze,
                                                scenario.start,
                                                scenario.end,
                                                map.metadata.size,
                                                &chdr::heuristics::manhattan_distance<scalar_t, coord_t>,
                                                &monotonic,
                                                &heterogeneous,
                                                &homogeneous,
                                                1U,
                                                0U,
                                                (map.metadata.size[0U] / 2U) * (map.metadata.size[1U] / 2U)
                                            });
                                        },
                                        variant
                                    );

    #if CHDR_DIAGNOSTICS == 1
                                                data.peak_memory = monotonic.__get_diagnostic_data().peak_allocated +
                                                               heterogeneous.__get_diagnostic_data().peak_allocated +
                                                                 homogeneous.__get_diagnostic_data().peak_allocated;
    #endif //CHDR_DIAGNOSTICS == 1

                                                write(pipefd[1], &data, sizeof(data));
                                                close(pipefd[1]);
                                                exit(0);
                                            }
                                            else if (child_pid > 0) {
                                                close(pipefd[1]);
                                                auto future = std::async(std::launch::async, [&]() {
                                                    int status;
                                                    waitpid(child_pid, &status, 0);
                                                    return (WIFEXITED(status) && WEXITSTATUS(status) == 0);
                                                });

                                                if (future.wait_for(timeout) == std::future_status::ready && future.get()) {
                                                    read(pipefd[0], &data, sizeof(data));
                                                }
                                                else {
                                                    kill(child_pid, SIGKILL);
                                                }

                                                close(pipefd[0]);
                                            }
                                        }

                                        const auto& [duration, length] = data.path_runtime;

                                        if (duration < 0.0L) {
                                            if (scenario.distance >= max_distances[name]) {
                                                max_distances[name] /= static_cast<scalar_t>(2.0L);
                                            }
                                            thread_log << " ERR: TIMEOUT\n";
                                        }
                                        else {

                                            thread_log << " " << duration << ", " << data.peak_memory << "\n";

                                            #pragma omp atomic
                                            test_averages[j].duration += duration;

                                            #pragma omp atomic
                                            test_averages[j].memory += data.peak_memory;

                                            std::pmr::vector<test_data>* bin = nullptr;
                                            {
                                                std::pmr::map<long double, std::pmr::vector<test_data>>* distance_map;

                                                auto s1 = global_averages.find(name);
                                                if (s1 == global_averages.end())
                                                {
                                                    #pragma omp critical
                                                    distance_map = &(
                                                        global_averages.try_emplace(name, std::pmr::map<long double, std::pmr::vector<test_data>>(&pmr)).first->second
                                                    );
                                                }
                                                else {
                                                    distance_map = &(s1->second);
                                                }

                                                auto s2 = distance_map->find(scenario.distance);
                                                if (s2 == distance_map->end()) {
                                                    #pragma omp critical
                                                    bin = &(
                                                        distance_map->try_emplace(scenario.distance, std::pmr::vector<test_data>(&pmr))
                                                    ).first->second;
                                                }
                                                else {
                                                    bin = &(s2->second);
                                                }
                                            }

                                            bin->push_back(test_data { duration, data.peak_memory });
                                        }
                                    }
                                    #pragma omp atomic
                                    ++tests_completed;
                                }

                                #pragma omp critical
                                {
                                    log << thread_log.str();

                                    auto now = std::chrono::high_resolution_clock::now();
                                    if (now >= next_log || tests_completed == total_tests) {
                                        next_log = now + log_interval;

                                        const auto progress = static_cast<long double>(tests_completed) / static_cast<long double>(total_tests);

                                        std::stringstream percentage;
                                        percentage << std::fixed << std::setprecision(2)
                                                   << (progress * 100.0L) << "%";

                                        size_t secs { 0U };
                                        {
                                            size_t  local_secs { 0U };
                                            size_t global_secs { 0U };

                                            { // Local window:

                                                const auto window_progress =
                                                    static_cast<long double>(tests_completed - local_window_tests) /
                                                        static_cast<long double>(total_tests - local_window_tests);

                                                const auto delta     = std::chrono::duration_cast<std::chrono::seconds>(now - local_window_begin);
                                                const auto end       = local_window_begin + (delta * (1.0L / window_progress));
                                                const auto remaining = std::chrono::duration_cast<std::chrono::seconds>(end - now);

                                                if (std::chrono::duration_cast<std::chrono::seconds>(now - local_window_begin) > local_window_size) {
                                                    local_window_begin = now;
                                                    local_window_tests = tests_completed;
                                                }

                                                local_secs = static_cast<size_t>(remaining.count());
                                            }
                                            { // Global window:

                                                const auto window_progress =
                                                    static_cast<long double>(tests_completed) /
                                                        static_cast<long double>(total_tests);

                                                const auto delta     = std::chrono::duration_cast<std::chrono::seconds>(now - global_window_begin);
                                                const auto end       = global_window_begin + (delta * (1.0L / window_progress));
                                                const auto remaining = std::chrono::duration_cast<std::chrono::seconds>(end - now);

                                                global_secs = static_cast<size_t>(remaining.count());
                                            }

                                            secs = static_cast<size_t>(0.7L * static_cast<long double>( local_secs)) +
                                                   static_cast<size_t>(0.3L * static_cast<long double>(global_secs));
                                        }

                                        const auto years   = secs / 31536000UL; secs %= 31536000UL;
                                        const auto days    = secs / 86400UL;    secs %= 86400UL;
                                        const auto hours   = secs / 3600UL;     secs %= 3600UL;
                                        const auto minutes = secs / 60UL;       secs %= 60UL;

                                        std::stringstream time_remaining;
                                        if (years   != 0UL) { time_remaining << years   << "y "; }
                                        if (days    != 0UL) { time_remaining << days    << "d "; }
                                        if (hours   != 0UL) { time_remaining << hours   << "h "; }
                                        if (minutes != 0UL) { time_remaining << minutes << "m "; }
                                                              time_remaining << secs    << "s";

                                        debug::log(
                                            std::to_string(tests_completed) + " / " + std::to_string(total_tests) + " (" + percentage.str() + ") " +
                                            time_remaining.str() + " remaining."
                                        );
                                    }
                                }
                            }

                            log << "\nMAP COMPLETED.\nAVERAGES:\n";

                            for (size_t j = 0U; j < tests.size(); ++j) {
                                const auto& [name, variant] = tests[j];

                                // Right-aligned output:
                                for (size_t k = 0U; k < longest_solver_name - name.size(); ++k) {
                                    log << " ";
                                }
                                log << name << ":\n"
                                    << "\t RUNTIME    (s): " << test_averages[j].duration / scenarios.size() << "\n"
                                    << "\t MEMORY (bytes): " << test_averages[j].memory   / scenarios.size() << "\n";
                            }
                        }
                        catch (const std::exception& e) {
                            log << e.what() << "\n";
                            debug::log(e, critical);
                        }
                    }
                    catch (const std::exception& e) {
                        debug::log(e, critical);
                    }
                }

                std::ofstream log(log_dir / "summary.log");

                std::cout << "GLOBAL AVERAGES:\n" << std::fixed << std::setprecision(9);
                      log << "GLOBAL AVERAGES:\n" << std::fixed << std::setprecision(9);

                for (auto& [name, distances] : global_averages) {
                    std::cout << name << ":\n";
                          log << name << ":\n";

                    std::map<std::pair<size_t, size_t>, std::tuple<long double, size_t, size_t>> binned_averages;

                    size_t lower_bin = 0, upper_bin = 1;
                    for (const auto& [distance, entries] : distances) {

                        while (distance > upper_bin) {
                            lower_bin = upper_bin;
                            upper_bin *= 2;
                        }

                        auto& [total_duration, total_memory, total_count] = binned_averages[{ lower_bin, upper_bin }];
                        for (const auto& entry : entries) {
                            total_duration += entry.duration;
                            total_memory   += entry.memory;
                            total_count++;
                        }
                    }

                    for (const auto& [bin, data] : binned_averages) {
                        const auto [l, u] = bin;

                        const auto& [total_duration, total_memory, total_count] = data;

                        const auto avg_duration = total_duration / total_count;
                        const auto avg_memory   = static_cast<long double>(total_memory) / total_count;

                        std::cout << "\tDISTANCE BIN [" << l << ", " << u << "]:\n"
                                  << "\t  AVERAGE RUNTIME  (s): "     << avg_duration << "\n"
                                  << "\t  AVERAGE MEMORY   (bytes): " << avg_memory << "\n";

                        log << "\tDISTANCE BIN [" << l << ", " << u << "]:\n"
                            << "\t  AVERAGE RUNTIME  (s): "     << avg_duration << "\n"
                            << "\t  AVERAGE MEMORY   (bytes): " << avg_memory << "\n";
                    }
                }
            }

            return EXIT_SUCCESS;
        }

        template <typename weight_t, typename coord_t>
        static int deduce_solver(const std::string_view& _solver, const coord_t& _size) {

            auto result = EXIT_FAILURE;

            using scalar_t = uint32_t;
            using  index_t = uint32_t;

            //return run_gppc_benchmarks<weight_t, coord_t, scalar_t, index_t>();

            /* GENERATE MAZE */
            constexpr size_t seed { 0U };

            const coord_t start {};
                  coord_t end;

            for (size_t i = 0U; i < _size.size(); ++i) {
                end[i] = _size[i] - 1U;
            }

            // Empty grid:
            // const std::vector<weight_t> nodes(chdr::utils::product<size_t>(_size), std::numeric_limits<weight_t>::lowest());
            // const auto grid = chdr::mazes::grid<coord_t, weight_t>(_size, nodes);
            //debug::log(_size[0] + _size[1] - 2U);

            // Random grid:
            //const auto grid = make_solvable_random_grid_maze<weight_t, coord_t, scalar_t, index_t>(start, end, _size, seed);

            // Maze grid:
            const auto grid = generator::grid::generate<weight_t>(start, end, _size, 0.0, 0.0, seed);

            const auto& test = grid;
            // auto graph_pool = chdr::heterogeneous_pool(); const auto test = chdr::mazes::graph<index_t, scalar_t>(grid, &graph_pool);
            // const auto test = generator::graph::generate<weight_t, index_t, coord_t, scalar_t>(start, end, size, seed);

            auto     monotonic = chdr::monotonic_pool();
            auto heterogeneous = chdr::heterogeneous_pool();
            auto   homogeneous = chdr::homogeneous_pool();

            struct params {

                using  weight_type [[maybe_unused]] = weight_t;
                using  scalar_type [[maybe_unused]] = scalar_t;
                using   index_type [[maybe_unused]] =  index_t;
                using   coord_type [[maybe_unused]] =  coord_t;

                using lazy_sorting [[maybe_unused]] = std::false_type;
                using   no_cleanup [[maybe_unused]] = std:: true_type;

                const decltype(test)& maze;
                const     coord_type  start;
                const     coord_type  end;
                const     coord_type  size;
                         scalar_type  (*h)(const coord_type&, const coord_type&) noexcept;

                decltype(    monotonic)*     monotonic_pmr;
                decltype(heterogeneous)* heterogeneous_pmr;
                decltype(  homogeneous)*   homogeneous_pmr;

                const scalar_type weight       =  1U;
                const      size_t capacity     =  0U;
                const      size_t memory_limit = -1U;
            };

            const params args { test, start, end, _size, chdr::heuristics::manhattan_distance<scalar_t, coord_t>, &monotonic, &heterogeneous, &homogeneous };

                 if (_solver == "astar"        ) { result = invoke<chdr::solvers::        astar, params>(args); }
            else if (_solver == "best_first"   ) { result = invoke<chdr::solvers::   best_first, params>(args); }
            else if (_solver == "bfs"          ) { result = invoke<chdr::solvers::          bfs, params>(args); }
            else if (_solver == "dfs"          ) { result = invoke<chdr::solvers::          dfs, params>(args); }
            else if (_solver == "dijkstra"     ) { result = invoke<chdr::solvers::     dijkstra, params>(args); }
            else if (_solver == "eidastar"     ) { result = invoke<chdr::solvers::     eidastar, params>(args); }
            else if (_solver == "eidbest_first") { result = invoke<chdr::solvers::eidbest_first, params>(args); }
            else if (_solver == "eiddfs"       ) { result = invoke<chdr::solvers::       eiddfs, params>(args); }
            else if (_solver == "flood"        ) { result = invoke<chdr::solvers::        flood, params>(args); }
            else if (_solver == "fringe"       ) { result = invoke<chdr::solvers::       fringe, params>(args); }
            else if (_solver == "gbest_first"  ) { result = invoke<chdr::solvers::  gbest_first, params>(args); }
            else if (_solver == "gbfs"         ) { result = invoke<chdr::solvers::         gbfs, params>(args); }
            else if (_solver == "gdfs"         ) { result = invoke<chdr::solvers::         gdfs, params>(args); }
            else if (_solver == "gjps"         ) { result = invoke<chdr::solvers::         gjps, params>(args); }
            else if (_solver == "gstar"        ) { result = invoke<chdr::solvers::        gstar, params>(args); }
            else if (_solver == "idastar"      ) { result = invoke<chdr::solvers::      idastar, params>(args); }
            else if (_solver == "idbest_first" ) { result = invoke<chdr::solvers:: idbest_first, params>(args); }
            else if (_solver == "iddfs"        ) { result = invoke<chdr::solvers::        iddfs, params>(args); }
            else if (_solver == "jps"          ) { result = invoke<chdr::solvers::          jps, params>(args); }
            else if (_solver == "mgstar"       ) { result = invoke<chdr::solvers::       mgstar, params>(args); }
            else if (_solver == "osmastar"     ) { result = invoke<chdr::solvers::     osmastar, params>(args); }
            else if (_solver == "smastar"      ) { result = invoke<chdr::solvers::      smastar, params>(args); }
            else {
                debug::log("ERROR: Unknown solver \"" + std::string(_solver) + "\"!", error);
            }

#if CHDR_DIAGNOSTICS == 1

            auto peak_memory_usage = monotonic.__get_diagnostic_data().peak_allocated +
                                 heterogeneous.__get_diagnostic_data().peak_allocated +
                                   homogeneous.__get_diagnostic_data().peak_allocated;

            std::cout << peak_memory_usage << "\n";

#endif //CHDR_DIAGNOSTICS == 1

            return result;
        }

        template <typename coord_t>
        static int deduce_weight(int _argc, const char* const _argv[], const coord_t& _coord) {

            auto result = EXIT_FAILURE;

            if (_argc > 0 && static_cast<size_t>(_argc) > MAZE_WEIGHT) {

                const std::string maze_format { _argv[MAZE_WEIGHT] };

                     if (maze_format == "bit"   ) { result = deduce_solver<bool>    ( { _argv[SOLVER] }, _coord); }
                else if (maze_format == "byte"  ) { result = deduce_solver<char>    ( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "int32" ) { result = deduce_solver<int32_t> ( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "uint32") { result = deduce_solver<uint32_t>( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "int64" ) { result = deduce_solver<int32_t> ( { _argv[SOLVER] }, _coord); }
                //else if (maze_format == "uint64") { result = deduce_solver<uint32_t>( { _argv[SOLVER] }, _coord); }
                else {
                    debug::log("ERROR: Unknown maze format \"" + std::string(maze_format) + "\"!", error);
                }
            }

            return result;
        }

        static int deduce_coord(int _argc, const char* const _argv[]) {

            auto result = EXIT_FAILURE;

            using index_t = uint32_t;

                 // if (static_cast<size_t>(_argc - 1) == X) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 1U> { static_cast<index_t>(std::stoul(_argv[X])) }); }
            if (static_cast<size_t>(_argc - 1) == Y) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 2U> { static_cast<index_t>(std::stoul(_argv[X])), static_cast<index_t>(std::stoul(_argv[Y])) }); }
            // else if (static_cast<size_t>(_argc - 1) == Z) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 3U> { static_cast<index_t>(std::stoul(_argv[X])), static_cast<index_t>(std::stoul(_argv[Y])), static_cast<index_t>(std::stoul(_argv[Z])) }); }
            // else if (static_cast<size_t>(_argc - 1) == W) { result = deduce_weight(_argc, _argv, chdr::coord<index_t, 4U> { static_cast<index_t>(std::stoul(_argv[X])), static_cast<index_t>(std::stoul(_argv[Y])), static_cast<index_t>(std::stoul(_argv[Z])), static_cast<index_t>(std::stoul(_argv[W])) }); }
            else {
                debug::log("ERROR: Invalid Dimensionality!", error);
            }

            return result;
        }

    public:

        static int main(const int _argc, const char* const _argv[]) {

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
int main(const int _argc, const char* const _argv[]) noexcept {

    auto result = EXIT_FAILURE;

    try {
        debug::log("CHDR " CHDR_VERSION);
        debug::log("Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson");
        debug::log("Licensed under CC BY-NC-ND 4.0");

        debug::log("main()", info);

        result = test::cli::main(_argc, _argv);
    }
    catch(...) {} //NOLINT(*-empty-catch)

    return result;
}