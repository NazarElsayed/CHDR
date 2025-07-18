/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef TEST_GPPC_HPP
#define TEST_GPPC_HPP

#include <filesystem>
#include <fstream>

#include <chdr.hpp>
#include <debug.hpp>

#include "utils/backtracking.hpp"

namespace test::generator {

    struct gppc final {

    private:

        static constexpr size_t null_v = static_cast<size_t>(-1U);

    public:

        template <typename weight_t, typename coord_t>
        struct map {

            struct meta {
                std::string name;
                std::string type;
                coord_t size;
            };

            meta metadata;
            chdr::mazes::grid<coord_t, weight_t> maze;

            // ReSharper disable once CppPossiblyUninitializedMember
            [[nodiscard]] constexpr map() noexcept : maze{{}} {} // NOLINT(*-pro-type-member-init, *-use-equals-default)

        };

        template <typename coord_t, typename scalar_t>
        struct scenario {

            coord_t  start;
            coord_t  end;
            scalar_t distance;

            [[nodiscard]] constexpr scenario(coord_t _start, coord_t _end, scalar_t _distance) noexcept :
                start    (_start   ),
                end      (_end     ),
                distance (_distance) {}
        };

        template <typename weight_t, typename coord_t, typename scalar_t>
        static constexpr auto generate(const std::filesystem::path& _map, const std::filesystem::path& _scenarios) {

            static_assert(std::is_integral_v<weight_t>, "Type T must be an integral type.");

            /*
             * See: https://www.movingai.com/benchmarks/formats.html
             */

            debug::log("(GPPC):");

            if (!std::filesystem::exists(_map) || !std::filesystem::exists(_scenarios)) {
                throw std::filesystem::filesystem_error("Invalid file path.", _map, std::make_error_code(std::errc::no_such_file_or_directory));
            }

            std::fstream fs;

            fs.open(_map, std::ios::in);
            if (!fs.is_open()) { throw std::ios_base::failure("Cannot open file."); }

            std::string str;

            /* LOAD MAP */
            map<weight_t, coord_t> map;
            {
                map.metadata.name = _map.filename();
                fs >> str >> map.metadata.type;
                fs >> str >> map.metadata.size[0];
                fs >> str >> map.metadata.size[1];
                fs >> str; // Skip "map" line.

                std::vector<weight_t> maze_data;
                maze_data.reserve(map.metadata.size[0] * map.metadata.size[1]);

                char c;
                while (fs.get(c)) {
                    if (c != '\r' && c != '\n') {
                        maze_data.push_back(
                            c == '.' || c == 'G'?
                                std::numeric_limits<weight_t>::lowest() :
                                std::numeric_limits<weight_t>::max()
                        );
                    }
                }

                fs.close();

                map.maze = chdr::mazes::grid<coord_t, weight_t>(map.metadata.size, std::move(maze_data));
            }

            fs.open(_scenarios, std::ios::in);
            if (!fs.is_open()) { throw std::ios_base::failure("Cannot open file."); }

            /* LOAD SCENARIOS */
            std::vector<scenario<coord_t, scalar_t>> scenarios;
            {
                /*
                 * NOTE: Adapted from official loader for GPPC scenarios.
                 * See: https://github.com/nathansttt/hog2/blob/PDB-refactor/utils/ScenarioLoader.cpp
                 */

                auto version { 0.0F };
                fs >> str;

                // Check for version number in header.
                if (str == "version") {
                    fs >> version;
                }
                else {
                    fs.seekg(0, std::ios::beg);
                }

                size_t bunk;
                size_t  start_x{};
                size_t  start_y{};
                size_t    end_x{};
                size_t    end_y{};
                double distance{};

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wfloat-equal"
#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4389) // '==': signed/unsigned mismatch
#endif

                if (version == 0.0F) {
                    while (fs >> bunk >> str >> start_x >> start_y >> end_x >> end_y >> distance) {
                        scenarios.emplace_back(coord_t { start_x, start_y}, coord_t { end_x, end_y }, distance);
                    }
                }
                else if (version == 1.0F) {
                    while (fs >> bunk >> str >> bunk >> bunk >> start_x >> start_y >> end_x >> end_y >> distance) {
                        scenarios.emplace_back(coord_t { start_x, start_y}, coord_t { end_x, end_y }, distance);
                    }
                }
                else {
                    fs.close();
                    throw std::runtime_error("Unknown version number.");
                }

#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif

                fs.close();
            }

            debug::log("\t[FINISHED] \t(~" + chdr::utils::trim_trailing_zeros(std::to_string(chdr::utils::product<size_t>(map.maze.size()) / 1000000000.0l)) + "b total candidate nodes)");

            return std::pair<gppc::map<weight_t, coord_t>, std::vector<scenario<coord_t, scalar_t>>> {
                std::move(map),
                std::move(scenarios)
            };
        }
    };
} //test::generator

#endif //TEST_GPPC_HPP