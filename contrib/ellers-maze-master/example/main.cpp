
#include <cassert>
#include <iostream>
#include <random>
#include <thread>

#include "ellrs/maze.hpp"

int main() {
    constexpr int width = 30;
    std::cout << std::string((width) * 3 + width + 1, '_')
              << '\n'; // first floor
    auto maze = ellrs::maze<width>();
    auto rand_bool =
        [bool_dist = std::uniform_int_distribution<int>{0, 1},
         rand = std::default_random_engine{std::random_device{}()}]() mutable {
            return bool_dist(rand);
        };
    while(true) {
        auto [vtype, vertical] = maze.getline(rand_bool);
        auto [htype, horizontal] = maze.getline(rand_bool);
        assert(vtype == ellrs::line_kind::vertical);
        assert(htype == ellrs::line_kind::horizontal);
        assert(vertical.size() == horizontal.size()
               && vertical.size() == width);

        std::cout << '|'; // start line border
        // print first row of vertical line to make double size ~ >   |
        //                                                      ~ >   |
        for(size_t i = 0; i < vertical.size(); ++i) {
            std::cout << "   " << (vertical[i] == ellrs::WALL ? '|' : ' ');
        }
        std::cout << '\n' << '|';

        for(size_t i{0}; i < width; ++i) {
            auto v_bit = vertical[i];
            auto h_bit = horizontal[i];
            std::cout << (h_bit == ellrs::WALL ? "___" : "   ")
                      << (v_bit == ellrs::WALL ? '|' : ' ');
        }
        std::cout << '\n';

        using namespace std::chrono_literals;

        std::this_thread::sleep_for(1s);
    }

    return 0;
}