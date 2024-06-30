/**
 * @file
 * @brief An Eller's maze generation algorithm.
 */

#ifndef ELLRS_MAZE_HPP
#define ELLRS_MAZE_HPP

#include <bitset>
#include <ranges>
#include <variant>
#include <vector>

namespace ellrs {

/*! @brief A king of the line: vertical or horizontal. */
enum class line_kind { vertical = 0, horizontal = 1 };

/*! @brief Inverts the line */
inline line_kind& operator++(line_kind& l) noexcept {
    l = static_cast<line_kind>(1 - static_cast<int>(l));
    return l;
};

using wall_type = bool;

/*! A type that store an index of cell */
using cell_i = size_t;

// A Container of generated walls
template<cell_i Width>
using line = std::bitset<Width>;

// Improve readability
constexpr wall_type WALL = true;
constexpr wall_type NOT_WALL = false;

// clang-format off
/**
 * @brief Generates the Eller's maze lines.
 * The result of generation just is an array of bits,so you can use it in many
 * cases. A generation of a maze starts with a generation of a vertical line and
 * then a horizontal. The maze generation starts with a vertical line followed 
 * by a horizontal one. However, when using the result, 
 * the horizontal line should be used first
 *                    _1_|_2_|_3_|_4_| 
 *     a horisontal   ~^ ^~ and then a vertical
 */
template<cell_i Width>
    requires(Width > 0)
// clang-format on
class maze {
    using set_i = cell_i; // max(sets) = len(cells)
    enum class next_cell_without_set : std::conditional_t<
        std::is_unsigned_v<cell_i>, std::make_signed_t<cell_i>, cell_i> {
        end_of_list = -1
    };

    // store index to set in sets container
    std::array<std::variant<set_i, next_cell_without_set>, Width>
        cells_and_its_set{};
    // put cells by sets
    using set_type = std::vector<cell_i>;
    std::array<set_type, Width> sets_{};
    std::vector<cell_i> cached_empty_sets_{};
    line_kind current_state_ = line_kind::vertical;

public:
    explicit maze() {
        // Assign the original set to the each cell
        for(auto cell: std::views::iota(static_cast<cell_i>(0),
                                        static_cast<cell_i>(Width))) {
            constexpr int optimal_set_size = Width / 5;
            sets_[cell].reserve(static_cast<size_t>(optimal_set_size));
            push_cell_to_set(cell, cell); // 1 cell -> 1 set, 2 cell -> 2 set
        }
    }

    /**
     * @brief Generate the next maze line alternating between vertically and
     * horizontally.
     *
     * @param rand_bool A random bool generator
     * @return A maze line
     */
    template<typename T>
        requires requires(T rand_bool) {
            { rand_bool() } -> std::convertible_to<bool>;
        }
    [[nodiscard]] std::pair<line_kind, line<Width>> getline(T& rand_bool) {
        auto r =
            std::pair<line_kind, line<Width>>{current_state_, line<Width>{}};
        switch(current_state_) {
        case line_kind::vertical: {
            r.second = gen_v_line(rand_bool);
            break;
        }
        case line_kind::horizontal: {
            r.second = gen_h_line(rand_bool);
            break;
        }
        }
        ++current_state_;
        return r;
    }

#pragma region algorithm_implementation

private:
    template<typename T>
    [[nodiscard]] line<Width> gen_v_line(T& rand_bool) {
        auto res = line<Width>{};
        const auto gen =
            std::views::iota(
                static_cast<cell_i>(0),
                static_cast<cell_i>(Width - 1) /*without a last cell*/)
            | std::views::transform([this, &rand_bool](auto cell) {
                  cell_i next_cell = cell + 1;
                  set_i set = get_cell_set(cell);
                  set_i next_set = get_cell_set(next_cell);
                  if(set == next_set || rand_bool() /* build wall or not*/) {
                      return WALL;
                  }
                  merge_sets(set, next_set);
                  return NOT_WALL;
              });
        for(size_t i = 0; i < res.size() - 1; ++i) {
            res.set(i, static_cast<bool>(gen[i]));
        }
        res.set(res.size() - 1, WALL); // a last right border
        return res;
    }
    template<typename T>
    [[nodiscard]] line<Width> gen_h_line(T& rand_bool) {
        auto res = line<Width>{};
        next_cell_without_set next =
            next_cell_without_set::end_of_list; // using to find changed cells

        for(auto& set: sets_) {
            bool way_exists{false}; // way is already started or not
            bool way_finish{
                false}; // this flag  indicates that we have built ..
            // ..a wall after a certain path because we will no longer need
            // another path

            for(size_t i{0}; i < set.size(); i++) {
                auto cell = set[i];
                if(((i == set.size() - 1) && !way_exists)) {
                    continue; // keep guaranteed way
                }

                if(way_finish || rand_bool() /* build a wall or not */) {
                    // pop cell from it`s set
                    *(set.begin()
                      + static_cast<std::make_signed_t<decltype(i)>>(i)) =
                        pop_back(set); //  swap and pop
                    cells_and_its_set[cell] = next;
                    next = static_cast<next_cell_without_set>(cell);
                    if(way_exists) {
                        way_finish = true;
                    }
                    res.set(cell, WALL);
                } else {
                    way_exists = true;
                }
            }
        }
        // push cells without sets to the unique set
        while(next != next_cell_without_set::end_of_list) {
            if(cached_empty_sets_.empty()) {
                cache_empty_sets();
            }
            auto cell = static_cast<cell_i>(next);
            next = std::get<next_cell_without_set>(cells_and_its_set[cell]);
            push_cell_to_set(pop_back(cached_empty_sets_), cell);
        }
        return res;
    }
#pragma endregion algorithm_implementation

    [[nodiscard]] set_i& get_cell_set(cell_i cell) {
        return std::get<set_i>(cells_and_its_set[cell]);
    }
    void push_cell_to_set(set_i set, cell_i cell) {
        sets_[set].push_back(cell);
        cells_and_its_set[cell] = set;
    }
    void merge_sets(set_i a, set_i to_a) {
        for(cell_i cell: sets_[to_a]) {
            cells_and_its_set[cell] = a;
        }
        sets_[a].insert(sets_[a].end(), sets_[to_a].begin(), sets_[to_a].end());
        sets_[to_a].clear();
    };
    void cache_empty_sets() noexcept {
        for(size_t i{0}; i < sets_.size(); i++) {
            if(sets_[i].empty()) {
                cached_empty_sets_.push_back(i);
            }
        }
    }
    template<typename T>
    auto pop_back(T& container) const noexcept -> typename T::value_type {
        set_i elt = container.back();
        container.pop_back();
        return elt;
    }
};

} // namespace ellrs

#endif
