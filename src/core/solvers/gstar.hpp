/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GSTAR_HPP
#define CHDR_GSTAR_HPP

#include <cassert>

#include "mazes/base/imaze.hpp"
#include "mazes/graph.hpp"
#include "mazes/grid.hpp"
#include "types/existence_set.hpp"
#include "types/heap.hpp"
#include "utils/utils.hpp"

namespace chdr::solvers {

    template<size_t Kd, typename scalar_t, typename index_t, typename params_t>
    struct [[maybe_unused]] gstar final {

        friend struct solver<gstar, Kd, scalar_t, index_t, params_t>;

        static_assert(std::is_integral_v<scalar_t> || std::is_floating_point_v<scalar_t>, "scalar_t must be either an integral or floating point type");
        static_assert(std::is_integral_v<index_t>, "index_t must be an integral type.");

    private:

        using solver_t = solver<gstar, Kd, scalar_t, index_t, params_t>;
        using  coord_t = coord<index_t, Kd>;

        struct node final {

            index_t m_index;

            scalar_t m_gScore;
            scalar_t m_fScore;

            node* m_parent;

            unsigned char m_successors;

            /**
             * @brief Constructs an uninitialized ASNode.
             *
             * This constructor creates an ASNode with uninitialized members.
             */
            [[nodiscard]] constexpr node() noexcept : // NOLINT(*-pro-type-member-init, *-use-equals-default)
                m_parent     (nullptr),
                m_successors (0U     ) {}

            [[nodiscard]] constexpr node(const index_t& _index, const scalar_t& _gScore, const scalar_t& _hScore, node* RESTRICT const _parent = nullptr) noexcept :
                m_index      (_index           ),
                m_gScore     (_gScore          ),
                m_fScore     (_gScore + _hScore),
                m_parent     (_parent          ),
                m_successors (0U               )
            {
                incr();
            }

            [[nodiscard]] constexpr node(const node& _other) noexcept :
                m_index      (_other.m_index     ),
                m_gScore     (_other.m_gScore    ),
                m_fScore     (_other.m_fScore    ),
                m_parent     (_other.m_parent    ),
                m_successors (_other.m_successors)
            {
                incr();
            }

            node& operator=(const node& _other) noexcept {

                assert(this != &_other);

                expunge();

                m_index      = _other.m_index;
                m_gScore     = _other.m_gScore;
                m_fScore     = _other.m_fScore;
                m_parent     = _other.m_parent;
                m_successors = _other.m_successors;

                return *this;
            }
            
            [[nodiscard]] constexpr node(node&& _other) noexcept :
                m_index      (_other.m_index     ),
                m_gScore     (_other.m_gScore    ),
                m_fScore     (_other.m_fScore    ),
                m_parent     (_other.m_parent    ),
                m_successors (_other.m_successors)
            {
                _other.m_parent     = nullptr;
                _other.m_successors = 0U;
            }

            node& operator=(node&& _other) noexcept {

                assert(this != &_other);

                expunge();

                m_index      = std::move(_other.m_index);
                m_gScore     = _other.m_gScore;
                m_fScore     = _other.m_fScore;
                m_parent     = _other.m_parent;
                m_successors = _other.m_successors;

                _other.m_parent     = nullptr;
                _other.m_successors = 0U;

                return *this;
            }

            ~node() noexcept {
                expunge();
            }

            void expunge() noexcept {

                while (m_parent != nullptr) {

                    decr();

                    if (m_parent->m_successors == 0U) {

                        auto* temp = m_parent;
                        m_parent = temp->m_parent;

                        temp->m_parent = nullptr;
                        delete temp;
                    }
                    else {
                        break;
                    }
                }
            }

            void decr() noexcept {

                if (m_parent != nullptr && m_parent->m_successors > 0U) {
                    m_parent->m_successors -= 1U;
                }
            }

            void incr() noexcept {
                if (m_parent != nullptr) {
                    m_parent->m_successors += 1U;
                }
            }

            template<typename node_t, typename coord_t>
            auto backtrack(const coord_t& _size, const size_t& _capacity = 0U) const {

                // Reserve space in result:
                std::vector<coord_t> result;
                result.reserve(_capacity);

                // Recurse from end node to start node, inserting into a result buffer:
                for (const auto* t = this; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent)) {
                    result.emplace_back(utils::to_nd(t->m_index, _size));
                }

                // Reverse the result:
                std::reverse(result.begin(), result.end());

                return result;
            }

            [[nodiscard]] friend constexpr bool operator < (const node& _a, const node& _b) noexcept {
                return _a.m_fScore == _b.m_fScore ?
                       _a.m_gScore >  _b.m_gScore :
                       _a.m_fScore >  _b.m_fScore;
            }
        };

        template <typename open_set_t, typename closed_set_t, typename buf_t>
        [[maybe_unused, nodiscard]] static constexpr auto solve_internal(open_set_t& _open, closed_set_t& _closed, buf_t& _buf, const size_t& _capacity, const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);
            const auto e = utils::to_1d(_params.end,   _params.size);

            _open.emplace(s, static_cast<scalar_t>(0), _params.h(_params.start, _params.end));

            // Main loop:
            while (!_open.empty()) {

                auto curr(std::move(_open.top()));
                _open.pop();

                if (curr.m_index != e) { // SEARCH FOR SOLUTION...

                    _closed.allocate(curr.m_index, _capacity, _params.maze.count());
                    _closed.emplace (curr.m_index);

                    for (const auto& n_data : _params.maze.get_neighbours(curr.m_index)) {

                        if (const auto& n = solver_t::get_data(n_data, _params); n.active) {

                            // Check if node is not already visited:
                            if (!_closed.contains(n.index)) {
                                 _closed.allocate(n.index, _capacity, _params.maze.count());
                                 _closed.emplace (n.index);

                                _open.emplace(n.index, curr.m_gScore + n.distance, _params.h(n.coord, _params.end) * _params.weight, new node(curr));
                            }
                        }
                    }
                }
                else { // SOLUTION REACHED ...

                    // Free data which is no longer relevant:

                    if constexpr (utils::has_method_clear        <  open_set_t>::value) {   _open.clear();         }
                    if constexpr (utils::has_method_shrink_to_fit<  open_set_t>::value) {   _open.shrink_to_fit(); }
                    if constexpr (utils::has_method_clear        <closed_set_t>::value) { _closed.clear();         }
                    if constexpr (utils::has_method_shrink_to_fit<closed_set_t>::value) { _closed.shrink_to_fit(); }

                    return curr.template backtrack<node>(_params.size, curr.m_gScore);
                }
            }

            return std::vector<coord_t>{};
        }

        [[maybe_unused, nodiscard]] static constexpr auto execute(const params_t& _params) {

            const auto s = utils::to_1d(_params.start, _params.size);

            const auto capacity = solver_t::determine_capacity(_params);

            existence_set closed({ s }, capacity);

            heap<node> open;
            open.reserve(capacity / 8U);

            stable_forward_buf<node*> buf;

            return solve_internal(open, closed, buf, capacity, _params);
        }
    };

} //chdr::solvers

#endif //CHDR_GSTAR_HPP