/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_GRAPH_HPP
#define CHDR_GRAPH_HPP

#include "base/IGraph.hpp"
#include "Grid.hpp"

#include <unordered_map>
#include "base/IGraph.hpp"

#include <vector>

namespace CHDR::Mazes {

    template<typename Ti, size_t Kd, typename Ts = float>
    class Graph : public IGraph<Ti, Ts> {

    private:

        using edge_t = typename IGraph<Ti, Ts>::edge_t;

        std::unordered_map<Ti, std::vector<edge_t>> m_Entries;

    public:

        constexpr Graph(std::initializer_list<std::initializer_list<edge_t>> _adjacency_list) : m_Entries{} {

            size_t node_id = 0U;

            for (auto& entry : _adjacency_list) {

                for (const auto& edge_list: entry) {
                    Add(node_id, edge_list);
                }

                ++node_id;
            }
        }

        constexpr Graph(Grid<Kd, Ts> _grid) : m_Entries{} {

            for (auto& element : _grid) {
                // TODO: Convert grid to (sparse) graph.
            }
        }

        void Add(const Ti& _from_id, const edge_t& _edge) {
            m_Entries[_from_id].emplace_back(_edge);
        }

        void Remove(const Ti& _from_id) {

            if (Contains(_from_id)) {
                m_Entries.erase(_from_id);
            }
        }

        void Trim() {

            for (auto it = m_Entries.begin(); it != m_Entries.end();) {
                if (it->second.empty()) {
                    it = m_Entries.erase(it);
                }
                else {
                    it->second.shrink_to_fit();
                    ++it;
                }
            }
        }

        [[nodiscard]] constexpr const std::vector<edge_t>& GetNeighbours(const Ti& _id) const override {

            auto it = m_Entries.find(_id);
            return it->second;
        }

        [[nodiscard]] constexpr bool Contains(const Ti& _id) const override {
            return m_Entries.find(_id) != m_Entries.end();
        }

        [[nodiscard]] constexpr size_t Count() const override {
            return m_Entries.size();
        }

        using       iterator = typename std::unordered_map<Ti, std::vector<edge_t>>::iterator;
        using const_iterator = typename std::unordered_map<Ti, std::vector<edge_t>>::const_iterator;

              iterator  begin()       { return m_Entries.begin();  }
        const_iterator  begin() const { return m_Entries.begin();  }
        const_iterator cbegin() const { return m_Entries.cbegin(); }

              iterator  end()       { return m_Entries.end();  }
        const_iterator  end() const { return m_Entries.end();  }
        const_iterator cend() const { return m_Entries.cend(); }

    };

} // CHDR::Mazes

#endif //CHDR_GRAPH_HPP