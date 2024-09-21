/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_WAY_HPP
#define CHDR_WAY_HPP

namespace CHDR {

    template <typename W = bool>
    class Way {

    private:

        RelationalNode<W> m_Start;
        RelationalNode<W> m_End;

    public:

        constexpr Way(const RelationalNode<W>& _start, const RelationalNode<W>& _end) :
            m_Start(_start),
            m_End(_end) {}

        /**
         * @brief Sets the start node of the Way.
         * @param _start The start node.
         */
        constexpr void Start(const RelationalNode<W>& _start) {
            m_Start = _start;
        }

        /**
         * @brief Returns the start node of the Way.
         * @return The start node of the Way.
         */
        [[nodiscard]] constexpr const RelationalNode<W>& Start() {
            return m_Start;
        }

        /**
         * @brief Returns the start node of the Way.
         * @return The start node of the Way.
         */
        [[nodiscard]] const RelationalNode<W>& Start() const {
            return m_Start;
        }

        /**
         * @brief Sets the end node of the Way.
         * @param _end The end node.
         */
        constexpr void End(const RelationalNode<W>& _end) {
            m_End = _end;
        }

        /**
         * @brief Returns the end node of the Way.
         * @return The end node of the Way.
         */
        [[nodiscard]] constexpr const RelationalNode<W>& End() {
            return m_End;
        }

        /**
         * @brief Returns the end node of the Way.
         * @return The end node of the Way.
         */
        [[nodiscard]] const RelationalNode<W>& End() const {
            return m_End;
        }
    };

} // CHDR

#endif //CHDR_WAY_HPP