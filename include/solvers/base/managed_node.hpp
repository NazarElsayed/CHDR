/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_MANAGEDNODE_HPP
#define CHDR_MANAGEDNODE_HPP

#include <cassert>

#include "bnode.hpp"
#include "../../types/pmr/polytonic_pool.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @nosubgrouping
     * @brief A hierarchical node in a pathfinding context, with a clean-up mechanism.
     *
     * @details This structure includes a pointer to its parent, allowing it to handle
     *          the hierarchical relationship between nodes.
     *          It uses a reference counting mechanism to aid with resource management.
     *
     * @warning Clean-up must be triggered manually, via the `expunge` function.
     *
     * @tparam index_t The data type used for indexing or identifying the node.
     * @tparam derived (optional) derived class type for CRTP (Curiously Recurring Template Pattern) usage.
     *
     * @see bnode
     * @see m_parent
     * @see expunge
     */
    template <typename index_t, typename derived = void>
    struct managed_node : bnode<index_t> {

    private:

        using node_t = std::conditional_t<std::is_void_v<derived>, managed_node<index_t>, derived>;
        using count_t = unsigned char;

    public:

        /**
         * @brief A pointer to the parent node in the hierarchical structure.
         *
         * This member allows traversal of the hierarchy by referencing the immediate
         * parent of the current node. Use of `restrict` indicates that this pointer
         * is not aliased, improving optimisation in certain compilers.
         *
         * @warning The lifetime of the parent node is not managed by this member. Users must
         *          ensure that the parent node remains valid for the duration of any access
         *          via this pointer.
         *
         * @remark If `m_parent` is `nullptr`, it indicates that the node is a root node
         *         and does not have a parent.
         */
        managed_node* RESTRICT m_parent;

    private:

        /**
         * @brief Tracks the number of successor nodes.
         *
         * This member variable is used to maintain a count of successor nodes directly connected
         * to the current node in a graph or hierarchical structure. A successor is typically
         * defined as a child node in the context of parent-child relationships.
         *
         * @remark This count is automatically incremented or decremented using the `incr` and `decr`
         *         member functions when successors are added or removed.
         *
         * @note The unsigned nature ensures that the count cannot be negative, adhering to
         *       logical constraints for successor tracking. Overflow and underflow conditions
         *       are checked with assertions during increment and decrement operations.
         *
         * @warning Proper management is required to keep the state consistent, especially when
         *          manually modifying related structures. Ensure that adjustments are correctly
         *          made within operations on the hierarchical relationships, such as node deletion
         *          via `expunge`.
         *
         * @see decr
         * @see expunge
         * @see incr
         * @see managed_node
         */
        count_t m_successors;

        /**
         * @brief Decrements the successor count of the parent node.
         *
         * This member function decreases the `m_successors` count for the parent node
         * (`m_parent`) by exactly 1.
         *
         * @remark This operation is commonly used when removing or expunging child
         *         nodes from a parent node in a hierarchical structure.
         *
         * @throws This function does not throw exceptions; however, violations of
         *         preconditions (e.g., a null parent or an underflow situation) trigger
         *         an assertion failure in debug builds.
         *
         * @pre `m_parent != nullptr` – The parent node must exist.
         * @pre `m_parent->m_successors > 0` – The parent node must have at least one successor.
         *
         * @post The parent's successor count (`m_successors`) is decremented by 1.
         *
         * @see expunge
         * @see incr
         * @see m_successors
         * @see managed_node
         */
        constexpr void decr() const noexcept {
            assert(m_parent != nullptr && "Dereferencing of nullptr.");
            assert(m_parent->m_successors != static_cast<decltype(m_successors)>(0U) && "Underflow detected!");

            --m_parent->m_successors;
        }

        /**
         * @brief Increments the successor count of the parent node.
         *
         * This member function increases the `m_successors` count for the parent node
         * (`m_parent`) by exactly 1.
         *
         * @remark This operation is commonly used during expansion or when
         *         adding child nodes from a parent node in a hierarchical structure.
         *
         * @throws This function does not throw exceptions; however, violations of
         *         preconditions (e.g., a null parent or an underflow situation) trigger
         *         an assertion failure in debug builds.
         *
         * @pre `m_parent != nullptr` – The parent node must exist.
         * @pre `m_parent->m_successors != -1U ` – The operation must not overflow.
         *
         * @see decr
         * @see expunge
         * @see m_successors
         * @see managed_node
         */
        constexpr void incr() const noexcept {
            assert(m_parent != nullptr && "Dereferencing of nullptr.");
            assert(m_parent->m_successors != static_cast<decltype(m_successors)>(-1U) && "Overflow detected!");

            ++m_parent->m_successors;
        }

    public:

        /**
         * @name Constructors
         * @{
         */

        /**
         * @brief Constructs an uninitialized node.
         *
         * This constructor creates a node with uninitialized members.
         */
        [[nodiscard]] constexpr managed_node() noexcept : // NOLINT(*-pro-type-member-init, *-use-equals-default)
            m_parent(nullptr),
            m_successors(0U) {}

        ~managed_node() noexcept = default;

        /**
         * @brief Constructs a node with a specified index and an optional parent pointer.
         *
         * This constructor initialises a node, allowing both the index and the parent node to be
         * specified. The parent pointer is useful in defining hierarchical relationships between
         * nodes in tree-like structures. By default, the parent node is set to `nullptr`, which
         * means the node is a root in the hierarchy.
         *
         * @param _index The index or identifier of the node, used in the context of pathfinding or search.
         * @param _parent [in] (optional) A pointer to the parent node in the hierarchy. If `nullptr`,
         *                 the node will be considered a root node.
         *
         * @warning The memory associated with the parent node is not managed by this constructor or
         *         the unmanaged_node class itself. It is the caller’s responsibility to ensure that
         *         the parent node remains valid for the lifetime of this object.
         *
         * @warning If `_parent` is not `nullptr`, it must point to a valid and initialised
         *          unmanaged_node. Failing to ensure this may lead to undefined behaviour.
         *
         * @see bnode
         * @see m_parent
         *
         * @return An instance of managed_node, initialised with the given parameters.
         */
        [[nodiscard]] HOT constexpr managed_node(index_t _index, managed_node* RESTRICT const _parent = nullptr) noexcept : bnode<index_t>(_index),
            m_parent(_parent),
            m_successors(0U)
        {
            if (_parent != nullptr) {
                incr();
            }
        }

        constexpr managed_node           (const managed_node&) = delete;
        constexpr managed_node& operator=(const managed_node&) = delete;

        [[nodiscard]] HOT constexpr managed_node(managed_node&&) noexcept = default;

#if __cplusplus > 202302L
        constexpr
#endif
        managed_node& operator=(managed_node&&) noexcept = default;

        /**
         * @}
         */

        /**
         * @brief Recursively releases dead nodes from the parent hierarchy.
         *
         * @details This function traverses up the parent hierarchy, erasing nodes
         *          that have no remaining successors. The traversal halts when either
         *          the root of the hierarchy is reached, or a node with a non-zero
         *          successor count is found.
         *
         * @param[in, out] _pmr A pointer to the polymorphic memory resource used for deallocating
         *                      nodes. It must manage the memory appropriately and be properly initialised.
         *
         * @warning Care should be taken to ensure that the hierarchy structure remains valid and
         *          consistency is maintained, as improper use can result in undefined behaviour due
         *          to invalid memory access or double deallocations.
         *
         * @see m_parent
         * @see managed_node
         */
        template <typename pmr_t>
        HOT void expunge(pmr_t* _pmr) noexcept {

            while (m_parent != nullptr) {
                decr();

                if (m_parent->m_successors != 0U) {
                    break;
                }

                auto* const RESTRICT d = m_parent;
                try {
                    m_parent = m_parent->m_parent;
                    _pmr->deallocate(d, sizeof(managed_node), alignof(managed_node));
                }
                catch (...) {
                    m_parent = d;
                }
            }
        }

        /**
         * @brief Returns the current successor count.
         *
         * @details Returns the current successor count, which is updated when the node is expanded or collapsed.
         *
         * @see expunge
         * @see managed_node
         *
         * @returns The successor count.
         */
        count_t count() const noexcept {
            return m_successors;
        }
    };

} //chdr::solvers

#endif //CHDR_MANAGEDNODE_HPP