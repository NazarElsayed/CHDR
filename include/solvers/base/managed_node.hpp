/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_MANAGEDNODE_HPP
#define CHDR_MANAGEDNODE_HPP

/**
 * @file managed_node.hpp
 */

#include <cassert>
#include <cstdint>

#include "bnode.hpp"
#include "../../types/pmr/heterogeneous_pool.hpp"

// ReSharper disable once CppUnusedIncludeDirective
#include "../../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr::solvers {

    /**
     * @nosubgrouping
     * @struct managed_node
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

        using  node_t = std::conditional_t<std::is_void_v<derived>, managed_node<index_t>, derived>;
        using count_t = uint8_t;

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
         * @pre If `_parent` is not `nullptr`, it must point to a valid and initialised unmanaged_node.
         *      Failing to ensure this may lead to undefined behaviour.
         *
         * @see bnode
         * @see m_parent
         */
        [[nodiscard]] HOT constexpr managed_node(index_t _index, managed_node* RESTRICT const _parent = nullptr) noexcept : bnode<index_t>(_index),
            m_parent(_parent),
            m_successors(0U)
        {
            if (_parent != nullptr) { // Increment:

                assert(m_parent != nullptr && "Dereferencing of nullptr.");
                assert(m_parent->m_successors != static_cast<decltype(m_successors)>(-1U) && "Overflow detected!");

                ++m_parent->m_successors;
            }
        }

        constexpr managed_node           (const managed_node&) = default;
        constexpr managed_node& operator=(const managed_node&) = default;

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
         * @param [in, out] _resource A pointer to the polymorphic memory resource used for deallocating nodes.
         *                            It must manage the memory appropriately and be properly initialised.
         *
         * @warning Care should be taken to ensure that the hierarchy structure remains valid and
         *          consistency is maintained, as improper use can result in undefined behaviour due
         *          to invalid memory access or double deallocations.
         *
         * @see m_parent
         * @see managed_node
         */
        template <typename memory_resource_t>
        HOT void expunge(memory_resource_t* _resource) {

            while (m_parent != nullptr) {

                { // Decrement:
                    assert(m_parent != nullptr && "Dereferencing of nullptr.");
                    assert(m_parent->m_successors != static_cast<decltype(m_successors)>(0U) && "Underflow detected!");

                    --m_parent->m_successors;
                }

                if (m_parent->count() == 0U) {
                    auto* const RESTRICT d = m_parent;
                    m_parent = m_parent->m_parent;
                    _resource->deallocate(d, sizeof(node_t), alignof(node_t));
                }
                else {
                    break;
                }
            }
        }

        /**
         * @brief Decrements parent's successor count and checks if it can be released.
         *
         * @details This function decrements the successor count of the parent node if one exists.
         *          If the parent's successor count reaches zero after decrementing, the parent
         *          node is marked for potential release by returning a pointer to it.
         *          If there is no parent node, nullptr is returned.
         *
         * @return A pointer to the parent node if it can be released (has zero successors),
         *         nullptr otherwise.
         *
         * @warning The actual release/deallocation of the returned node must be handled by
         *          the caller. This function only indicates which node is eligible for release.
         *
         * @pre The parent node's successor count must be greater than 0 before decrementing.
         *      This is checked via assertion.
         *
         * @see m_parent
         * @see m_successors
         * @see count()
         */
        HOT auto* forget_one() {

            managed_node* result { nullptr };

            if (m_parent != nullptr) {

                assert(m_parent != nullptr && "Dereferencing of nullptr.");
                assert(m_parent->m_successors != static_cast<decltype(m_successors)>(0U) && "Underflow detected!");

                // Decrement:
                if (--m_parent->m_successors == 0U) {
                    result = m_parent;
                }
            }

            return result;
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
        [[nodiscard]] HOT constexpr count_t count() const noexcept { return m_successors; }

        /**
         * @brief Sets the current successor count.
         *
         * @details Manually changes the current successor count.
         *
         * @see expunge
         * @see managed_node
         *
         * @warning Performing manual reference management can break the node and lead to memory leaks.
         *          Only use this function if you know what you are doing!
         */
        HOT constexpr void count(count_t _count) noexcept { m_successors = _count; }
    };

} //chdr::solvers

#endif //CHDR_MANAGEDNODE_HPP