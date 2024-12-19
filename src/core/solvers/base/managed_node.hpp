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
#include <cstddef>

#include "../../utils/intrinsics.hpp"
#include "bnode.hpp"

namespace chdr::solvers {

    template <typename T>
    struct tagged_ptr {

    private:

//#define TAGGED

#ifdef TAGGED

        uintptr_t m_data_union;

#else //!TAGGED

        T* RESTRICT m_ptr;
        uint16_t m_tag;

#endif //!TAGGED

    public:

#ifdef TAGGED

        constexpr tagged_ptr() noexcept : m_data_union(0U) {}

        constexpr tagged_ptr(T* RESTRICT _parent) noexcept : m_data_union(reinterpret_cast<uintptr_t>(_parent)) {}

        [[nodiscard]] constexpr T* ptr() const noexcept {
            return reinterpret_cast<T*>(m_data_union & ((1ULL << 48U) - 1U));
        }

        constexpr void ptr(T* RESTRICT _parent) noexcept {
            m_data_union = (m_data_union & ~((1ULL << 48U) - 1U)) | reinterpret_cast<uintptr_t>(_parent);
        }

        [[nodiscard]] constexpr uint16_t tag() const noexcept {
            return static_cast<uint16_t>(m_data_union >> 48U);
        }

        constexpr void tag(const uint16_t& _value) noexcept {
            m_data_union = (m_data_union & ((1ULL << 48U) - 1U)) | (static_cast<uintptr_t>(_value) << 48U);
        }

#else //!TAGGED

        constexpr tagged_ptr() noexcept :
            m_ptr(nullptr),
            m_tag(0U) {}

        constexpr tagged_ptr(T* RESTRICT _parent) noexcept :
            m_ptr(_parent),
            m_tag(0U) {}

        [[nodiscard]] constexpr T* ptr() const  noexcept { return m_ptr;    }
        constexpr void ptr(T* RESTRICT _parent) noexcept { m_ptr = _parent; }

        [[nodiscard]] constexpr uint16_t tag() const noexcept { return m_tag;   }
        constexpr void tag(const uint16_t& _value)   noexcept { m_tag = _value; }

#endif //!TAGGED

    };

    template<typename index_t>
    struct managed_node : bnode<index_t> {

        tagged_ptr<managed_node> m_counter;

        // ReSharper disable once CppPossiblyUninitializedMember
        [[nodiscard]] constexpr managed_node() noexcept : m_counter() {}  // NOLINT(*-pro-type-member-init, *-use-equals-default)

        [[nodiscard]] constexpr managed_node(const index_t& _index, managed_node* RESTRICT const _parent = nullptr) noexcept : bnode<index_t>(_index),
            m_counter(_parent)
        {
            incr();
        }

        [[nodiscard]] constexpr managed_node(const managed_node& _other) noexcept : bnode<index_t>(_other.m_index),
            m_counter(_other.m_counter)
        {
            m_counter.tag(0U);
            incr();
        }

        managed_node& operator=(const managed_node& _other) noexcept {

            if (this != &_other) {
                expunge();

                bnode<index_t>::operator=(_other.m_index);

                m_counter = _other.m_counter;
            }

            return *this;
        }

        [[nodiscard]] constexpr managed_node(managed_node&& _other) noexcept : bnode<index_t>(std::move(_other.m_index)),
            m_counter(_other.m_counter)
        {}

        constexpr managed_node& operator=(managed_node&& _other) noexcept {

            if (this != &_other) {
                expunge();

                bnode<index_t>::operator=(std::move(_other.m_index));

                m_counter = std::move(_other.m_counter);
            }

            return *this;
        }

        virtual ~managed_node() noexcept {
            expunge();
        }

        void expunge() noexcept {

            while (m_counter.ptr() != nullptr) {

                decr();

                if (m_counter.tag() == 0U) {

                    auto temp = m_counter;
                    m_counter = temp.ptr()->m_counter;

                    temp.ptr(nullptr);
                    delete temp.ptr();
                }
                else {
                    break;
                }
            }
        }

        constexpr void decr() noexcept {
            if (m_counter.ptr() != nullptr) {
                assert(m_counter.tag() != static_cast<decltype(m_counter.tag())>(0U) && "Underflow detected!");
                m_counter.tag(m_counter.tag() - 1U);
            }
        }

        constexpr void incr() noexcept {
            if (m_counter.ptr() != nullptr) {
                assert(m_counter.tag() != static_cast<decltype(m_counter.tag())>(-1U) && "Overflow detected!");
                m_counter.tag(m_counter.tag() + 1U);
            }
        }

        template <typename node_t, typename coord_t>
        [[nodiscard]] auto backtrack(const coord_t& _size, const size_t& _capacity = 0U) const {

            static_assert(std::is_base_of_v<managed_node, node_t>, "node_t must derive from managed_node");

            // Reserve space in result:
            std::vector<coord_t> result;
            result.reserve(_capacity);

            // Recurse from end node to start node, inserting into a result buffer:
            for (const auto* t = this; t->m_counter.ptr() != nullptr; t = static_cast<const node_t*>(t->m_counter.ptr())) {
                result.emplace_back(utils::to_nd(t->m_index, _size));
            }

            // Reverse the result:
            std::reverse(result.begin(), result.end());

            return result;
        }
    };

} //chdr::solvers

#endif //CHDR_MANAGEDNODE_HPP