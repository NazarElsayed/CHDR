/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_RAW_BLOCK_HPP
#define CHDR_RAW_BLOCK_HPP

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <exception>

#include "../../../utils/intrinsics.hpp"

template <typename T>
struct raw_block final {

    T* RESTRICT m_data;
    size_t m_size;

    raw_block(const size_t& _size) :
        m_data(static_cast<T*>(std::malloc(_size * sizeof(T)))),
        m_size(_size)
    {
        if (m_data == nullptr) { throw std::bad_alloc(); }
    }

    ~raw_block() noexcept {
        if (m_data != nullptr) {
            std::free(m_data);
            m_data = nullptr;
        }
    }

    raw_block           (const raw_block&) = delete;
    raw_block& operator=(const raw_block&) = delete;

    raw_block(raw_block&& _other) noexcept :
        m_data(_other.m_data),
        m_size(_other.m_size)
    {
        _other.m_data = nullptr;
        _other.m_size = 0U;
    }

    raw_block& operator=(raw_block&& _other) noexcept {

        if (this != &_other) {
            reset();

            m_data = _other.m_data;
            m_size = _other.m_size;
            _other.m_data = nullptr;
            _other.m_size = 0U;
        }
        return *this;
    }

    auto* get() const noexcept { return m_data; }

    void reset() noexcept {

        if (m_data != nullptr) {
            std::free(m_data);
            m_data = nullptr;
            m_size = 0U;
        }
    }

    T& operator[](const size_t& _index) noexcept {
        assert(_index < m_size && "Index out of bounds");
        return m_data[_index];
    }

    const T& operator[](const size_t& _index) const noexcept {
        assert(_index < m_size && "Index out of bounds");
        return m_data[_index];
    }
};

#endif //CHDR_RAW_BLOCK_HPP