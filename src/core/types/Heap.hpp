#ifndef CHDR_HEAP_HPP
#define CHDR_HEAP_HPP

#include <vector>
#include <algorithm>

namespace CHDR {

    template<typename T, typename C>
    class Heap {

    private:

        std::vector<T> m_Data;
        C m_CompareFunction;

    public:

        [[nodiscard]] constexpr bool Empty() const { return m_Data.empty(); }

        [[nodiscard]] constexpr size_t Size() const { return m_Data.size(); }

        [[nodiscard]] constexpr const T& Top() const { return m_Data.front(); }

        constexpr void Push(const T& _value) {
            m_Data.push_back(_value);
            std::push_heap(m_Data.begin(), m_Data.end(), m_CompareFunction);
        }

        constexpr void Pop() {
            std::pop_heap(m_Data.begin(), m_Data.end(), m_CompareFunction);
            m_Data.pop_back();
        }

        constexpr void Rebuild() {
            std::make_heap(m_Data.begin(), m_Data.end(), m_CompareFunction);
        }
    };

}

#endif
