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

        [[nodiscard]] bool Empty() const { return m_Data.empty(); }

        [[nodiscard]] size_t Size() const { return m_Data.size(); }

        [[nodiscard]] const T& Top() const { return m_Data.front(); }

        void Push(const T& _value) {
            m_Data.push_back(_value);
            std::push_heap(m_Data.begin(), m_Data.end(), m_CompareFunction);
        }

        void Pop() {
            std::pop_heap(m_Data.begin(), m_Data.end(), m_CompareFunction);
            m_Data.pop_back();
        }

        void Rebuild() {
            std::make_heap(m_Data.begin(), m_Data.end(), m_CompareFunction);
        }
    };

}

#endif
