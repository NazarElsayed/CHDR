#ifndef CHDR_STACKALLOCATOR_HPP
#define CHDR_STACKALLOCATOR_HPP

#include <cstddef>
#include <memory>

template <typename T, size_t StackSize>
class StackAllocator {

private:

    alignas(T) char stack[StackSize * sizeof(T)];

    size_t stackPointer;

public:

    using value_type = T;

    StackAllocator() : stackPointer(0) {}

    T *allocate(const size_t& _n) {

        T* result;

        if (stackPointer + _n <= StackSize) {
            result = reinterpret_cast<T*>(stack + stackPointer * sizeof(T));
            stackPointer += _n;
        }
        else {
            result = std::allocator<T>().allocate(_n);
        }

        return result;
    }

    void deallocate(T* _p, const size_t& _n) {

        if (_p >= reinterpret_cast<T*>(stack) && _p < reinterpret_cast<T*>(stack + sizeof(stack))) {
            stackPointer -= _n;
        }
        else {
            std::allocator<T>().deallocate(_p, _n);
        }
    }

    template<typename U>
    struct rebind {
        using other = StackAllocator<U, StackSize>;
    };
};

#endif //CHDR_STACKALLOCATOR_HPP