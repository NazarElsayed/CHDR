#ifndef CHDR_COORD_HPP
#define CHDR_COORD_HPP

#include <cstddef>
#include <array>
#include <type_traits>

namespace CHDR {

    template<typename T, const size_t Kd>
    using Coord = std::array<T, Kd>;

    template<typename T>
    static constexpr T golden_ratio() {

        T result;

        static_assert(std::is_integral_v<T>, "T must be an integral type");

        if constexpr (std::is_same_v<T, uint8_t> || std::is_same_v<T, int8_t> || std::is_same_v<T, char>) {
            result = 0x9E;
        }
        else if constexpr (std::is_same_v<T, uint16_t> || std::is_same_v<T, int16_t>) {
            result = 0x9E37;
        }
        else if constexpr (std::is_same_v<T, uint32_t> || std::is_same_v<T, int32_t>) {
            result = 0x9E3779B9;
        }
        else if constexpr (std::is_same_v<T, uint64_t> || std::is_same_v<T, int64_t>) {
            result = 0x9E3779B97F4A7C15;
        }
#if defined(__GNUC__) || defined(__clang__)
        else if constexpr (std::is_same_v<T, __uint128_t> || std::is_same_v<T, __int128_t>) {
            result = ((__uint128_t)0x9E3779B97F4A7C15 << 64) + 0xE2DB6FB8F11B894A; // (0x9E3779B97F4A7C15E2DB6FB8F11B894A)
        }
#endif

        return result;
    }

} // CHDR

namespace std {

    template<typename T, const size_t Kd>
    struct hash<CHDR::Coord<T, Kd>> {

        size_t operator()(const CHDR::Coord<T, Kd>& _value) const {

            size_t result = _value.size();

            // Golden ratio hashing function.
            for (const auto& element : _value) {
                result ^= std::hash<T>{}(element) + CHDR::golden_ratio<T>() + (result << 6U) + (result >> 2U);
            }

            return result;
        }
    };

} // std

#endif //CHDR_COORD_HPP