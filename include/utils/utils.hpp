/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_UTILS_HPP
#define CHDR_UTILS_HPP

/**
 * @file utils.hpp
 */

#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <type_traits>

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr {

    /**
     * @struct utils
     * @brief A static utility class containing common utility functions across the library.
     *
     * @details Contains static helper functions for arithmetic operations, multidimensional
     *         indexing, and coordinate transformations.
     */
    struct utils {

    private:

        /**
         * @brief Multiplies two numbers and clamps in the case of overflow.
         *
         * @details Multiplies two arithmetic values and ensures that the result does not exceed the
         *          maximum value for the given type. If an overflow is detected, the maximum value
         *          for the given type is returned instead.
         *
         * @tparam T The arithmetic type of the two input values. The type must satisfy
         *           `std::is_arithmetic_v<T>`.
         *
         * @param[in] _a The first value to be multiplied.
         * @param[in] _b The second value to be multiplied.
         *
         * @return The product of `_a` and `_b`. If the multiplication would result
         *         in an overflow, the maximum value for the type `T` is returned.
         */
        template <typename T>
        [[nodiscard]] static constexpr T overflow_safe_multiply(const T& _a, const T& _b) noexcept {

            static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type.");

            constexpr auto max = std::numeric_limits<T>::max();

            return LIKELY((_b == 0) || (_a <= max / _b)) ? _a * _b : max; // Alternatively, throw.
        }

        /**
         * @brief Calculates the product of all elements in a coordinate or array.
         *
         * @details Recursively calculates the product of all elements in a multidimensional array-like structure,
         *          utilising overflow-safe multiplication. The calculation begins from the given index and continues
         *          through the array until reaching the end. If the index exceeds the size of the array, the base
         *          case returns `1`. The function ensures that overflow does not occur during multiplication.
         *
         * @tparam T The arithmetic type used for the computation. It must satisfy
         *           `std::is_arithmetic_v<T>` and `std::is_trivially_constructible_v<T>`.
         *
         * @tparam coord_t The type of the coordinate. It must implement `std::tuple_size`
         *                 and support subscript indexing.
         *
         * @param [in] _coord The coordinate whose elements' product is to be computed.
         *
         * @param _index (optional) The current index indicating where in the array computation starts.
         *                          Defaults to `0`.
         *
         * @return The product of all elements in the specified coordinate starting from `_index`.
         *         If `_index >= size of _coord`, `1` is returned as the base recursion case. If any
         *         multiplication results in an overflow, the value is clamped to the maximum representable
         *         value for the type `T`.
         */
        template <typename T, typename coord_t>
        [[nodiscard]] static constexpr T product_helper(const coord_t& _coord, size_t _index = 0U) noexcept {

            static_assert(std::is_arithmetic_v<T> && std::is_trivially_constructible_v<T>, "T must be a trivially constructible arithmetic type.");

            constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

            return (_index < Kd) ? overflow_safe_multiply<T>(product_helper<T>(_coord, _index + 1U), _coord[_index]) : T{1};
        }

    public:

        utils()                         = delete;
        utils           (const utils& ) = delete;
        utils           (const utils&&) = delete;
        utils& operator=(const utils& ) = delete;
        utils& operator=(const utils&&) = delete;
        ~utils()                        = delete;

        /**
         * @brief Calculates the product of all elements in a coordinate or array.
         *
         * @details Recursively calculates the product of all elements in a multidimensional array-like structure,
         *          utilising overflow-safe multiplication. The calculation begins from the given index and continues
         *          through the array until reaching the end. If the index exceeds the size of the array, the base
         *          case returns `1`. The function ensures that overflow does not occur during multiplication.
         *
         * @param[in] _coord The coordinate whose elements' product is to be computed.
         *
         * @tparam T The arithmetic type used for the computation. It must satisfy
         *           `std::is_arithmetic_v<T>` and `std::is_trivially_constructible_v<T>`.
         *
         * @tparam coord_t The type of the coordinate. It must implement `std::tuple_size`
         *                 and support subscript indexing.
         *
         * @return The product of all elements in the specified coordinate starting from `_index`.
         *         If `_index >= size of _coord`, `1` is returned as the base recursion case. If any
         *         multiplication results in an overflow, the value is clamped to the maximum representable
         *         value for the type `T`.
         */
        template <typename T, typename coord_t>
        [[nodiscard]] static constexpr T product(const coord_t& _coord) noexcept {
            return product_helper<T>(_coord);
        }

        /**
         * @brief Maps a one-dimensional index to a multidimensional coordinate representation.
         *
         * @details Converts the given one-dimensional array index `_index` into a set of indices
         *          that correspond to the various dimensions of a multidimensional space
        *           represented by `_sizes`.
        *           Supports 1–N dimensional spaces, with explicit optimisations
         *          performed for the first four dimensions
         *
         * @param [in] _index The flattened one-dimensional array index to be converted.
         *                    Must be an integral type, and represent a valid index within the
         *                    bounds defined by `_sizes`.
         *
         * @param [in] _sizes A collection of sizes of each dimension in the multidimensional
         *                    space. Each element specifies the extent of the corresponding
         *                    dimension. The number of dimensions, `Kd`, is determined by
         *                    the size of this parameter and must be greater than 0.
         *
         * @return The computed multidimensional coordinate as a container of type `coord_t`,
         *         where each element specifies the index along a particular dimension.
         */
        template<typename coord_t>
        [[nodiscard]] static constexpr auto to_nd(const typename std::decay_t<coord_t>::value_type& _index, const coord_t& _sizes) noexcept {

            using T = typename std::decay_t<decltype(_index)>;
            static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

            constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;
            static_assert(Kd > 0U, "The number of dimensions must be greater than 0.");

            coord_t result{};

            if constexpr (Kd == 1U) {
                result = { _index };
            }
            else if constexpr (Kd == 2U) {
                result = {
                    _index % _sizes[0U],
                    _index / _sizes[0U]
                };
            }
            else if constexpr (Kd == 3U) {
                const auto w1 = _sizes[0U] * _sizes[1U];
                const auto w2 = _index % w1;

                result = {
                    w2 % _sizes[1U],
                    w2 / _sizes[1U],
                    _index / w1
                };
            }
            else if constexpr (Kd == 4U) {
                const auto w1 = _sizes[2U] * _sizes[1U];
                const auto w2 = w1 * _sizes[0U];
                const auto w3 = _index % w2;

                result = {
                    (w3 % w1) % _sizes[2U],
                    (w3 % w1) / _sizes[2U],
                     w3 / w1,
                    _index / w2,
                };
            }
            else {
                std::array<T, Kd> strides{};
                strides[0U] = 1;

                IVDEP
                for (size_t i = 1U; i < Kd; ++i) {
                    strides[i] = strides[i - 1U] * _sizes[i - 1U];
                }

                // Please note this loop uses integer underflow to bypass a quirk of reverse for-loops.
                auto idx = _index;
                IVDEP
                for (size_t i = Kd; i != 0U; --i) {
                    result[i - 1U] = idx / strides[i - 1U];
                    idx %= strides[i - 1U];
                }
            }

            return result;
        }

        /**
         * @brief Maps a multidimensional index to a one-dimensional array index.
         *
         * @details Calculates the flattened one-dimensional index corresponding to
         *          the specified multidimensional indices `_indices`, given the
         *          dimensions and sizes of the multidimensional space `_sizes`.
         *          Supports 1–N dimensional spaces, with explicit optimisations
         *          performed for the first four dimensions.
         *
         * @param [in] _indices A coordinate representation where each element specifies
         *                      the index along a particular dimension. The size of
         *                      this parameter must match the number of dimensions.
         *
         * @param [in] _sizes The sizes of each dimension in the multidimensional space.
         *                    Each element defines the maximum size along the
         *                    corresponding dimension. The size of this parameter
         *                    must match the number of dimensions.
         *
         * @returns The computed one-dimensional array index as a value of type T,
         *          where T represents the integral type used by coordinate elements.
         */
        template<typename coord_t>
        [[nodiscard]] static constexpr auto to_1d(const coord_t& _indices, const coord_t& _sizes) noexcept {

            using T = typename std::decay_t<coord_t>::value_type;
            static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

            constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;
            static_assert(Kd > 0U, "The number of dimensions must be greater than 0.");

            T result{};

        if constexpr (Kd == 1U) {
            result = _indices[0U];
        }
        else if constexpr (Kd == 2U) {
            result = (_indices[1U] * _sizes[0U]) + _indices[0U];
        }
        else if constexpr (Kd == 3U) {
            result = (_indices[2U] * (_sizes[1U]  * _sizes[0U])) +
                     (_indices[1U] *  _sizes[0U]) +
                      _indices[0U];
        }
        else if constexpr (Kd == 4U) {
            result = (_indices[3U] * (_sizes[2U]  * _sizes[1U]   * _sizes[0U])) +
                     (_indices[2U] * (_sizes[1U]  * _sizes[0U])) +
                     (_indices[1U] *  _sizes[0U]) +
                      _indices[0U];
        }
        else {
            result = 0;

            IVDEP
            for (size_t i = Kd; i != 0U; --i) {
                result = (result * _sizes[i - 1UL]) + _indices[i - 1UL];
            }
        }

            return result;
        }

        /**
         * @brief Determines the sign of a given value.
         *
         * @details Computes the sign of the specified value by leveraging compile-time checks
         *          to handle signed and unsigned types differently. For signed types, it returns:
         *          - A positive indicator `1` for values greater than zero.
         *          - A negative indicator `-1` for values less than zero.
         *          - Zero `0` for values exactly equal to zero.
         *          For unsigned types, it simply returns zero for values equal to zero,
         *          and a positive indicator otherwise. This function enforces static type checks
         *          to ensure the input type supports required operations (e.g., less-than or equality).
         *
         * @param [in] _val The value whose sign is to be determined. It must be an arithmetic type
         *                  that supports the comparison operators needed for evaluation.
         *
         * @returns The calculated sign of the input value as `return_t`. This will differ based on
         *          whether the type `T` is signed or unsigned:
         *          - For signed types: a positive, negative, or zero value in the expected numerical range.
         *          - For unsigned types: zero or a positive equivalent of `one_v`.
         */
        template <typename return_t, typename T>
        [[nodiscard]] static constexpr return_t sign(const T& _val) noexcept {

            constexpr auto zero_v = static_cast<T>(0);
            constexpr auto  one_v = static_cast<T>(1);

            if constexpr (std::is_signed_v<T>) {
                static_assert(std::is_invocable_r_v<bool, decltype(std::less<>()), T, T>, "Type T must support the less-than operator.");
                static_assert(!std::is_same_v<return_t, bool> && "return_t cannot be bool if T is signed.");

                return static_cast<return_t>(zero_v < _val) - (_val < zero_v);
            }
            else {
                static_assert(std::is_invocable_r_v<bool, decltype(std::equal_to<>()), T, T>, "Type T must support the equality operator.");

                return static_cast<return_t>(_val == zero_v ? zero_v : one_v);
            }
        }

        /**
         * @brief Computes the power of an unsigned integral base raised to an unsigned integral exponent.
         *
         * @details This function calculates the result of raising the given base value to the power of the provided exponent.
         *          It utilises an iterative, optimised method (exponentiation by squaring), suitable for unsigned integral
         *          types that support arithmetic operations. The input must satisfy the constraints as enforced by the static assertion.
         *
         * @tparam T  The type of the base and exponent values. Must be an unsigned integral type supporting arithmetic operations.
         *            Any type that does not satisfy these conditions will result in a static assertion failure.
         *
         * @param _base The unsigned integral base to be raised to the provided exponent.
         * @param _exp  The unsigned integral exponent. If the exponent is 0, the result is always 1, regardless of the base.
         *
         * @return The computed value of _base raised to the power of _exp.
         *         For example, powui(2, 3) would return 8.
         *
         * @note The calculations assume that the result remains within the range of the provided unsigned integral type.
         *       Overflow behaviour is undefined and must be considered by the caller.
         */
        template <typename T>
        static constexpr T powui(T _base, T _exp) noexcept {

            static_assert(std::is_unsigned_v<T> && std::is_integral_v<T> && std::is_arithmetic_v<T>,
                          "powui requires an unsigned integral supporting arithmetic operations.");

            T result { 1U };

            while (_exp > 0U) {
                if (_exp % 2U == 1U) {
                    result *= _base;
                }
                _base *= _base;
                _exp  /= 2U;
            }

            return result;
        }

        /**
         * @brief Computes the absolute value of a given arithmetic type.
         *
         * @details This function returns the absolute value of the input parameter, ensuring
         *          correct handling of signed and unsigned arithmetic types. If the input type
         *          is unsigned, the value is returned as-is. For signed types, a negative value
         *          is converted to its positive equivalent.
         *
         * @param [in] _value The arithmetic value for which the absolute value is to be computed.
         *
         * @pre Type T must satisfy the arithmetic constraint.
         *
         * @return The absolute value of the input parameter. If the input is unsigned or non-negative,
         *         the original value is returned. For signed negative values, the positive equivalent is returned.
         */
        template <typename T>
        HOT static constexpr T abs(const T& _value) noexcept {

            static_assert(std::is_arithmetic_v<T>, "Type T must be arithmetic.");

            return std::is_unsigned_v<T> || _value >= static_cast<T>(0) ? _value : -_value;
        }

        /**
         * @brief Computes the square root of a given value.
         *
         * @details This method calculates the square root of a given value, leveraging a compile-time
         *          constant evaluation mechanism if supported. For constant evaluation, it uses
         *          an iterative approach (Newton's method). If the value is evaluated at runtime,
         *          it delegates the computation to the standard library's `std::sqrt`. The function
         *          handles special cases, such as values of `0` and `1`, directly. If the input value
         *          is negative during compile-time evaluation, the result will be `NaN` (Not a Number).
         *
         * @param [in] _value The input value whose square root is to be computed. It must be an arithmetic type.
         *                    Passing a negative value during constant evaluation will result in `NaN`.
         *
         * @returns The computed square root of the input value. If the input is `0` or `1`, the return value
         *          will be the input itself. If the input is negative and evaluated at compile time, the
         *          function will return `NaN`. For runtime evaluation, behaviour for negative inputs
         *          depends on the standard library's `std::sqrt` implementation.
         */
        template <typename T>
        HOT static constexpr T sqrt(const T& _value) noexcept {

            static_assert(std::is_arithmetic_v<T>, "Type T must be arithmetic.");

            if (__builtin_is_constant_evaluated()) {

                if (_value >= static_cast<T>(0)) {

                    if (_value != static_cast<T>(0) && _value != static_cast<T>(1)) {

                        T x    { _value };
                        T last {    0   };

                        while (x != last) {
                            last = x;
                            x = (x + _value / x) / static_cast<T>(2);
                        }

                        return x;
                    }

                    return _value; // sqrt(0) = 0, sqrt(1) = 1
                }

                return std::numeric_limits<T>::quiet_NaN();
            }

            return static_cast<T>(std::sqrt(_value));
        }

        /**
         * @brief Returns the smaller of two values.
         *
         * @details Compares two values of type T and returns the smaller of the two.
         *          The function imposes a static assertion to ensure that the type T
         *          supports the less-than operator for comparison.
         *
         * @param [in] _a The first value to be compared.
         * @param [in] _b The second value to be compared.
         *
         * @returns A reference to the smaller of the two values. If both values are equal,
         *          the reference to the first argument `_a` is returned.
         */
        template <typename T>
        HOT static constexpr const T& min(const T& _a, const T& _b) noexcept {
            static_assert(std::is_invocable_r_v<bool, decltype(std::less<>()), T, T>, "Type T must support the less-than operator.");
            return (_a < _b) ? _a : _b;
        }

        /**
         * @brief Determines the maximum of two comparable values.
         *
         * @details Compares two input values of the same type and returns the larger of the two.
         *          The comparison relies on the less-than (`<`) operator, and the type `T` must
         *          support this operation to be used with this function.
         *
         * @param [in] _a The first value to compare.
         * @param [in] _b The second value to compare.
         *
         * @returns A constant reference to the larger of the two input values.
         */
        template <typename T>
        HOT static constexpr const T& max(const T& _a, const T& _b) noexcept {
            static_assert(std::is_invocable_r_v<bool, decltype(std::less<>()), T, T>, "Type T must support the less-than operator.");
            return (_a < _b) ? _b : _a;
        }

        /**
         * @brief Constrains a value to lie within a specified range.
         *
         * @details This function ensures that the given value `_value` is clamped within the range defined
         *          by `_min` and `_max`. If `_value` is less than `_min`, `_min` is returned. If `_value`
         *          exceeds `_max`, `_max` is returned. Otherwise, `_value` is returned unaltered.
         *
         * @param [in] _value  The value to be clamped.
         * @param [in] _min The minimum allowable value of the range.
         * @param [in] _max The maximum allowable value of the range.
         *
         * @returns A reference to the constrained value, either `_value`, `_min`, or `_max`, depending on the range checks.
         */
        template <typename T>
        HOT static constexpr const T& clamp(const T& _value, const T& _min, const T& _max) noexcept {
            static_assert(std::is_invocable_r_v<bool, decltype(std::less<>()), T, T>, "Type T must support the less-than operator.");

            if (_value >= _min) {
                return _value < _max ? _value : _max;
            }

            return _min;
        }

        /**
         * @brief Converts a duration in seconds to a formatted string representation.
         *
         * @details Produces a formatted string representation, of the given duration in seconds,
         *          scaling the value to an appropriate unit (seconds, milliseconds, microseconds, or nanoseconds).
         *          The resulting string will include the unit suffix and round the result based on the provided scale.
         *
         * @param _duration The duration in seconds to be converted. It must be expressed as a floating-point number.
         *
         * @param _scale (optional) The precision scale to be used for rounding the scaled result.
         *                          By default, it uses the smallest precision value for long double representation.
         *                          The value is clamped to the range 0 -> 1.
         *
         * @returns A string containing the scaled and formatted duration, including its
         *          appropriate time unit (e.g. "s" for seconds, "ms" for milliseconds).
         */
        [[nodiscard]] static std::string to_string(long double _duration, long double _scale = std::numeric_limits<long double>::epsilon()) {

            constexpr std::array<const char* const, 4U> units { "s", "ms", "µs", "ns" };

            auto result = _duration;

            size_t i = 0U;
            while (i < units.size() && abs(result) < 1.0L) {
                result *= 1000.0L;
                ++i;
            }

            _scale = clamp(_scale, 0.0L, 1.0L);

            return trim_trailing_zeros(
                std::to_string(
                    std::floor((result / _scale) + 0.5L) * _scale
                )
            ) + units[i];
        }

        /**
         * @brief Removes trailing zeroes from a numeric string.
         *
         * @details Removes any trailing zeroes from a numeric string representation, as well as
         *          a trailing decimal point if all decimal digits are removed.
         *
         * @param _str The string to be processed. It must represent a numeric value.
         *             If any trailing zeros or an unnecessary decimal point are present,
         *             they will be removed from this string.
         *
         * @returns A string with trailing zeros removed. If there was a trailing decimal
         *          point with no digits following, it is also removed. The modified string
         *          maintains its numeric value.
         */
        [[nodiscard]] static std::string trim_trailing_zeros(std::string _str) {

            auto& trimmed = _str.erase(_str.find_last_not_of('0') + 1U, std::string::npos);

            if (trimmed.back() == '.') {
                trimmed.pop_back();
            }

            return trimmed;
        }
    };

} //chdr

#endif //CHDR_UTILS_HPP