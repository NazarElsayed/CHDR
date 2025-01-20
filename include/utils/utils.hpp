/*
 * Computational Helper for Direction and Routing (CHDR)
 * Copyright (c) 2024 by Nazar Elsayed & Louis Eriksson
 *
 * Licensed under CC BY-NC-ND 4.0
 * https://creativecommons.org/licenses/by-nc-nd/4.0/
 */

#ifndef CHDR_UTILS_HPP
#define CHDR_UTILS_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <type_traits>

// ReSharper disable once CppUnusedIncludeDirective
#include "../utils/intrinsics.hpp" // NOLINT(*-include-cleaner)

namespace chdr {

	struct utils {

	private:

		template <typename T>
		[[nodiscard]] static constexpr T overflow_safe_multiply(const T& _a, const T& _b) noexcept {

			static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type.");

			constexpr auto max = std::numeric_limits<T>::max();

			return LIKELY(_b == 0 || _a <= max / _b) ? _a * _b : max; // Alternatively, throw
		}

		template <typename T, typename coord_t>
		[[nodiscard]] static constexpr T product_helper(const coord_t& _array, const size_t& _index = 0U) noexcept {
			
			static_assert(std::is_arithmetic_v<T> && std::is_trivially_constructible_v<T>, "T must be a trivially constructible arithmetic type.");
			
			constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

			return (_index < Kd) ? overflow_safe_multiply<T>(product_helper<T>(_array, _index + 1U), _array[_index]) : T{1};
		}

	public:

		 utils()                         = delete;
		 utils           (const utils& ) = delete;
		 utils           (const utils&&) = delete;
		 utils& operator=(const utils& ) = delete;
		 utils& operator=(const utils&&) = delete;
		~utils()                         = delete;

		template <typename T, typename coord_t>
		[[nodiscard]] static constexpr T product(const coord_t& _array) noexcept {
			return product_helper<T>(_array);
		}

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
				const auto w1 = _sizes[0U] * _sizes[1U],
				           w2 = _index % w1;

				result = {
					w2 % _sizes[1U],
					w2 / _sizes[1U],
					_index / w1
				};
			}
			else if constexpr (Kd == 4U) {
				const auto w1 = _sizes[2U] * _sizes[1U],
				           w2 = w1 * _sizes[0U],
				           w3 = _index % w2;

				result = {
					(w3 % w1) % _sizes[2U],
					(w3 % w1) / _sizes[2U],
					 w3 / w1,
					_index / w2,
				};
			}
			else if constexpr (Kd > 4U) {
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
        else if constexpr (Kd > 4U) {
        	result = 0;

        	IVDEP
        	for (size_t i = Kd; i != 0U; --i) {
        		result = (result * _sizes[i - 1UL]) + _indices[i - 1UL];
        	}
        }

			return result;
		}

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

		template <typename T>
		HOT static constexpr T abs(const T& _value) noexcept {

			static_assert(std::is_arithmetic_v<T>, "Type T must be arithmetic.");

			return std::is_unsigned_v<T> || _value >= 0 ? _value : -_value;
		}

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
							x    = (x + _value / x) / static_cast<T>(2);
						}

						return x;
					}

					return _value; // sqrt(0) = 0, sqrt(1) = 1
				}

				return std::numeric_limits<T>::quiet_NaN();
			}
			else {
				return static_cast<T>(std::sqrt(_value));
			}
		}

		template <typename T>
		HOT static constexpr const T& min(const T& _a, const T& _b) noexcept {
			static_assert(std::is_invocable_r_v<bool, decltype(std::less<>()), T, T>, "Type T must support the less-than operator.");
			return (_a < _b) ? _a : _b;
		}

		template <typename T>
		HOT static constexpr const T& max(const T& _a, const T& _b) noexcept {
			static_assert(std::is_invocable_r_v<bool, decltype(std::less<>()), T, T>, "Type T must support the less-than operator.");
			return (_a < _b) ? _b : _a;
		}

		template <typename T>
		HOT static constexpr const T& clamp(const T& _value, const T& _min, const T& _max) noexcept {
			static_assert(std::is_invocable_r_v<bool, decltype(std::less<>()), T, T>, "Type T must support the less-than operator.");

			if (_value >= _min) {
				return _value < _max ? _value : _max;
			}
			else {
				return _min;
			}
		}
		
        [[nodiscard]] static std::string to_string(const long double& _duration, const long double& _scale = std::numeric_limits<long double>::epsilon()) {

            static std::array<std::string, 4U> units = { "s", "ms", "µs", "ns" };

			auto result = _duration;

            size_t i = 0U;
            while (i < units.size() && result < static_cast<long double>(1.0)) {
                result *= static_cast<long double>(1000.0);
                ++i;
            }

            return trim_trailing_zeros(
	            std::to_string(
		            std::floor(result / _scale + static_cast<long double>(0.5)) * _scale
	            )
            ) + units[i];
        }

		[[nodiscard]] static std::string trim_trailing_zeros(std::string _str) {

			_str.erase(_str.find_last_not_of('0') + 1U, std::string::npos);

			if (_str.back() == '.') {
				_str.pop_back();
			}

			return _str;
		}
	};

} //chdr

#endif //CHDR_UTILS_HPP