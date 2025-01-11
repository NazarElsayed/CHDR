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
#include <type_traits>
#include <vector>

#include "types/existence_set.hpp"
#include "utils/intrinsics.hpp"

namespace chdr {

	struct utils {

	private:

		template <typename T, typename coord_t>
		[[nodiscard]] static constexpr T product_helper(const coord_t& _array, const size_t& _index = 0U) {

			constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;

			return (_index < Kd) ? _array[_index] * product_helper<T>(_array, _index + 1U) : T{1};
		}

	public:

		template <typename T, typename coord_t>
		[[nodiscard]] static constexpr T product(const coord_t& _array) noexcept {

			constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;
			if constexpr (Kd == 0U) {
				return T{0};
			}
			else {

				if (__builtin_is_constant_evaluated()) {
					return product_helper<T>(_array);
				}
				else {

					T result = _array[0U];
					for (size_t i = 1U; i < _array.size(); ++i) {
						result *= _array[i];
					}

					// TODO: Ensure that product does not overflow!
					return result;
				}
			}
		}

		/**
		 * @brief Converts a one-dimensional index to an N-dimensional coordinate.
		 *
		 * This function takes a one-dimensional index and a list of dimensions, and calculates the corresponding
		 * N-dimensional coordinate. The dimensions represent the size of each dimension in the N-dimensional space.
		 *
		 * @tparam T The type of the index. Only integer types are allowed.
		 * @tparam Kd The dimensions to convert into.
		 * @param _index The one-dimensional index to be converted.
		 * @param _sizes The dimensions of the N-dimensional space.
		 * @return A Coord object representing the N-dimensional coordinate.
		 * @throws std::runtime_error If the type of _index is not an integral type.
		 *
		 * @note The function assumes that the number of dimensions (_dimensions) is greater than 0.
		 */
		template<typename coord_t>
		[[nodiscard]] static constexpr auto to_nd(const typename std::decay_t<coord_t>::value_type& _index, const coord_t& _sizes) noexcept {

			using T = typename std::decay_t<decltype(_index)>;

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			coord_t result{};

			constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;
            if constexpr (Kd > 4U) {

                std::array<T, Kd> strides{};
                strides[0U] = 1;

                for (size_t i = 1U; i < Kd; ++i) {
                    strides[i] = strides[i - 1U] * _sizes[i - 1U];
                }

                // Please note this loop uses integer underflow to bypass a quirk of reverse for-loops.
                auto idx = _index;
                for (size_t i = Kd - 1U; i != std::numeric_limits<size_t>::max(); --i) {
                    result[i] = idx / strides[i];
                    idx %= strides[i];
                }
            }
            else if constexpr (Kd > 3U) {

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
            else if constexpr (Kd > 2U) {

                const auto w1 = _sizes[0U] * _sizes[1U],
                           w2 = _index % w1;

                result = {
                        w2 % _sizes[1U],
                        w2 / _sizes[1U],
                    _index / w1
                };
            }
            else if constexpr (Kd > 1U) {
                result = {
                    _index % _sizes[0U],
                    _index / _sizes[0U]
                };
            }
            else if constexpr (Kd > 0U) {
                result = { _index };
            }
            else {
                static_assert([]{ return false; }(), "No specialisation exists for n-dimensional values to 0D");
            }

			return result;
		}

		/**
		 * @param _indices The indices of the element in each dimension.
		 * @param _sizes The sizes of the array in each dimension.
		 * @return The one-dimensional index corresponding to the given multi-dimensional indices.
		 *
		 * This method takes in multi-dimensional indices and array sizes and returns the corresponding one-dimensional index.
		 * Only integer types are allowed for the indices.
		 */
		template<typename coord_t>
		[[nodiscard]] static constexpr auto to_1d(const coord_t& _indices, const coord_t& _sizes) noexcept {

			using T = typename std::decay_t<coord_t>::value_type;

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			T result{};

			constexpr auto Kd = std::tuple_size_v<std::decay_t<coord_t>>;
            if constexpr (Kd > 4U) {

                result = 0;

                // Please note this loop uses integer underflow to bypass a quirk of reverse for-loops.
                for (size_t i = Kd - 1UL; i != -1UL; --i) {
                    result = (result * _sizes[i]) + _indices[i];
                }
            }
            else if constexpr (Kd > 3U) {
                result = (_indices[3U] * (_sizes[2U] * _sizes[1U] * _sizes[0U])) +
                         (_indices[2U] * (_sizes[1U] * _sizes[0U])) +
                         (_indices[1U] *  _sizes[0U]) +
                          _indices[0U];
            }
            else if constexpr (Kd > 2U) {
                result = (_indices[2U] * (_sizes[1U] * _sizes[0U])) +
                         (_indices[1U] *  _sizes[0U]) +
                          _indices[0U];
            }
            else if constexpr (Kd > 1U) {
                result = (_indices[1U] * _sizes[0U]) + _indices[0U];
            }
            else if constexpr (Kd > 0U) {
                result = _indices[0U];
            }
            else {
                static_assert([]{ return false; }(), "No specialisation exists for converting 0-dimensional values to 1D");
            }

			return result;
		}

		/**
		 * Determines the sign of a given value.
		 *
		 * @param _val The value for which the sign needs to be determined.
		 *     The type of value must be suitable for comparison with zero.
		 * @return 1 if the value is positive, -1 if the value is negative, and 0 if the value is zero.
		 */
		template <typename return_t, typename T>
		[[nodiscard]] static constexpr return_t sign(const T& _val) noexcept {

			constexpr auto zero_v = static_cast<T>(0);
			constexpr auto  one_v = static_cast<T>(1);

			if constexpr (std::is_unsigned_v<T>) {
				return static_cast<return_t>(_val == zero_v ? zero_v : one_v);
			}
			else {
				static_assert(!std::is_same_v<return_t, bool> && "return_t cannot be bool if T is signed.");
				
				return static_cast<return_t>(zero_v < _val) - (_val < zero_v);
			}
		}

		template <typename T>
		static constexpr T powui(T _base, T _exp) noexcept {

			static_assert(std::is_integral_v<T>, "powui only supports integral types.");
			static_assert(std::is_unsigned_v<T>, "powui requires an unsigned integral.");

			T result { 1U };

			while (_exp > 0U) {
				if (_exp % 2U == 1U) {
					result *= _base;
				}
				_base *= _base;
				_exp /= 2U;
			}
			
			return result;
		}

		template <typename T>
		static constexpr T abs(const T& _value) noexcept {

			static_assert(std::is_arithmetic_v<T>, "abs only supports arithmetic types.");

			return std::is_unsigned_v<T> || _value >= 0 ? _value : -_value;
		}

		/**
		* @brief Computes the square root of a number using the Newton-Raphson method.
		*
		* This function works for both integral and floating-point types.
		*
		* @tparam T The type of the input number (integral or floating-point).
		* @param _value The number to compute the square root for.
		* @return The square root of the input value.
		*/
		template <typename T>
		static constexpr T sqrt(const T& _value) noexcept {

			static_assert(std::is_arithmetic_v<T>, "Input must be an arithmetic type.");
			static_assert(!std::is_same_v<T, bool>, "Input type cannot be boolean.");

			if (!__builtin_is_constant_evaluated()) {
				return std::sqrt(_value);
			}
			else {

				if (_value >= T(0)) {

					if (_value != T(0) && _value != T(1)) {

						T x    {_value};
						T last {0};

						while (x != last) {
							last = x;
							x    = (x + _value / x) / T(2);
						}

						return x;
					}
					return _value; // sqrt(0) = 0, sqrt(1) = 1
				}
				return std::numeric_limits<T>::quiet_NaN();
			}
		}

		template <typename T, typename collection_t>
		static constexpr void preallocate_emplace(collection_t& _collection, const T& _value, const size_t& _increment, const size_t& _max_increment = std::numeric_limits<size_t>::max()) noexcept {

			if constexpr (std::is_same_v<collection_t, existence_set<>>) {
				_collection.allocate(_value, _increment, _max_increment);
			}

			_collection.emplace(_value);
		}

		[[nodiscard]] static std::string trim_trailing_zeros(std::string _str) {

            _str.erase(_str.find_last_not_of('0') + 1U, std::string::npos);

            if (_str.back() == '.') {
                _str.pop_back();
            }

            return _str;
        }

		template<typename node_t, typename coord_t>
		static constexpr auto rbacktrack(const node_t& _node, const coord_t& _size) {

			// Calculate size of the path.
			size_t count = 0U;
			for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent), ++count) {}

			// Construct result in reverse order.
			std::vector<coord_t> result(count);
			size_t i = 0U;
			for (const auto* RESTRICT t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent), ++i) {
				result[count - 1U - i] = utils::to_nd(t->m_index, _size);
			}

			return result;
		}

		template<typename node_t, typename coord_t>
		static constexpr auto rbacktrack(const node_t& _node, const coord_t& _size, const size_t& _depth) {

			// Construct result in reverse order.
			std::vector<coord_t> result(_depth);
			size_t i = 0U;
			for (const auto* t = &_node; t->m_parent != nullptr; t = static_cast<const node_t*>(t->m_parent), ++i) {
				result[_depth - 1U - i] = utils::to_nd(t->m_index, _size);
			}

			return result;
		}

		template <typename open_set_t, typename coord_t>
		[[nodiscard]] static constexpr auto ibacktrack(const open_set_t& _open, const coord_t& _size) {

			// Reserve space in result:
			std::vector<coord_t> result;
			result.reserve(_open.size());

			// Recurse from end node to start node, inserting into a result buffer:
			for (auto it = _open.rbegin(); it != _open.rend(); ++it) {
				result.emplace_back(utils::to_nd(it->m_index, _size));
			}

			return result;
		}

        [[nodiscard]] static std::string to_string(const long double& _duration) {

            static std::array<std::string, 4U> units = { "s", "ms", "µs", "ns" };

			auto result = _duration;

            size_t i = 0U;
            while (i < units.size() && result < static_cast<long double>(1.0)) {
                result *= static_cast<long double>(1000.0);
                ++i;
            }

            return trim_trailing_zeros(std::to_string(result)) + units[i];
        }

	};

} //chdr

#endif //CHDR_UTILS_HPP