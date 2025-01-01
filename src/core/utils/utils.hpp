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
#include <cstddef>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "types/coord.hpp"
#include "types/existence_set.hpp"

namespace chdr {

	struct utils {

	private:

		// http://loungecpp.wikidot.com/tips-and-tricks%3aindices
		template <size_t... Is>
		struct indices {};

		template <size_t N, size_t... Is>
		struct build_indices : build_indices<N - 1, N - 1, Is...> {};

		template <size_t... Is>
		struct build_indices<0, Is...> : indices<Is...> {};

		template <typename T, typename U, size_t N, size_t... Is>
		[[nodiscard]] static constexpr auto array_cast_helper(const std::array<U, N>& _a, indices<Is...> /*unused*/) {
			return std::array<T, N>{ static_cast<T>(std::get<Is>(_a))... };
		}

		template <typename T, typename Ta, size_t Kd>
		[[nodiscard]] static constexpr T product_helper(const std::array<Ta, Kd>& _array, const size_t& _index = 0U) {
			return (_index < Kd) ? _array[_index] * product_helper<T, Ta, Kd>(_array, _index + 1U) : T{1};
		}

	public:

		template<typename T, typename = void>
		struct has_method_clear : std::false_type {};

		template<typename T>
		struct has_method_clear<T, std::void_t<decltype(std::declval<T>().clear())>> : std::true_type {};

		template<typename T, typename = void>
		struct has_method_shrink_to_fit : std::false_type {};

		template<typename T>
		struct has_method_shrink_to_fit<T, std::void_t<decltype(std::declval<T>().shrink_to_fit())>> : std::true_type {};

		template <typename T>
		[[nodiscard]] static constexpr bool is_existence_set() {
			return std::is_same_v<T, existence_set<>>;
		}

		template<typename T, typename U, size_t N>
		[[nodiscard]] static constexpr auto array_cast(const std::array<U, N> &_a) {
			return array_cast_helper<T>(_a, build_indices<N>());
		}

		/**
		 * @brief Converts an std::vector to an std::array of a specified size using move semantics.
		 *
		 * This function takes an std::vector and converts its elements to an std::array
		 * of a specified size. It throws an std::runtime_error if the size of the vector
		 * does not match the size of the array.
		 *
		 * @tparam Tp The type of elements in the vector and array.
		 * @tparam Nm The size of the array.
		 * @param[in,out] _vector The vector to be converted.
		 * @return The resulting array.
		 * @throws std::runtime_error if the size of the vector does not match the size of the array.
		 */
		template<typename Tp, size_t Nm>
		[[nodiscard]] static constexpr std::array<Tp, Nm> to_array(const std::vector<Tp>&& _vector) {

            std::array<Tp, Nm> result;

            if (LIKELY(_vector.size() == Nm)) {

                std::move(
                    _vector.begin(),
                    _vector.begin() + std::min(_vector.size(), result.size()),
                     result.begin()
                );
            }
            else {
                throw std::runtime_error("Vector -> Array size mismatch! (" + std::to_string(_vector.size()) + ", " + std::to_string(Nm) + ")");
            }

			return result;
		}

		/**
		 * @brief Converts an std::array to an std::vector using move semantics.
		 *
		 * This function takes an std::array of any type and size and moves its elements
		 * into an std::vector. The size of the resulting vector will be equal to the size
		 * of the original array. If the original array has fewer elements than the size
		 * of the vector, only the available elements are moved into the vector.
		 *
		 * @tparam Tp The type of elements in the array.
		 * @tparam Nm The size of the array.
		 * @param[in,out] _array The std::array whose elements are to be moved to the std::vector.
		 * @return std::vector<T> The resulting std::vector with moved elements.
		 *
		 * @note The function assumes that the array size is greater than 0 and less than
		 * or equal to the maximum value of size_t. If this condition is not met, a
		 * static_assert will be triggered.
		 */
		template<typename Tp, size_t Nm>
		[[nodiscard]] static constexpr std::vector<Tp> to_vector(const std::array<Tp, Nm>&& _array) {

			std::vector<Tp> result;
			result.reserve(Nm);

			std::move(_array.begin(), _array.end(), std::back_inserter(result));

			return result;
		}

		/**
		 * @brief Moves all elements from one vector to another.
		 *
		 * This function moves all elements from the source vector (_from) to the destination vector (_to).
		 * The elements are moved using std::make_move_iterator to take advantage of move semantics, which can
		 * be more efficient than copying elements individually.
		 * After the move, the source vector will be empty.
		 *
		 * @tparam Tp The type of elements in the vectors.
		 * @param[in] _from The source vector from which elements will be moved.
		 * @param[in] _to The destination vector to which elements will be moved.
		 */
		template<typename Tp>
		static constexpr void move_into(std::vector<Tp>&& _from, std::vector<Tp>& _to) {

			_to.reserve(_from.size());

			_to.insert(
				_to.end(),
				std::make_move_iterator(_from.begin()),
				std::make_move_iterator(_from.end())
			);

			_from.clear();
		}

		/**
		 * Copies the elements from one vector to another.
		 *
		 * This function copies all the elements from the source vector `_from` to the destination vector `_to`.
		 * The elements are added to the end of the destination vector in the same order as they appear in the source vector.
		 *
		 * @tparam Tp The type of elements in the vector.
		 * @param[in] _from The source vector containing elements to be copied.
		 * @param[in,out] _to The destination vector to which the elements will be copied.
		 *
		 * @note The function does not clear the destination vector before copying. If there are existing elements in the
		 * destination vector, the copied elements are added to the end of the vector without removing the existing elements.
		 *
		 * @see std::vector
		 */
		template<typename Tp>
		static constexpr void copy_into(const std::vector<Tp>& _from, std::vector<Tp>& _to) {

			_to.reserve(_from.size());

			_to.insert(
				  _to.end(),
				_from.begin(),
				_from.end()
			);
		}

		/**
		 * Calculate the product of the elements in the given array.
		 *
		 * @tparam T The type to return.
		 * @tparam Ta The type of the array.
		 * @param _array The array to calculate the product of its elements.
		 * @return The product of the elements in the array.
		 *
		 * @note The input is not modified.
		 */
		template <typename T, typename Ta, size_t Kd>
		[[nodiscard]] static constexpr T product(const std::array<Ta, Kd>& _array) noexcept {

			if constexpr (Kd == 0U) {
				return T{0};
			}
			else {

				if (__builtin_is_constant_evaluated()) {
					return product_helper<T, Ta, Kd>(_array);
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
		 * @tparam Args The dimensions to convert into.
		 * @param _index The one-dimensional index to be converted.
		 * @param _sizes The dimensions of the N-dimensional space.
		 * @return A Coord object representing the N-dimensional coordinate.
		 * @throws std::runtime_error If the type of _index is not an integral type.
		 *
		 * @note The function assumes that the number of dimensions (_dimensions) is greater than 0.
		 *
		 * Example usage:
		 * \code{cpp}
		 * static const auto as3d = to_nd(63, 4, 4, 4);
		 * \endcode
		 */
		template<typename T, typename... Args>
		[[nodiscard]] static constexpr auto to_nd(const T& _index, const Args&... _sizes) noexcept {
			return to_nd(_index, {_sizes...});
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
		template<typename T, size_t Kd>
		[[nodiscard]] static constexpr auto to_nd(const T& _index, const coord<T, Kd>& _sizes) noexcept {

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			coord<T, Kd> result{};

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
		template<typename T, typename... Args>
		[[nodiscard]] static constexpr auto to_1d(const coord<T, sizeof...(Args)>& _indices, const Args&... _sizes) noexcept {
			return to_1d({_indices}, {_sizes...});
		}

		/**
		 * @param _indices The indices of the element in each dimension.
		 * @param _sizes The sizes of the array in each dimension.
		 * @return The one-dimensional index corresponding to the given multi-dimensional indices.
		 *
		 * This method takes in multi-dimensional indices and array sizes and returns the corresponding one-dimensional index.
		 * Only integer types are allowed for the indices.
		 */
		template<typename T, size_t Kd>
		[[nodiscard]] static constexpr auto to_1d(const coord<T, Kd>& _indices, const coord<T, Kd>& _sizes) noexcept {

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			T result{};

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
		template <typename T, typename Ta>
		[[nodiscard]] static constexpr int sign(const Ta& _val) noexcept {

			constexpr auto zero_v = static_cast<Ta>(0);
			constexpr auto  one_v = static_cast<Ta>(1);

			if constexpr (std::is_signed_v<Ta>) {
				return static_cast<T>(zero_v < _val) - (_val < zero_v);
			}
			else {
				return _val == zero_v ? zero_v : one_v;
			}
		}

        [[nodiscard]] static std::string trim_trailing_zeros(std::string _str) {

            _str.erase(_str.find_last_not_of('0') + 1U, std::string::npos);

            if (_str.back() == '.') {
                _str.pop_back();
            }

            return _str;
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

}//chdr

#endif //CHDR_UTILS_HPP