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
#include <limits>
#include <type_traits>
#include <vector>

#include "types/Coord.hpp"

#include "SIMDExtensions.hpp"

namespace CHDR {

	struct Utils {

	private:

		// http://loungecpp.wikidot.com/tips-and-tricks%3aindices
		template <std::size_t... Is>
		struct indices {};

		template <std::size_t N, std::size_t... Is>
		struct build_indices: build_indices<N-1, N-1, Is...> {};

		template <std::size_t... Is>
		struct build_indices<0, Is...>: indices<Is...> {};

		template<typename T, typename U, size_t N, size_t... Is>
		static constexpr auto ArrayCast_Helper(const std::array<U, N> &a, indices<Is...>) {
			return std::array<T, N> { static_cast<T>(std::get<Is>(a))... };
		}

	public:

		template<typename T, typename U, size_t N>
		static constexpr auto ArrayCast(const std::array<U, N> &_a) {
			return ArrayCast_Helper<T>(_a, build_indices<N>());
		}

		/**
		 * @brief Converts an std::vector to an std::array of a specified size using move semantics.
		 *
		 * This function takes an std::vector and converts its elements to an std::array
		 * of a specified size. It throws an std::runtime_error if the size of the vector
		 * does not match the size of the array.
		 *
		 * @tparam _Tp The type of elements in the vector and array.
		 * @tparam _Nm The size of the array.
		 * @param[in,out] _vector The vector to be converted.
		 * @return The resulting array.
		 * @throws std::runtime_error if the size of the vector does not match the size of the array.
		 */
		template<typename _Tp, const size_t _Nm>
		static constexpr std::array<_Tp, _Nm> ToArray(const std::vector<_Tp>&& _vector) {

			if (_vector.size() != _Nm) {
				throw std::runtime_error("Vector -> Array size mismatch! (" + std::to_string(_vector.size()) + ", " + std::to_string(_Nm) + ")");
			}

			std::array<_Tp, _Nm> result;
			std::move(
				_vector.begin(),
				_vector.begin() + std::min(_vector.size(), result.size()),
				 result.begin()
			);

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
		 * @tparam _Tp The type of elements in the array.
		 * @tparam _Nm The size of the array.
		 * @param[in,out] _array The std::array whose elements are to be moved to the std::vector.
		 * @return std::vector<T> The resulting std::vector with moved elements.
		 *
		 * @note The function assumes that the array size is greater than 0 and less than
		 * or equal to the maximum value of size_t. If this condition is not met, a
		 * static_assert will be triggered.
		 */
		template<typename _Tp, const size_t _Nm>
		static constexpr std::vector<_Tp> ToVector(const std::array<_Tp, _Nm>&& _array) {

			std::vector<_Tp> result;
			result.reserve(_Nm);

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
		 * @tparam _Tp The type of elements in the vectors.
		 * @param[in] _from The source vector from which elements will be moved.
		 * @param[in] _to The destination vector to which elements will be moved.
		 */
		template<typename _Tp>
		static constexpr void MoveInto(std::vector<_Tp>&& _from, std::vector<_Tp>& _to) {

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
		 * @tparam _Tp The type of elements in the vector.
		 * @param[in] _from The source vector containing elements to be copied.
		 * @param[in,out] _to The destination vector to which the elements will be copied.
		 *
		 * @note The function does not clear the destination vector before copying. If there are existing elements in the
		 * destination vector, the copied elements are added to the end of the vector without removing the existing elements.
		 *
		 * @see std::vector
		 */
		template<typename _Tp>
		static constexpr void CopyInto(const std::vector<_Tp>& _from, std::vector<_Tp>& _to) {

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
		template <typename T, typename Ta, const size_t Kd>
		static constexpr T Product(const std::array<Ta, Kd>& _array) {

			// TODO: Ensure that product does not overflow!

			T result;

			if constexpr (Kd == 0U) {
				result = 0;
			}
			else {

				result = _array[0U];
				for (size_t i = 1U; i < _array.size(); ++i) {
					result *= _array[i];
				}
			}

			return result;
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
		 * static const auto as3d = ToND(63, 4, 4, 4);
		 * \endcode
		 */
		template<typename T, typename... Args>
		static constexpr auto ToND(const T& _index, const Args&... _sizes) {
			return ToND(_index, {_sizes...});
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
		 * \endcode
		 */
		template<typename T, const size_t Kd>
		static constexpr auto ToND(const T& _index, const Coord<T, Kd>& _sizes) {

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			Coord<T, Kd> result{};

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
                static_assert([]{ return false; }(), "No specialisation exists for n-dimensional value to 0D");
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
		static constexpr auto To1D(const Coord<T, sizeof...(Args)>& _indices, const Args&... _sizes) {
			return To1D({_indices}, {_sizes...});
		}

		/**
		 * @param _indices The indices of the element in each dimension.
		 * @param _sizes The sizes of the array in each dimension.
		 * @return The one-dimensional index corresponding to the given multi-dimensional indices.
		 *
		 * This method takes in multi-dimensional indices and array sizes and returns the corresponding one-dimensional index.
		 * Only integer types are allowed for the indices.
		 */
		template<typename T, const size_t Kd>
		static constexpr auto To1D(const Coord<T, Kd>& _indices, const Coord<T, Kd>& _sizes) {

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			T result{};

            if constexpr (Kd > 4U) {

                result = 0;

                // Please note this loop uses integer underflow to bypass a quirk of reverse for-loops.
                for (size_t i = Kd - 1U; i != std::numeric_limits<size_t>::max(); --i) {
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
                static_assert([]{ return false; }(), "No specialisation exists for converting 0-dimensional value to 1D");
            }

			return result;
		}

        static std::string Trim_Trailing_Zeros(std::string _str) {

            _str.erase(_str.find_last_not_of('0') + 1U, std::string::npos);

            if (_str.back() == '.') {
                _str.pop_back();
            }

            return _str;
        }

        static std::string ToString(long double _duration) {

            static std::array<std::string, 4U> units = { "s", "ms", "µs", "ns" };

            size_t i = 0U;
            while (i < units.size() && _duration < static_cast<long double>(1.0)) {
                _duration *= static_cast<long double>(1000.0);
                ++i;
            }

            return Trim_Trailing_Zeros(std::to_string(_duration)) + units[i];
        }

	};

} // CHDR

#endif //CHDR_UTILS_HPP