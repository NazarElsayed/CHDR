#ifndef CHDR_UTILS_HPP
#define CHDR_UTILS_HPP

#include <Debug.hpp>

#include <array>
#include <stdexcept>
#include <vector>

#include "Coord.hpp"

namespace CHDR {

	struct Utils {

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
		template<typename _Tp, size_t _Nm>
		static constexpr std::array<_Tp, _Nm> ToArray(const std::vector<_Tp>&& _vector) {

			if (_vector.size() != _Nm) {

				Debug::Log("Vector -> Array size mismatch! (" + std::to_string(_vector.size()) + ", " + std::to_string(_Nm) + ")");

				throw std::runtime_error("Vector -> Array size mismatch!");
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
		template<typename _Tp, size_t _Nm>
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

		template<typename T, typename... Args>
		static constexpr auto ToND(size_t index, Args... dimensions) {
			constexpr size_t N = sizeof...(Args);

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			Coord<T, N> indices;

			std::array<T, N> dims = {dimensions...};
			std::array<T, N> _strides{};

			_strides[0] = 1;
			for (size_t i = 1; i < N; ++i){
				_strides[i] = _strides[i - 1] * dims[i - 1];
			}

			for (int i = N-1; i >= 0; --i) {
				indices[i] = index / _strides[i];
				index %= _strides[i];
			}

			return indices;
		}

		template<typename T, typename... Args>
		static constexpr auto To1D(const Coord<T, sizeof...(Args)>& indices, Args... sizes) {
			constexpr size_t N = sizeof...(Args);

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			std::array<T, N> _sizes{ sizes... };

			T result = 0;
			for (int i = N-1; i >= 0; --i){
				result = (result * _sizes[i]) + indices[i];
			}

			return result;
		}

	};

} // CHDR

#endif //CHDR_UTILS_HPP