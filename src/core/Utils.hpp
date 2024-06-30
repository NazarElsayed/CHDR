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

		/**
		 * @brief Checks if two arrays are equal element-wise.
		 *
		 * This function compares each element of the two arrays and returns true if all elements are equal.
		 *
		 * @tparam T The element type of the arrays.
		 * @tparam Kd The size of the arrays.
		 * @param _a The first array to compare.
		 * @param _b The second array to compare.
		 * @return True if the two arrays are equal, false otherwise.
		 */
		template <typename T, size_t Kd>
		static constexpr bool Equals(const std::array<T, Kd>& _a, const std::array<T, Kd>& _b) {

			bool result = true;

			for (size_t i = 0; i < Kd; ++i) {
				if (_a[i] != _b[i]) {
					result = false;

					break;
				}
			}

			return result;
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
		static constexpr T Product(const std::array<Ta, Kd>& _array) {

			static_assert(Kd > 0, "Kd must be greater than 0.");

			T result;

			if constexpr (Kd == 0) {
				result = 0;
			}
			else {

				result = _array[0];

				for (size_t i = 1; i < _array.size(); ++i) {
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

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			constexpr auto N = sizeof...(Args);

			Coord<T, N> result{};

			std::array<T, N> dims { _sizes... };
			std::array<T, N> strides{};

			strides[0] = 1;
			for (size_t i = 1; i < N; ++i) {
				strides[i] = strides[i - 1] * dims[i - 1];
			}

			// Please note this loop uses integer underflow to bypass a quirk of reverse for-loops.
			auto idx = _index;
			for (size_t i = N - 1; i != std::numeric_limits<size_t>::max(); --i) {
				result[i] = idx / strides[i];
				idx %= strides[i];
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
		 * @tparam Kd The dimensions to convert into.
		 * @param _index The one-dimensional index to be converted.
		 * @param _sizes The dimensions of the N-dimensional space.
		 * @return A Coord object representing the N-dimensional coordinate.
		 * @throws std::runtime_error If the type of _index is not an integral type.
		 *
		 * @note The function assumes that the number of dimensions (_dimensions) is greater than 0.
		 *
		 * Example usage:
		 * \code{cpp}
		 * static const auto as3d = ToND(63, { 4, 4, 4 });
		 * \endcode
		 */
		template<typename T, size_t Kd>
		static constexpr auto ToND(const T& _index, const Coord<T, Kd>& _sizes) {

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			Coord<T, Kd> result{};

			std::array<T, Kd> strides{};

			strides[0] = 1;
			for (size_t i = 1; i < Kd; ++i) {
				strides[i] = strides[i - 1] * _sizes[i - 1];
			}

			// Please note this loop uses integer underflow to bypass a quirk of reverse for-loops.
			auto idx = _index;
			for (size_t i = Kd - 1; i != std::numeric_limits<size_t>::max(); --i) {
				result[i] = idx / strides[i];
				idx %= strides[i];
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

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			constexpr size_t N = sizeof...(Args);

			T result{0};

			std::array sizes { _sizes... };

			// Please note this loop uses integer underflow to bypass a quirk of reverse for-loops.
			for (size_t i = N - 1; i != std::numeric_limits<size_t>::max(); --i) {
				result = (result * sizes[i]) + _indices[i];
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
		template<typename T, size_t Kd>
		static constexpr auto To1D(const Coord<T, Kd>& _indices, const Coord<T, Kd>& _sizes) {

			static_assert(std::is_integral_v<T>, "Only integer types are allowed.");

			T result{0};

			// Please note this loop uses integer underflow to bypass a quirk of reverse for-loops.
			for (size_t i = Kd - 1; i != std::numeric_limits<size_t>::max(); --i) {
				result = (result * _sizes[i]) + _indices[i];
			}

			return result;
		}

	};

} // CHDR

#endif //CHDR_UTILS_HPP