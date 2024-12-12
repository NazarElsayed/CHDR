/*
 * Copyright (c) 2024 Loui Eriksson
 * All Rights Reserved.
 */

#ifndef LOUIERIKSSON_DEBUG_HPP
#define LOUIERIKSSON_DEBUG_HPP

#include <cstdlib>
#include <ctime>
#include <memory>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"

#pragma clang diagnostic push
#endif //ifdef __clang__

#ifdef __JETBRAINS_IDE__
#pragma ide diagnostic ignored "performance-avoid-endl"
#endif

#include <csignal>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#if __linux__ || __APPLE__
	
	#include <execinfo.h>

#elif _WIN32

	#include <windows.h>
	#include <intrin.h>
	#include <io.h>
	#include <fcntl.h>

#endif

/* Debugging assertions and traps
 * Portable Snippets - https://github.com/nemequ/portable-snippets
 * Created by Evan Nemerson <evan@nemerson.com>
 *
 *   To the extent possible under law, the authors have waived all
 *   copyright and related or neighbouring rights to this code.  For
 *   details, see the Creative Commons Zero 1.0 Universal license at
 *   https://creativecommons.org/publicdomain/zero/1.0/
 */
#pragma region Debugging assertions and traps

#if !defined(PSNIP_NDEBUG) && defined(NDEBUG) && !defined(PSNIP_DEBUG)
#  define PSNIP_NDEBUG 1
#endif

#if defined(__has_builtin) && !defined(__ibmxl__)
#  if __has_builtin(__builtin_debugtrap)
#    define psnip_trap() __builtin_debugtrap()
#  elif __has_builtin(__debugbreak)
#    define psnip_trap() __debugbreak()
#  endif
#endif
#if !defined(psnip_trap)
#  if defined(_MSC_VER) || defined(__INTEL_COMPILER)
#    define psnip_trap() __debugbreak()
#  elif defined(__ARMCC_VERSION)
#    define psnip_trap() __breakpoint(42)
#  elif defined(__ibmxl__) || defined(__xlC__)
#    include <builtins.h>
#    define psnip_trap() __trap(42)
#  elif defined(__DMC__) && defined(_M_IX86)
#    define psnip_trap(void) __asm int 3h;
#  elif defined(__i386__) || defined(__x86_64__)
#    define psnip_trap(void) __asm__ __volatile__("int3");
#  elif defined(__thumb__)
#    define psnip_trap(void) __asm__ __volatile__(".inst 0xde01");
#  elif defined(__aarch64__)
#    define psnip_trap(void) __asm__ __volatile__(".inst 0xd4200000");
#  elif defined(__arm__)
#    define psnip_trap(void) __asm__ __volatile__(".inst 0xe7f001f0");
#  elif defined (__alpha__) && !defined(__osf__)
#    define psnip_trap(void) __asm__ __volatile__("bpt");
#  elif defined(_54_)
#    define psnip_trap(void) __asm__ __volatile__("ESTOP");
#  elif defined(_55_)
#    define psnip_trap(void) __asm__ __volatile__(";\n .if (.MNEMONIC)\n ESTOP_1\n .else\n ESTOP_1()\n .endif\n NOP");
#  elif defined(_64P_)
#    define psnip_trap(void) __asm__ __volatile__("SWBP 0");
#  elif defined(_6x_)
#    define psnip_trap(void) __asm__ __volatile__("NOP\n .word 0x10000000");
#  elif defined(__STDC_HOSTED__) && (__STDC_HOSTED__ == 0) && defined(__GNUC__)
#    define psnip_trap() __builtin_trap()
#  else
#    include <signal.h>
#    if defined(SIGTRAP)
#      define psnip_trap() raise(SIGTRAP)
#    else
#      define psnip_trap() raise(SIGABRT)
#    endif
#  endif
#endif

#if defined(HEDLEY_LIKELY)
#  define PSNIP_DBG_LIKELY(expr) HEDLEY_LIKELY(expr)
#elif defined(__GNUC__) && (__GNUC__ >= 3)
#  define PSNIP_DBG_LIKELY(expr) __builtin_expect(!!(expr), 1)
#else
#  define PSNIP_DBG_LIKELY(expr) (!!(expr))
#endif

#if !defined(PSNIP_NDEBUG) || (PSNIP_NDEBUG == 0)
#  define psnip_dbg_assert(expr) do { \
    if (!PSNIP_DBG_LIKELY(expr)) { \
      psnip_trap(); \
    } \
  } while (0)
#else
#  define psnip_dbg_assert(expr)
#endif

#pragma endregion Debugging assertions and traps

namespace {
	
	/**
	 * @enum log_type
	 * @brief Enumeration representing different types of log messages.
	 *
	 * The LogType enum represents different levels of log messages that can be used to categorize log entries.
	 * Each log type is assigned a character value which is used to differentiate between log messages.
	 */
	 enum log_type : unsigned char {
		   trace = 1U,       /**< @brief In-depth tracking of system operations.     */
		   debug = 1U << 1U, /**< @brief General code debugging.                     */
		    info = 1U << 2U, /**< @brief General insights about application status.  */
		 warning = 1U << 3U, /**< @brief Potential issues that could cause problems. */
		   error = 1U << 4U, /**< @brief Major issues disrupting normal operations.  */
		critical = 1U << 5U, /**< @brief Severe problems causing system failure.     */
	};
	
	class print final {
		
		friend struct debug;
		
	private:
		
		static constexpr const char* to_string(const log_type& _type) noexcept {

			const auto* result = "UNKNOWN";
			
			switch (_type) {
				case critical: { result = "CRITICAL"; break; }
				case error:    { result = "ERROR";    break; }
				case warning:  { result = "WARNING";  break; }
				case info:     { result = "INFO";     break; }
				case debug:    { result = "DEBUG";    break; }
				case trace:    { result = "TRACE";    break; }
			}
			
			return result;
		}
		
		static void multiplatform(const std::string_view& _message, const log_type& _type, const bool& _makeInline) {
	
#ifdef __linux__
			try {
	            ansi(_message, _type, _makeInline);
			}
			catch (const std::exception&) {
				fallback(_message, _type, _makeInline);
				throw;
			}
#elif _WIN32
			try {
	            win32(_message, _type, _makeInline);
			}
			catch (const std::exception&) {
				fallback(_message, _type, _makeInline);
				throw;
			}
#elif __APPLE__
			try {
	            ansi(_message, _type, _makeInline);
			}
			catch (const std::exception&) {
				fallback(_message, _type, _makeInline);
				throw;
			}
#else
			fallback(_message, _type, _makeInline);
#endif
		
		}
		
		static void fallback(const std::string_view& _message, const log_type& _type, const bool& _makeInline) {
			
			std::cout << to_string(_type) << ": " << _message;
			
			if (_makeInline && _type != info) {
				std::cout << std::flush;
			}
			else {
				std::cout << std::endl;
			}
		}
		
#if __linux__ | __APPLE__
		
		static void ansi(const std::string_view& _message, const log_type& _type, const bool& _makeInline) {
			
			/* ANSI TEXT COLORS */
			#define ANSI_RESET   "\033[0m"
			#define ANSI_BLACK   "\033[30m"
			#define ANSI_RED     "\033[31m"
			#define ANSI_GREEN   "\033[32m"
			#define ANSI_YELLOW  "\033[33m"
			#define ANSI_BLUE    "\033[34m"
			#define ANSI_MAGENTA "\033[35m"
			#define ANSI_CYAN    "\033[36m"
			#define ANSI_WHITE   "\033[37m"
			
			/* ANSI BACKGROUND COLORS */
			#define ANSI_BG_BLACK   "\033[40m"
			#define ANSI_BG_RED     "\033[41m"
			#define ANSI_BG_GREEN   "\033[42m"
			#define ANSI_BG_YELLOW  "\033[43m"
			#define ANSI_BG_BLUE    "\033[44m"
			#define ANSI_BG_MAGENTA "\033[45m"
			#define ANSI_BG_CYAN    "\033[46m"
			#define ANSI_BG_WHITE   "\033[47m"
			
			switch (_type) {
				
				case critical: {
					std::cout << ANSI_MAGENTA << _message << ANSI_RESET << '\a';
					
					if (_makeInline) {
						std::cout << std::flush;
					}
					else {
						std::cout << std::endl;
					}
					
					break;
				}
				case error: {
					std::cout << ANSI_RED << _message << ANSI_RESET;
					
					if (_makeInline) {
						std::cout << std::flush;
					}
					else {
						std::cout << std::endl;
					}
					
					break;
				}
				case warning: {
					std::cout << ANSI_YELLOW << _message << ANSI_RESET;
					
					if (_makeInline) {
						std::cout << std::flush;
					}
					else {
						std::cout << std::endl;
					}
					
					break;
				}
				case info: {
					std::cout << ANSI_CYAN << _message << ANSI_RESET;
					
					if (!_makeInline) {
						std::cout << '\n';
					}
					
					break;
				}
				case debug: {
					std::cout << ANSI_WHITE << _message << ANSI_RESET;
					
					if (_makeInline) {
						std::cout << std::flush;
					}
					else {
						std::cout << std::endl;
					}
					
					break;
				}
				case trace: {
					std::cout << ANSI_BG_WHITE << ANSI_BLACK << _message << ANSI_RESET;
					
					if (_makeInline) {
						std::cout << '\n';
					}
					else {
						std::cout << std::endl;
					}
					
					break;
				}
				default: {
					std::cout << ANSI_BG_MAGENTA << ANSI_BLACK << _message << ANSI_RESET;
					
					if (_makeInline) {
						std::cout << std::flush;
					}
					else {
						std::cout << std::endl;
					}
					
					break;
				}
			}
			
			#undef ANSI_RESET
			#undef ANSI_BLACK
			#undef ANSI_RED
			#undef ANSI_GREEN
			#undef ANSI_YELLOW
			#undef ANSI_BLUE
			#undef ANSI_MAGENTA
			#undef ANSI_CYAN
			#undef ANSI_WHITE
			
			#undef ANSI_BG_BLACK
			#undef ANSI_BG_RED
			#undef ANSI_BG_GREEN
			#undef ANSI_BG_YELLOW
			#undef ANSI_BG_BLUE
			#undef ANSI_BG_MAGENTA
			#undef ANSI_BG_CYAN
			#undef ANSI_BG_WHITE
		}

#elif _WIN32

		void SetCAttr(void* _h, const WORD &_attribute) {
			
		    if (!SetConsoleTextAttribute(_h, _attribute)) {
		        throw std::runtime_error("Failed to set the console text attribute.");
		    }
		}

		static void win32(const std::string_view& _message, const log_type& _type, const bool& _makeInline) {

#define FOREGROUND_BLACK   0x0
#define FOREGROUND_CYAN	   0x3
#define FOREGROUND_RED     0x4
#define FOREGROUND_MAGENTA 0x5
#define FOREGROUND_YELLOW  0x6
#define FOREGROUND_WHITE   0x7
#define BACKGROUND_BLACK   0x00
#define BACKGROUND_MAGENTA 0x50
#define BACKGROUND_WHITE   0x70

			SetConsoleOutputCP(CP_UTF8);

			try {
				HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
				
				if (h != nullptr && h != INVALID_HANDLE_VALUE) {
					
					CONSOLE_SCREEN_BUFFER_INFO cinfo;
					if (GetConsoleScreenBufferInfo(h, &cinfo)) {
						
						const auto& previous_attr = cinfo.wAttributes;
						
						switch (_type) {
							case critical: {
	                            SetCAttr(h, BACKGROUND_BLACK | FOREGROUND_MAGENTA);
								std::cout << _message;
	                            SetCAttr(h, previous_attr);
								
								Beep(800, 200);
								
								if (_makeInline) {
									std::cout << std::flush;
								}
								else {
									std::cout << std::endl;
								}
								
								break;
							}
							case error: {
	                            SetCAttr(h, BACKGROUND_BLACK | FOREGROUND_RED);
								std::cout << _message;
	                            SetCAttr(h, previous_attr);
								
								if (_makeInline) {
									std::cout << std::flush;
								}
								else {
									std::cout << std::endl;
								}
								
								break;
							}
							case warning: {
	                            SetCAttr(h, BACKGROUND_BLACK | FOREGROUND_YELLOW);
								std::cout << _message;
	                            SetCAttr(h, previous_attr);
								
								if (_makeInline) {
									std::cout << std::flush;
								}
								else {
									std::cout << std::endl;
								}
								
								break;
							}
							case info: {
	                            SetCAttr(h, BACKGROUND_BLACK | FOREGROUND_CYAN);
								std::cout << _message;
	                            SetCAttr(h, previous_attr);
								
								if (!_makeInline) {
									std::cout << '\n';
								}
								
								break;
							}
							case debug: {
	                            SetCAttr(h, BACKGROUND_BLACK | FOREGROUND_WHITE);
								std::cout << _message;
	                            SetCAttr(h, previous_attr);
								
								if (_makeInline) {
									std::cout << std::flush;
								}
								else {
									std::cout << std::endl;
								}
								
								break;
							}
							case trace: {
	                            SetCAttr(h, BACKGROUND_WHITE | FOREGROUND_BLACK);
								std::cout << _message;
	                            SetCAttr(h, previous_attr);
								
								if (_makeInline) {
									std::cout << '\n';
								}
								else {
									std::cout << std::endl;
								}
								
								break;
							}
							default: {
	                            SetCAttr(h, BACKGROUND_MAGENTA | FOREGROUND_BLACK);
								std::cout << _message;
	                            SetCAttr(h, previous_attr);
								
								if (_makeInline) {
									std::cout << std::flush;
								}
								else {
									std::cout << std::endl;
								}
							}
						}
						
					}
					else {
						throw std::runtime_error("Failed to get the console screen buffer info.");
					}
				}
				else {
					throw std::runtime_error("Failed to get the standard output handle.");
				}
			}
			catch (const std::exception& e) {
				std::cerr << "WIN32_LOG_ERR: " << e.what() << std::endl;
				
				throw e;
			}

#undef FOREGROUND_BLACK
#undef FOREGROUND_CYAN
#undef FOREGROUND_RED
#undef FOREGROUND_MAGENTA
#undef FOREGROUND_YELLOW
#undef FOREGROUND_WHITE
#undef BACKGROUND_BLACK
#undef BACKGROUND_MAGENTA
#undef BACKGROUND_WHITE

		}

#endif
	
	};
	
	/**
	 * @brief The debug class provides a set of static methods for debugging and logging.
	 *
	 * The debug class is a utility class that provides various methods for debugging and logging purposes.
	 * It includes methods for performing assertions, triggering breakpoints, flushing the log output,
	 * and logging messages with different log types.
	 */
	struct debug final {
	
	private:
		
		inline static std::mutex s_lock;
		
		/**
		 * @brief Metadata about a log.
		 */
		struct meta final {
			
			std::time_t m_timestamp;
			     size_t m_thread_id;
			     bool   m_inline;
		};
		
		/** Metadata about the previous log. */
		inline static meta s_last_log = { 0U, -1U, false };
		
	public:
		
		struct thread_id final {
		
		private:
			
			inline static std::mutex s_lock;
			
			inline static size_t s_ctr;
			inline static std::unordered_map<std::thread::id, size_t> s_thread_ids;
		
		public:
		
            [[maybe_unused, nodiscard]] static size_t get(const std::thread::id _id) {
				
				const std::lock_guard guard(s_lock);
				
				size_t result;
				
				if (s_thread_ids.find(_id) != s_thread_ids.end()) {
					result = s_thread_ids[_id];
				}
				else {
					s_thread_ids[_id] = s_ctr;
					result = s_ctr++;
				}
				
				return result;
			}
			
            [[maybe_unused, nodiscard]] static size_t get() {
				return get(std::this_thread::get_id());
			}
			
		};
		
		/**
		 * @brief Asserts a condition and logs a message if the condition is false.
		 * By default, the log type is set to `debug`.
		 *
		 * @param[in] _condition The condition to assert.
		 * @param[in] _message The message to log if the condition is false.
		 * @param[in] _type (optional) The type of log message to log.
		 * @param[in] _makeInline (optional) A flag indicating if the log message should be displayed inline.
		 */
        [[maybe_unused]] static void asrt(const bool& _condition, const std::string_view& _message, const log_type& _type = log_type::debug, const bool& _makeInline = false) noexcept {
			
			if (!_condition) {
				log(_message, _type, _makeInline);
			}
		}
		
		/**
		 * @brief Triggers a breakpoint.
		 *
		 * @note This function only has an effect on debug builds.
		 *
		 * @warning Please note that if the function cannot identify the correct signal for a breakpoint, no breakpoint will occur.
		 */
        [[maybe_unused]] static void brk() noexcept {
		
#if !defined(NDEBUG) || _DEBUG
			
			flush();
			
			try {
				const std::lock_guard guard(s_lock);
				
				psnip_trap();
			}
			catch (const std::exception& e) {
				std::cerr << "BREAKPOINT_ERR: " << e.what() << "\n";
			}
			
#endif
		}
		
		/**
		 * @brief Flushes the log output.
		 *
		 * This static method is used to flush the log output. It flushes the output stream `std::cout` to ensure that any buffered log messages are immediately written to the output device.
		 *
		 * @note This method is declared `noexcept`, indicating that it does not throw any exceptions.
		 *
		 * @par Related Function
		 * - debug::Flush()
		 */
        [[maybe_unused]] static void flush() noexcept {
			
			const std::lock_guard guard(s_lock);
			
			try {
				try {
					std::cout << std::flush;
				}
				catch (const std::exception& e) {
					std::cerr << e.what() << std::endl;
				}
			}
			catch (...) {}
		}
		
		/**
		 * @brief Logs an exception with a specified log type.
		 *
		 * This static method is used to log an exception with a specified log type.
		 * By default, the log type is set to `LogType::error`.
		 *
		 * @param[in] _e The exception to log.
		 * @param[in] _type (optional) The log type of the message.
		 * @param[in] _makeInline (optional) A flag indicating if the log message should be displayed inline.
		 *
		 * @note This method is declared `noexcept`, indicating that it does not throw any exceptions.
		 *
		 * @par Related Functions
		 * - debug::log(const std::string_view&, const LogType&, const bool&)
		 */
        [[maybe_unused]] static void log(const std::exception& _e, const log_type& _type = error, const bool& _makeInline = false) noexcept {
			log(_e.what(), _type, _makeInline);
		}
		
		/**
		 * @brief Logs a message with a specified log type.
		 *
		 * This function logs a message with a specified log type.
		 * By default, the log type is set to `debug`.
		 *
		 * @param[in] _message The message to be logged.
		 * @param[in] _type (optional) The log type for the message (default is `LogType::debug`).
		 * @param[in] _makeInline (optional) Specifies whether the log message should be displayed inline (default is `false`).
		 */
        [[maybe_unused]] static void log(const std::string_view& _message, const log_type& _type = log_type::debug, const bool& _makeInline = false) noexcept {
			
			const std::lock_guard guard(s_lock);
			
			static constexpr size_t max_frames = 10;
			
			try {
				try {
					
					/*
					 * Construct a header containing various pieces of metadata about the current log.
					 */
					std::ostringstream message;
					
					const meta meta {
						std::time(nullptr),
						thread_id::get(),
						_makeInline
					};
					
					// Timestamp:
					if (!s_last_log.m_inline) {
						message << std::put_time(std::localtime(&meta.m_timestamp), "[%H:%M:%S %d/%m/%Y] ");
					}
					
					// Thread id:
					if (!s_last_log.m_inline || s_last_log.m_thread_id != meta.m_thread_id) {
						message << "[" + std::to_string(meta.m_thread_id) + "] ";
					}
					
					message << _message.data();
					
					// Print log to console:
					print::multiplatform(message.str(), _type, _makeInline);
					
					// Add trace information:
					if (_type == trace || _type == critical) {
						
						// Start trace on new line always.
						if (s_last_log.m_inline) {
							std::cout << "\n";
						}
						
						const auto trace = stack_trace(max_frames);
						for (size_t i = 0U; i < trace.size(); ++i) {
							
							// Indent each trace:
							for (size_t j = 0; j < i; ++j) {
								std::cout << '\t';
							}
							
							print::multiplatform(trace[i], log_type::trace, false);
						}
						
						// Flush the console.
						std::cout << std::flush;
					}

#if !defined(NDEBUG) || _DEBUG
					if (_type == log_type::critical) { brk(); }
#endif
					s_last_log = meta;
					
				}
				catch (const std::exception& e) {
					std::cerr << "LOG_ERR: " << e.what() << std::endl;
				}
			}
			catch (...) {}
		}
		
        [[maybe_unused, nodiscard]] static std::vector<std::string> stack_trace(const size_t& _frames) {

			std::vector<std::string> result;
			
			try {
			
	#if __linux__ || __APPLE__
				
				// Capture stack frames:
				void* array[_frames];
				const auto frames = backtrace(array, static_cast<int>(_frames));
				
				// Convert addresses into an array of human-readable strings
				const std::unique_ptr<char*, void(*)(void*)> strings(backtrace_symbols(array, frames), std::free);
				
				for (int i = 0; i < frames; ++i) {
				    result.emplace_back(strings.get()[i]);
				}
				
				if (result.size() == _frames) {
					result.emplace_back("...");
				}
	#else
				debug::log("Stack trace support not been implemented for this platform!", LogType::warning);
	#endif

				(void)_frames; // Suppress unused variable warning.

			}
			catch (const std::exception& e) {
				log(e);
			}
			
			return result;
		}
	};
	
} //

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

#endif //LOUIERIKSSON_DEBUG_HPP