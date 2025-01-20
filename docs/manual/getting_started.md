# Getting Started

## Installation

As CHDR is a header-only library, no complex platform-dependent setup is required. Simply include the headers in your
project or integrate the library into your build system with ease.

### Example CMake

```cmake
# Add the CHDR library:
add_library(libchdr INTERFACE path_to_chdr/chdr.hpp)
target_include_directories(libchdr INTERFACE path_to_chdr)

# Link CHDR to your project:
target_link_libraries(YourProject PRIVATE
        libchdr
)
```

## Dependencies

CHDR requires a C++ environment (version 17 or above) with support for the standard library.

## Compatibility

| Platform |   Status   |
|:---------|:----------:|
| Windows  | Tested ✔️  |
| Linux    | Tested ✔️  |
| MacOS    | Untested ❓ |


| Compiler |   Status   |
|:---------|:----------:|
| GCC      | Tested ✔️  |
| Clang    | Tested ✔️  |
| ICPX     | Tested ✔️  |
| MSVC     | Untested ❓ |


| Dynamic Analysis  |   Status   |
|:------------------|:----------:|
| Google Sanitizers | Passing ✔️ |
| Valgrind          | Passing ✔️ |

### Other Languages

At present CHDR does not offer compatibility with languages other than C++. We prioritise consolidating and enhancing the set of features offered by our library before extending its support to other languages.

Each programming language has its unique characteristics and paradigms. With this in mind our strategy for supporting a specific language would be determined by its compatibility with C++ and the performance efficiency it offers. The support may be facilitated through language wrappers (interfaces to the library's compiled code) or ports (rewriting parts or all of the library in the target language), whichever is more suited. 