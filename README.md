# CHDR 0.1.0

## Table of Contents

- [Background](#background)
- [Installation](#installation)
 - [Example CMake](#example-cmake)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [Roadmap](#roadmap)
 - [Solvers](#solvers)
 - [Maze types](#maze-types)
 - [Post-Processing](#post-processing)
 - [Eikonal Solvers](#eikonal-solvers)
 - [Computational](#computational)
 - [Compatibility](#compatibility)
- [Other Languages](#other-languages)
- [License](#license)
- [References](#references)

## Background

CHDR is a C++ library offering a comprehensive cross-platform solution for pathfinding in K-dimensions. It provides routing support for a variety of maze types and an extensive range of solvers.

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

## Usage

Once CHDR is linked to your project, simply include the header file to get started.

```cpp
#include <chdr.hpp>
```

For more information, visit our [manual](docs/manual.html).

## Dependencies

CHDR requires a C++ environment (version 17 or above) with support for the standard library.

## Roadmap

### Solvers

| Type                                                     |    Status     |
|:---------------------------------------------------------|:-------------:|
| A-Star (A*)                                              | Completed ✔️  |
| Best-First Search (B*)                                   | Completed ✔️  |
| Breadth-First Search (BFS)                               | Completed ✔️  |
| Depth-First Search (DFS)                                 | Completed ✔️  |
| Dijkstra's Algorithm                                     |  Planned 📝   |
| Enhanced Iterative-Deepening A* (IDA*+)                  | Completed ✔️  |
| Enhanced Iterative-Deepening B* (IDB*+)                  | Completed ✔️  |
| Enhanced Iterative-Deepening Depth-First Search (IDDFS+) | Completed ✔️  |
| Flood Fill                                               | Completed ✔️  |
| Fringe Search (F*)                                       | Completed ✔️  |
| Graveyard Breadth-First Search (GBFS)                    | Completed ✔️  |
| Graveyard Depth-First Search (GDFS)                      | Completed ✔️  |
| Graveyard Jump-Point Search (GJPS)                       | Completed ✔️  |
| Graveyard Search (G*)                                    | Completed ✔️  |
| Iterative-Deepening A* (IDA*)                            | Completed ✔️  |
| Iterative-Deepening B* (IDB*)                            | Completed ✔️  |
| Iterative-Deepening Depth-First Search (IDDFS)           | Completed ✔️  |
| Jump-Point Search (JPS)                                  | Completed ✔️️ |


### Maze types

| Type      |    Status    |
|:----------|:------------:|
| Fields    | Completed ✔️ |
| Graphs    | Completed ✔️ |
| Grids     | Completed ✔️ |
| Tree      |  Planned 📝  |
| Mesh      |  Planned 📝  |
| Hierarchy |  Planned 📝  |


### Post-Processing

| Type                |   Status   |
|:--------------------|:----------:|
| Path Smoothing      | Planned 📝 |
| Path Simplification | Planned 📝 |


### Eikonal Solvers

| Type                          |   Status   |
|:------------------------------|:----------:|
| Jump Flooding Algorithm (JFA) | Planned 📝 |


### Computational

| Feature                              |     Status      |
|:-------------------------------------|:---------------:|
| Compile-Time K-dimensionality        |  Completed ✔️   |
| Compile-Time Routing                 |   Planned 📝    |
| Concurrent Capabilities              | In Progress 🏗️ |
| Dynamic Mazes and Obstacle-Avoidance |   Planned 📝    |
| Maze Optimisation (Baking)           | In Progress 🏗️ |
| Runtime K-dimensionality             | In Progress 🏗️ |


### Compatibility

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

## License

The CHDR project is licensed under
 [Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International](https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en).

## References

Reinefeld, A. and Marsland, T. A., 1994. Enhanced iterative-deepening search. IEEE Transactions on Pattern Analysis and Machine Intelligence, 16 (7), 701–710. Available at https://doi.org/10.1109/34.297950. [Accessed 3 Oct. 2024].
