# CHDR: Computational Helper for Direction and Routing

## Table of Contents

- [About](#About)
- [Features](#Features)
- [Instructions](#Instructions)
- [Dependencies](#Dependencies)
- [References](#References)

### About

CHDR is a C++ library aiming to offer a comprehensive cross-platform solution for pathfinding in K-dimensions. It provides routing support for a variety of maze types including grid-based, graph-based, recursive, and weighted (fields) using an extensive range of algorithms.

CHDR is designed to be fast and lightweight. Alongside data and algorithmic optimisations, it utilises metaprogramming to resolve expensive code paths at compile time.

### Features

#### Platform Support

| Platform |   Status   |
|:---------|:----------:|
| Linux    | Tested ✔️  |
| Windows  | Tested ✔️  |
| MacOS    | Untested ❓ |

#### Pathfinding Algorithms

| Type                                                    |     Status      |
|:--------------------------------------------------------|:---------------:|
| A-Star (A*)                                             |  Completed ✔️   |
| Breadth-First Search (BFS)                              |  Completed ✔️   |
| Depth-First Search (DFS)                                |  Completed ✔️   |
| Dijkstra's Algorithm                                    |   Planned 📝    |
| Enhanced Iterative Deepening Search (EIDA*)             |   Planned 📝    |
| Enhanced Simple Memory Bounded A-Star (SMA*+)           |   Planned 📝    |
| Enhanced Simple Memory Bounded Graveyard Search (SMG*+) |   Untested ❓    |
| Graveyard Breadth-First Search (GBFS)                   |  Completed ✔️   |
| Graveyard Depth-First Search (GDFS)                     |  Completed ✔️   |
| Graveyard Jump-Point Search (GJPS)                      |   Planned 📝    |
| Graveyard Search (G*)                                   |  Completed ✔️   |
| Jump-Point Search (JPS)                                 | In Progress 🏗️ |

#### Maze types

| Type      |     Status      |
|:----------|:---------------:|
| Fields    |  Completed ✔️   |
| Graphs    | In Progress 🏗️ |
| Grids     |  ️Completed ✔️  |
| Recursive |   Planned 📝    |

#### Post-Processing

| Type                |   Status   |
|:--------------------|:----------:|
| Path Smoothing      | Planned 📝 |
| Path Simplification | Planned 📝 |

#### Eikonal Solvers

| Type                          |   Status   |
|:------------------------------|:----------:|
| Jump Flooding Algorithm (JFA) | Planned 📝 |

#### Computational

| Feature                                          |     Status      |
|:-------------------------------------------------|:---------------:|
| Compile-Time K-dimensionality                    |  Completed ✔️   |
| Compile-Time Routing                             |   Planned 📝    |
| Concurrent Capabilities                          |   Planned 📝    |
| Dynamic Mazes and Obstacle-Avoidance             |   Planned 📝    |
| Maze Optimisation (Baking)                       | In Progress 🏗️ |
| Runtime K-dimensionality                         | In Progress 🏗️ |

### Instructions

#### C++ (17 and above)

As CHDR is a header-only library, you can simply include the headers directly in your project or build and link using your preferred build system.

#### Other languages

At present, CHDR does not offer compatibility with languages other than C++. We prioritise consolidating and enhancing the set of features offered by our library before extending its support to other languages.

Each programming language has its unique characteristics and paradigms. With this in mind, our strategy for supporting a specific language would be determined by its compatibility with C++ and the performance efficiency it offers. The support may be facilitated through language wrappers (interfaces to the library's compiled code) or ports (rewriting parts or all of the library in the target language), whichever is more suited. 

### Dependencies

CHDR is completely standalone and does not require the use of any third-party libraries.

However, CHDR does require that your project is compatible with a C++17 environment and supports the following standard libraries:

#### &lt;array&gt;
#### &lt;cmath&gt;
#### &lt;cstddef&gt;
#### &lt;cstring&gt;
#### &lt;functional&gt;
#### &lt;future&gt;
#### &lt;limits&gt;
#### &lt;memory&gt;
#### &lt;queue&gt;
#### &lt;stack&gt;
#### &lt;stdexcept&gt;
#### &lt;type_traits&gt;
#### &lt;unordered_set&gt;
#### &lt;vector&gt;

### References

Lovinger, J. and Zhang, X. (2017) ‘Enhanced simplified memory-bounded a star (SMA*+)’, 
EPiC Series in Computing, Vol. 50, pp. 202–212. Available at: https://doi.org/10.29007/v7zc. [Accessed 10 Aug. 2024].