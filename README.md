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

#### Pathfinding Algorithms

| Type                                          |     Status      |
|:----------------------------------------------|:---------------:|
| A-Star (A*)                                   |  Completed ✔️   |
| Breadth-First Search (BFS)                    |  Completed ✔️   |
| Depth-First Search (DFS)                      |  Completed ✔️   |
| Dijkstra's Algorithm                          |   Planned 📝    |
| Enhanced Simple Memory Bounded A-Star (SMA*+) | In Progress 🏗️ |
| Jump-Point Search (JPS)                       | In Progress 🏗️ |

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
| Deferred and Real-Time Solver Garbage Collection |  Completed ✔️   |
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
#### &lt;list&gt;
#### &lt;memory&gt;
#### &lt;queue&gt;
#### &lt;stack&gt;
#### &lt;stdexcept&gt;
#### &lt;type_traits&gt;
#### &lt;unordered_set&gt;
#### &lt;vector&gt;

### References