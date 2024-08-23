# CHDR: Computational Helper for Direction and Routing

## Table of Contents

- [About](#About)
- [Features](#Features)
- [Instructions](#Instructions)
- [Dependencies](#Dependencies)
- [References](#References)

### About

CHDR is a C++ library aiming to offer a comprehensive cross-platform solution for pathfinding in K-dimensions. It provides routing support for a variety of maze types including grid-based, graph-based, recursive, and weighted mazes (fields) using an extensive range of algorithms.

CHDR is designed to be fast and lightweight. Alongside data and algorithmic optimisations, it makes heavy use of metaprogramming to resolve expensive code paths at compile time.

### Features

#### Pathfinding Algorithms

| Type                                          |     Status      |
|:----------------------------------------------|:---------------:|
| A-Star (A*)                                   |  Completed ✔️   |
| Enhanced Simple Memory Bounded A-Star (SMA*+) | In Progress 🏗️ |
| Dijkstra's Algorithm                          |   Planned 📝    |
| Jump-Point Search (JPS)                       |   Planned 📝    |
| Depth-First Search (DFS)                      | In Progress 🏗️ |
| Breadth-First Search (BFS)                    | In Progress 🏗️ |

#### Maze types

| Type      |     Status      |
|:----------|:---------------:|
| Grids     |  ️Completed ✔   |
| Graphs    | In Progress 🏗️ |
| Recursive |   Planned 📝    |
| Fields    |  Completed ✔️   |

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

| Feature                              |     Status      |
|:-------------------------------------|:---------------:|
| Concurrent Capabilities              |   Planned 📝    |
| Compile-Time Routing                 |   Planned 📝    |
| Dynamic Mazes and Obstacle-Avoidance |   Planned 📝    |
| Maze Optimisation (Baking)           | In Progress 🏗️ |
| Compile-Time K-dimensionality        |  Completed ✔️   |
| Runtime K-dimensionality             | In Progress 🏗️ |

### Instructions

#### C++ (17 and above)

As CHDR is a header-only library, you can simply include the headers directly in your project or build and link using your preferred build system.

#### Other languages

At present, CHDR does not offer compatibility with languages other than C++. We prioritise consolidating and enhancing the set of features offered by our library before extending its support to other languages.

Each programming language has its unique characteristics and paradigms. With this in mind, our strategy for supporting a specific language would be determined by its compatibility with C++ and the performance efficiency it offers. The support may be facilitated through language wrappers (interfaces to the library's compiled code) or ports (rewriting parts or all of the library in the target language), whichever is more suited. 

### Dependencies

### References