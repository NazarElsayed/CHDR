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

#### Algorithms

| Type                       |     Status      |
|:---------------------------|:---------------:|
| A-star (A*)                | In Progress 🏗️ |
| Dijkstra's Algorithm       |   Planned 📝    |
| Jump-Point Search (JPS)    |   Planned 📝    |
| Depth-First Search (DFS)   |   Planned 📝    |
| Breadth-First Search (BFS) |   Planned 📝    |
| Fast Marching Method (FMM) |   Planned 📝    |

#### Maze types

| Type      |     Status      |
|:----------|:---------------:|
| Grids     |  Completed ✔️   |
| Graphs    | In Progress 🏗️ |
| Recursive |   Planned 📝    |
| Fields    |  Completed ✔️   |

#### Computational

| Feature                              |     Status      |
|:-------------------------------------|:---------------:|
| Concurrent Capabilities              |   Planned 📝    |
| Compile-Time Routing                 |   Planned 📝    |
| Dynamic Mazes and Obstacle-Avoidance |   Planned 📝    |
| Maze Optimisation (Baking)           | In Progress 🏗️ |

### Instructions

#### C++ (17 and above)

As CHDR is a header-only library, you can simply include the headers directly in your project or build and link using your preferred build system.

#### Other languages

At present, CHDR does not offer compatibility with languages other than C++. We prioritise consolidating and enhancing the set of features offered by our library before extending its support to other languages.

Each programming language has its unique characteristics and paradigms. With this in mind, our strategy for supporting a specific language would be determined by its compatibility with C++ and the performance efficiency it offers. The support may be facilitated through language wrappers (interfaces to the library's compiled code) or ports (rewriting parts or all of the library in the target language), whichever is more suited. 

### Dependencies

### References