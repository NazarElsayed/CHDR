# Introduction

<h1>CHDR 0.1.0</h1>

A comprehensive solution for pathfinding in K-dimensions.


## Table of Contents

- [Background](#background)
- [Manual](docs/manual/manual.md)
- [Roadmap](docs/manual/roadmap.md)
- [License](#license)
- [References](#references)


## Background

The CHDR project aims to be the world's fastest and most efficient pathfinding library.

CHDR is a comprehensive pathfinding solution offering a simple API with ultra-low latency, low memory usage, and high-throughput implementations of many state-of-the-art algorithms commonly used in Robotics, Game Development, GNSS Navigation, Artificial Intelligence, and other high-performance computing applications.

CHDR is designed for compatibility across a wide range of platforms, compilers, and toolchains. It is provided as a standalone, templated C++17 header-only library with a loosely-coupled and modular architecture that stringently adheres to standards. As a result, CHDR compiles under very pedantic conditions and passes our tests with both Valgrind and Google's Sanitizers.

CHDR avoids compromising between usability and performance, utilising metaprogramming as a technique for providing compile-time processing and type agnosticism, while supporting routing in 1D, 2D, 3D, 4D, and higher-dimensionality spaces.


## Usage

For full guides, example implementations, and step-by-step tutorials on how to use the library effectively, please refer to
the [manual](docs/manual/manual.md).


## Contributing

If you encounter any issues or have suggestions for improvements, feel free to post them in the project's issue tracker. Your feedback helps us make CHDR better for the entire community. At this time, please note that derivative works or modifications to the library are not permitted under the current license agreement. Refer to the [license](#license) section for more information.


## License

**[Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International](https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en)**

CHDR is currently licensed under **[CC BY-NC-ND 4.0](https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en)**. Please refer to the license documentation for exact details. For permission regarding use in commercial or derivative works, please contact the developers.

## References

Reinefeld, A. and Marsland, T. A., 1994. Enhanced iterative-deepening search. IEEE Transactions on Pattern Analysis and Machine Intelligence, 16 (7), 701–710. Available at https://doi.org/10.1109/34.297950. [Accessed 3 Oct. 2024].
