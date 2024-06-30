# Eller's Maze

<!--toc:start-->
- [Eller's Maze](#ellers-maze)
  - [About](#about)
  - [Usage](#usage)
  - [Thanks](#thanks)
<!--toc:end-->

## About 
This is my implementation of Eller's Algorithm for generating infinite mazes as described by [Jamis Buck](https://weblog.jamisbuck.org/2010/12/29/maze-generation-eller-s-algorithm).
This library generates a bitset where each bit represents a cell, with 'true' indicating a wall and 'false' indicating a non-wall. An [example](https://github.com/autogalkin/ellersmaze/blob/master/example/main.cpp) prints the maze in the terminal.


https://user-images.githubusercontent.com/97976281/226358399-9aaf4e0f-7195-4ffa-bbd7-84ae50b0ced5.mov


## Usage

The library provides a class with a one function:
```cpp
ellrs::maze maze{width};
auto rand_bool =
    [bool_dist = std::uniform_int_distribution<int>{0, 1},
     rand = std::default_random_engine{std::random_device{}()}]() mutable {
        return bool_dist(rand);
    };// you can pass your own random bool generator
while(true) {
    auto  [ vtype, vertical ]   = maze.getline(rand);
    auto  [ htype, horizontal ] = maze.getline(rand);

    assert(vtype == ellrs::line_kind::vertical);
    assert(htype == ellrs::line_kind::horizontal);
}
```

You must start generation with a vertical line, but while using a result horizontal line must be first:
```
                           |   |   |   |
                        _1_|_2_|_3_|_4_|
    print a horisontal ~^  ^~  and then print a vertical wall.
```

Also you can use your own random bool generator to generate a more interested result:
```cpp
bool randomBool() {
   return rand() > (RAND_MAX / 2);
}
maze.getline(&randomBool);
```

## Thanks

- Thank you [Jamis Buck](https://weblog.jamisbuck.org/2010/12/29/maze-generation-eller-s-algorithm) for the algorithm description.
