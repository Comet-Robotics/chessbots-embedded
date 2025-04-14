# ChessBots Embedded
Welcome to the ChessBots Embedded repo!

Adding this stuff for myself for the light sensor task, on what's remaining:
- Determine if it's needed to have the polled value also be a sum, or if taking a sample at one point is fine.
- Determine how to track what tile each sensor is on. Will likely need this because how else can we tell if one part of the sensor is on a different color from another sensor?
- Determine how we will use auto-alignment upon reaching an edge. Likely, will have the forward motor stay static while the back motor moves until it detects a change.
- Determine how to center the robot about a tile.
- Actually test the centering on different light levels.
- Determine how we will center the robot into the middle of the tile.
  - First, write code to measure the length of a single tile.
  - 

For more information about the project and how to contribute, please take a look at the [Wiki](https://github.com/Comet-Robotics/chessbot-embedded/wiki)
