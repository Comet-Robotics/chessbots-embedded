# ChessBots Embedded
Welcome to the ChessBots Embedded repo!

Adding this stuff for myself for the light sensor task, on what's remaining. Should delete and replace with normal README at the end:
- [ ] Determine if it's needed to have the polled value also be a sum, or if taking a sample at one point is fine.
- [X] Determine how to track what tile each sensor is on. Will likely need this because how else can we tell if one part of the sensor is on a different color from another sensor?
- [X] Determine how we will use auto-alignment upon reaching an edge. Likely, will have the forward motor stay static while the back motor moves until it detects a change.
  - [ ] First, have it stop moving when moving forward upon detecting a change.
  - [ ] Then, have the other motor continue moving until it's aligned.
  - [ ] Implement a slow down function to stop it slowly.
- [ ] Determine how we will center the robot into the middle of the tile.
  - [ ] First, write code to measure the length of a single tile.
    - [ ] To do this, first move the motor forwards until it reaches the edge. Then, move the motor back until the back sensors detect a tile change. The ticks it takes to move backwards will be the length
  - [ ] Then, as normal move the motor forwards or backwards until it reaches the tile edge. Let's say in thsi example we move it backwrds. Then, move it forwards by tileLength/2 ticks, so that now it's centered vertically, albeit not horizontally just yet.
  - [ ] To center it horizontally, now all we do is we rotate it 90 degrees (let's say to the right), and repeat the same process of driving backwards to the edge, then going forwards by tileLength/2. Then rotate left.
- [ ] Actually test the centering on different light levels.
- [ ] One small issue: is there code to rotate perfectly 90 degrees? If not, we might have to code that. <-- update: we will likely have a turn by degrees function so we chilling in this regard
