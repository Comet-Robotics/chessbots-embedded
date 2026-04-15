#include "robot/robot.h"

void center_test(Robot& r);

// Turns off the motors, so we can easily read data
void sleepy_test(Robot& r);

void line_test(Robot& r);

// Moves to 0 -> 2PI -> 0 -> 4PI -> 0 on a clock to test angular PID
void circle_test(Robot& r);

// Moves to the points on a 1x1 meter square
void square_test(Robot& r);