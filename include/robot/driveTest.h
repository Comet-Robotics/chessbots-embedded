#ifndef CHESSBOT_DRIVETEST_H
#define CHESSBOT_DRIVETEST_H

class MotorEncoderTest
{
public:
    inline void startMotorAndEncoderTest() { init(); }

private:
    int startEncoderA;
    int startEncoderB;
    const int ENCODER_TOLERANCE = 100;

    void init();
    void testLeftMotor();
    void testWait();
    void testRightMotor();
};

#endif