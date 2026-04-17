#ifndef CHESSBOT_DRIVETEST_H
#define CHESSBOT_DRIVETEST_H

class MotorEncoderTest
{
public:
    inline void startMotorAndEncoderTest() { init(); }

private:
    int startEncoderA;
    int _startEncoderB;
    int startEncoderB;
    int prevEncA;
    int prevEncB;
    unsigned long prevTime;
    float maxEncoderVelocity = 0;
    float prevEncVelA;
    float prevEncVelB;
    float maxEncoderAccel = 0;
    unsigned long encoderCheckTimerId;
    bool testSuccessful;
    const int ENCODER_TOLERANCE = 100;

    void init();
    void testLeftMotor();
    void testWait();
    void testRightMotor();
    void testDriveForward();
    void checkMotorDeadzone(bool leftMotor);
    void testMotorDeadzones();
    void checkEncoderVelocity();
    void testDriveDone();
};

#endif