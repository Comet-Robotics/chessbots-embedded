#ifndef CHESSBOT_PWM_H
#define CHESSBOT_PWM_H

void setupPWM(int pin, int channel);
void writePWM(int channel, int dutyCycle);
int mapPowerToDuty(float power);

#endif