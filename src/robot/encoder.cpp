#ifndef CHESSBOT_ENCODER_CPP
#define CHESSBOT_ENCODER_CPP

#define CLK 2
#define DT 3
#define SW 4

// Associated Header File
#include "robot/encoder.h"
#include "utils/logging.h"

// Built-In Libraries
#include "Arduino.h"

// Will read the values on the encoders. WIP
int counter = 0;
int currentStateCLK;
int previousStateCLK;
char* currentDirection = "none";
unsigned long lastButtonTime = 0; // the last time the output pin was toggled

void setupEncoders() {
    // Set the CLK and DT pins as inputs
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP); // Set the switch pin as input with pull-up resistor

  // Read the initial state of the CLK pin
  previousStateCLK = digitalRead(CLK);
}

void EncoderInputLoop(){
    currentStateCLK = digitalRead(CLK); // Read the current state of the CLK pin

    if(currentStateCLK != previousStateCLK && currentStateCLK == 1) { // If the CLK state has changed
        if(digitalRead(DT) != currentStateCLK){
            counter --;
            currentDirection = "counterclockwise";
        }
        else{
            counter ++;
            currentDirection = "clockwise";
        }
        log((char*)"Direction: ", 4);
        log(currentDirection, 4);
        log((char*)" | Counter: ", 4);
        log(counter, 4);
    }

    // Store the current state as the previous state
    previousStateCLK = currentStateCLK;

    // Read the state of the switch button
    int buttonState = digitalRead(SW); 

    if (buttonState == LOW && (millis() - lastButtonTime) > 50) { // If the button is pressed
        lastButtonTime = millis(); // Update the last button press time
        logln((char*)"Button Pressed", 4);
    }
}
#endif