//
// Created by sdy_zjx on 2023/3/2.
//

#ifndef SMART_CAR_MOTOR_H
#define SMART_CAR_MOTOR_H
#include "Arduino.h"

class Motor
{
public:
    float kp = 1;
    int speed_car_turn = 100;
    int line_base_speed = 150;
    int pwmA_pin;
    int pwmB_pin;
    void setPins( int A_PIN1, int A_PIN2, int pwma_pin,int B_PIN1, int B_PIN2,int pwmb_pin);
    void drive( char motorTyoe, int val );

    void forward(int val);
    void left();
    void right();
    void brake();

private:
    int L_IN1;
    int L_IN2;
    int R_IN1;
    int R_IN2;
};


#endif //SMART_CAR_MOTOR_H
