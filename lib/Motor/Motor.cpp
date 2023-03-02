//
// Created by sdy_zjx on 2023/3/2.
//

#include "Motor.h"
void Motor::drive(char motorType, int val)
{
    if (val > 255)
    {
        val = 255;
    }

    if (val < -255)
    {
        val = -255;
    }

    if (val > 0)
    {
        if (motorType == 'R')
        {
            digitalWrite(R_IN1, 1);
            digitalWrite(R_IN2, 0);
            analogWrite(pwmA_pin, val);
        }
        else
        {
            digitalWrite(L_IN1, 1);
            digitalWrite(L_IN2, 0);
            analogWrite(pwmB_pin, val);
        }
    }
    else
    {
        if (motorType == 'R')
        {
            digitalWrite(R_IN1, 0);
            digitalWrite(R_IN2, 1);
            analogWrite(pwmA_pin, -val);
        }
        else
        {
            digitalWrite(L_IN1, 0);
            digitalWrite(L_IN2, 1);
            analogWrite(pwmB_pin, -val);
        }
    }
}
void Motor::brake()
{
    digitalWrite(L_IN1,LOW);
    digitalWrite(L_IN2,LOW);
    digitalWrite(R_IN1,LOW);
    digitalWrite(R_IN2,LOW);
}
void Motor::forward(int val)
{
    drive('R', val);
    drive('L', val);
}
void Motor::setPins( int A_PIN1, int A_PIN2, int pwma_pin,int B_PIN1, int B_PIN2,int pwmb_pin)
{
    L_IN1 = A_PIN1;
    L_IN2 = A_PIN2;
    R_IN1 = B_PIN1;
    R_IN2 = B_PIN2;
    pwmA_pin=pwma_pin;
    pwmB_pin=pwmb_pin;
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);
    pinMode(pwmA_pin, OUTPUT);
    pinMode(pwmB_pin, OUTPUT);
}