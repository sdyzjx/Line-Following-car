//
// Created by sdy_zjx on 2023/3/2.
//

#ifndef SMART_CAR_PID_H
#define SMART_CAR_PID_H


class PID {
public:
    double nowerror = 0.0;
    double setpoint;
    double sumerror = 0.0;
    double lasterror = 0.0;
    double kp, ki, kd;
    double dt;
    double derivative;
    PID(double setpoint_in,double kp_in ,double ki_in, double kd_in,double dt_in);
    double output(double input);
    void set_pid(double kp_in ,double ki_in, double kd_in);
};


#endif //SMART_CAR_PID_H
