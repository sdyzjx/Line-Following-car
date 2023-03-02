#include "PID.h"
PID::PID(double setpoint_in,double kp_in ,double ki_in, double kd_in,double dt_in) {
    kp=kp_in;
    ki=ki_in;
    kd=kd_in;
    dt=dt_in;
    setpoint=setpoint_in;
}
/** 输入：当前值 **/
double PID::output(double input){
    double value;
    nowerror = setpoint - input;
    sumerror += nowerror*dt;
    derivative = (nowerror - lasterror)/dt;
    value = kp*nowerror + ki*sumerror + kd*derivative;
    lasterror = nowerror;
    value = value + input;
    return value;//下个时刻目标
}
void PID::set_pid(double kp_in ,double ki_in, double kd_in){
    kp = kp_in;
    ki = ki_in;
    kd = kd_in;
}