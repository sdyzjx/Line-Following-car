/*-----------------------------------
 * 导入库
 -----------------------------------*/
#include <Arduino.h>
#include "../lib/PID/PID.h"
#include "../lib/Motor/Motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/*-----------------------------------
 * 针脚/常量定义
 -----------------------------------*/
#define l_motor_1 27
#define l_motor_2 26
#define r_motor_1 25
#define r_motor_2 33
#define l_motor_pwm 32
#define r_motor_pwm 14
#define ENCODERr_A_PIN 23
#define ENCODERr_B_PIN 22
#define ENCODERl_A_PIN 15
#define ENCODERl_B_PIN 13
#define TARGET_SPEED 600
/*-----------------------------------
 * 全局变量定义
 -----------------------------------*/
double kp=0.5 ,ki=0 ,kd=0 ,dt=0.3;
Motor motor;
PID rps_l_pid(TARGET_SPEED, 0.5, ki, 0, dt); //左轮pid
PID rps_r_pid(TARGET_SPEED, 0.5, ki, 0, dt); //右轮pid
//PID x_cam_diff_control(0, 0.5, 0, 0, dt);
//PID x_cam_slope_control(0, 0.5, 0, 0, dt);
PID x_IR_diff_control(3, 0.5, 0, 0, dt);
int pulse_number_l = 0;
int pulse_number_r = 0;
int stop_flag = 0;
double rps_r = 0;
double rps_l = 0;
int l_i = 0;
int r_i = 0;
double x_diff = 3;
double slope = 0;
float x_diff_old = 3;
/*-----------------------------------
 * 速度获取/处理
 -----------------------------------*/
void read_quadrature_r()
{
    if (digitalRead(ENCODERr_A_PIN) == LOW) { //如果是下降沿触发的中断
        if (digitalRead(ENCODERr_B_PIN) == LOW)      pulse_number_r++;//根据另外一相电平判定方向
        else      pulse_number_r--;
    }
    else {   //如果是上升沿触发的中断
        if (digitalRead(ENCODERr_B_PIN) == LOW)      pulse_number_r--; //根据另外一相电平判定方向
        else     pulse_number_r++;
    }
}
void read_quadrature_l()
{
    if (digitalRead(ENCODERl_A_PIN) == LOW) { //如果是下降沿触发的中断
        if (digitalRead(ENCODERl_B_PIN) == LOW)      pulse_number_l++;//根据另外一相电平判定方向
        else      pulse_number_l--;
    }
    else {   //如果是上升沿触发的中断
        if (digitalRead(ENCODERl_B_PIN) == LOW)      pulse_number_l--; //根据另外一相电平判定方向
        else     pulse_number_l++;
    }
}
void speed( void * parameter ) /*获得速度*/{
    double rps1,rps2;
    rps_r = pulse_number_r*100/(90*12*0.005);
    rps_l = pulse_number_l*100/(90*12*0.005);
    //Serial.println(rps_r);
    //Serial.println(rps_l);

    pulse_number_l = 0;
    pulse_number_r = 0;
}
/*-----------------------------------
 * 电机稳速pid
 -----------------------------------*/
void control_time( void * parameter ){
   // Serial.println(rps_l);
    //Serial.println(rps_r);
    if (stop_flag==0){
        double rps_l_output_val = rps_l_pid.output((double)rps_l);
        double rps_r_output_val = rps_r_pid.output((double)rps_r);
        //motor.line_following(int(1));
        //Serial.println((int)(rps_l_output_val*0.21));
        if (rps_l_output_val < 200) rps_l_output_val = 200;
        if (rps_r_output_val < 200) rps_r_output_val = 200;

        int pwm_l_output_val = (int)(rps_l_output_val*0.21);
        int pwm_r_output_val = (int)(rps_r_output_val*0.21);
//-----------------------------------------------------------
        if (abs(rps_l - rps_l_pid.setpoint) >= 50){//区间增量
            if(rps_l - rps_l_pid.setpoint > 0){
                l_i --;
                pwm_l_output_val += l_i;
            }
            if(rps_l - rps_l_pid.setpoint < 0){
                l_i ++;
                pwm_l_output_val += l_i;
            }
        }
        if (abs(rps_l - rps_l_pid.setpoint) < 50){
            l_i = 0;
        }

        if (abs(rps_r - rps_r_pid.setpoint) >= 50){
            if(rps_r - rps_r_pid.setpoint > 0){
                r_i --;
                pwm_r_output_val += r_i;
            }
            if(rps_r - rps_r_pid.setpoint < 0){
                r_i ++;
                pwm_r_output_val += r_i;
            }
        }
        if (abs(rps_r - rps_r_pid.setpoint) < 50){
            r_i = 0;
        }
//----------------------------------
        if (pwm_l_output_val>=255) pwm_l_output_val = 255;
        if (pwm_r_output_val>=255) pwm_r_output_val = 255;
        double x_diff_val = x_IR_diff_control.output_pos(x_diff);
        int bias = 2*x_diff_val*50;
        Serial.print("r_i:");
        Serial.println(r_i);
        motor.drive('L' ,pwm_l_output_val+bias);
        motor.drive('R' ,pwm_r_output_val-bias);
    }
    else{
        motor.brake();
    }
}


/*-----------------------------------
 * 差速控制PID(IR)
 -----------------------------------*/
void x_diff_IR( void * parameter ) {
    double x_diff_val = x_IR_diff_control.output_pos(x_diff);
   // Serial.print(x_diff);
    //Serial.print(x_IR_diff_control.setpoint);
    //Serial.print("x_diff_val");
    //Serial.println(x_diff_val);
    double speed_diff = x_diff_val;
    double r_control_val = -(speed_diff / 3 * 100);
    double l_control_val = (speed_diff / 3 * 100);
    //Serial.print("r_control_val:");
    //Serial.println(r_control_val);
    //Serial.print("l_control_val:");
    //Serial.println(l_control_val);
    //Serial.print("x_diff_val:");
    //Serial.println(x_diff_val);
    rps_r_pid.setpoint = TARGET_SPEED + r_control_val;
    rps_l_pid.setpoint = TARGET_SPEED + l_control_val;
}
/*-----------------------------------
 * 差速控制PID(camera)

void x_diff_cam( void * parameter ) {
    double diff_val = x_cam_diff_control.output((double)x_diff);
    double slope_val = x_cam_slope_control.output((double)x_diff);
    double speed_diff = diff_val / 120 + slope_val / 1.57;
    double r_control_val = -speed_diff * 1200;
    double l_control_val = speed_diff * 1200;
    rps_l_pid.setpoint = r_control_val;
    rps_r_pid.setpoint = l_control_val;
}
* -----------------------------------/
/*-----------------------------------
 * 传感器数据获取
 -----------------------------------*/
String recFun(){
    //static char data[50] = {0};
    String rec_str = "";
    if(Serial2.available()&& (char)Serial2.read() == 'd')
    {
        rec_str = 'd'+Serial2.readStringUntil('*') + '*';
        //Serial.println(rec_str);
    }
    if(Serial.available()&& (char)Serial.read() == 'c')
    {
        rec_str = 'c'+Serial.readStringUntil('@') + '@';
        return rec_str;
    }
    else if(Serial2.available()&& (char)Serial2.read() == 'a')
    {
        rec_str = 'a'+Serial2.readStringUntil('b') + 'b';
        if (rec_str.length() == 7) {
           // Serial.println(rec_str);
            return rec_str;
        }
        else {
            rec_str = "";
            return rec_str;
        }

    }
    else
    {
        return rec_str;
    }

}
String explain_str(char flag_start, char flag_end, String input_str) {
    int index_start = 0;
    int index_end = 0;
    String final_data = "";
    char str_array[100] = {0};
    int len = input_str.length() + 1;
    input_str.toCharArray(str_array,len);
    index_start = input_str.indexOf(flag_start);
    index_end = input_str.indexOf(flag_end);
    for ( int i = index_start+1; i < index_end; i++ ){
        final_data += str_array[i];
    }
    return final_data;
}
/*-----------------------------------
 * 上位机相关函数
 -----------------------------------*/
void looprec(){
    String command_str = "";
    String IR_rec = "";
    double rps_kp = 0, rps_ki = 0, rps_kd = 0;
    //double cam_slope_p = 0, cam_slope_i = 0, cam_slope_d = 0;
    double ir_diff_p = 0, ir_diff_i = 0, ir_diff_d = 0;
    String recieve_str;
    recieve_str = recFun();
    if ( recieve_str.charAt(0) == 'c') {
        command_str = recieve_str;
        //String cam_diff = "";
        //String cam_slope = "";
        String rps_pid = "";
        String IR_diff_pid = "";
        rps_pid = explain_str( '^','%',command_str) + '%';
        IR_diff_pid = explain_str('%', '@', command_str) + '@';

        ir_diff_p = explain_str('P', 'I', IR_diff_pid).toDouble();
        ir_diff_i = explain_str('I', 'D', IR_diff_pid).toDouble();
        ir_diff_d = explain_str('D', '@', IR_diff_pid).toDouble();

        rps_kp = explain_str('p', 'i', rps_pid).toDouble();
        rps_ki = explain_str('i', 'd', rps_pid).toDouble();
        rps_kd = explain_str('d', '%', rps_pid).toDouble();

        rps_l_pid.set_pid(rps_kp, rps_ki, rps_kd);
        rps_r_pid.set_pid(rps_kp, rps_ki, rps_kd);
        x_IR_diff_control.set_pid(ir_diff_p, ir_diff_i, ir_diff_d);
        //Serial.println(rps_l_pid.kd);
    }
    if ( recieve_str.charAt(0) == 'd' ) {
        x_diff = explain_str( 'd','*',recieve_str).toInt();
    }
    if ( recieve_str.charAt(0) == 'a' ) {
        IR_rec = explain_str('a', 'b', recieve_str);
        if (IR_rec.length() == 5) {
            //Serial.print("recieve_str:");
            //Serial.println(IR_rec);
            int pos = 0, count = 0;
            for (int i = 0; i < 5; ++i) {
                if (IR_rec.charAt(i) == '0') {
                    pos = pos + (i+1);
                    //Serial.print("pos:");
                    //Serial.println(pos);
                    count++;
                }
            }
            if (count != 0) {
                x_diff = (double)pos / (double)count;
                x_diff_old = x_diff;
            }

            else {
                x_diff = 3;
            }

            //Serial.print("x_diff:");
            //Serial.println(x_diff);
        }
    }
}
union SeFrame
{
    float Float; //Arduino中float占4个字节，对应于simulink中的single（32位，4个字节）
    byte Byte[4];
};
SeFrame Sefram;
void sendFloat( float FLOAT)
{
    Sefram.Float= FLOAT;
    Serial.write(Sefram.Byte[0]);
    Serial.write(Sefram.Byte[1]);
    Serial.write(Sefram.Byte[2]);
    Serial.write(Sefram.Byte[3]);
}
void send_wave(void * parameter)/*发送给上位机*/{
    //定义通道名帧头帧尾
    char* frameNameHead = "AABBCC";
    char* frameNameEnd = "CCBBAA";
    //定义数据帧头帧尾
    char* frameDataHead = "DDEEFF";
    char* frameDataEnd = "FFEEDD";
    //定义通道名
    char* name= "x_diff,rps_l,rps_r,P,I,D,dt";
    //通过串口1，向上位机发送数据
    Serial.write(frameNameHead);
    Serial.write(name);
    Serial.write(frameNameEnd);
    Serial.write(frameDataHead);
    float a = 1.0;
    sendFloat((float)x_diff);
    sendFloat((float)rps_l);
    sendFloat((float)rps_r);
    sendFloat((float)x_IR_diff_control.kp);
    sendFloat((float)x_IR_diff_control.ki);
    sendFloat((float)x_IR_diff_control.kd);
    sendFloat((float)dt);
    Serial.write(frameDataEnd);
}
/*-----------------------------------
 * 参数初始化
 -----------------------------------*/
int flag = 0;
void setup() {
    Serial.begin(9600);
    Serial2.begin(115200);
    motor.setPins(l_motor_1 ,l_motor_2,l_motor_pwm,r_motor_1, r_motor_2,r_motor_pwm);
    TimerHandle_t Speed;
    Speed = xTimerCreate("myTimer",30,pdTRUE,(void*)1,speed);
    TimerHandle_t control;
    control = xTimerCreate("myTimer",30,pdTRUE,(void*)1,control_time);
    TimerHandle_t Send;
    Send = xTimerCreate("Send",500,pdTRUE,(void*)1,send_wave);
    TimerHandle_t diff_control;
    diff_control = xTimerCreate("diff_control", 30, pdTRUE, (void*)1, x_diff_IR);
    xTimerStart(control,40);
    xTimerStart(Send,40);
    xTimerStart(Speed,50);
    //xTimerStart(diff_control,40);
    attachInterrupt(ENCODERl_A_PIN, read_quadrature_l, FALLING);
    attachInterrupt(ENCODERr_A_PIN, read_quadrature_r, FALLING);

}
void loop() {
    looprec();
}
