#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H
#include <pigpiod_if2.h>

#define motor1_DIR 19
#define motor1_PWM 26
#define motor1_ENA 23
#define motor1_ENB 24

#define motor2_DIR 6
#define motor2_PWM 13
#define motor2_ENA 27
#define motor2_ENB 17

#define PI 3.141592
#include "eric_a_bringup/MotorPacket.h"
#include "geometry_msgs/Twist.h"
//Text_Input
void Text_Input(void);
int PWM_range;
int PWM_frequency;
int PWM_limit;
double Control_cycle;
int Acceleration_ratio;
double Wheel_radius;
double Robot_radius;
int Encoder_resolution;
double Wheel_round;
double Robot_round;

//Motor_Setup
int Motor_Setup(void);
int pinum;
int current_PWM1;
int current_PWM2;
bool current_Direction1;
bool current_Direction2;
int acceleration;

//Interrupt_Setting
void Interrupt_Setiing(void);
volatile int EncoderCounter1;
volatile int EncoderCounter2;
volatile int EncoderCounter1A;
volatile int EncoderCounter1B;
volatile int EncoderCounter2A;
volatile int EncoderCounter2B;
volatile int EncoderSpeedCounter1;
volatile int EncoderSpeedCounter2;
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
int Motor1_Encoder_Sum();
int Motor2_Encoder_Sum();
void Init_Encoder(void);
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void);

//Motor_Controller
void Motor_Controller(int motor_num, bool direction, int pwm);
void Accel_Controller(int motor_num, bool direction, int desired_pwm);

//Example
bool switch_direction;
int Theta_Distance_Flag;
void Switch_Turn_Example(int PWM1, int PWM2);
void Theta_Turn(double Theta, int PWM);
void Distance_Go(double Distance, int PWM);
void Theta_Distance(double Theta, int Turn_PWM, double Distance, int Go_PWM);

//Utiliy
int Limit_Function(int pwm);
double RPM_Value1;
double RPM_Value2;
void RPM_Calculator();
double linear_vel1; //linear_vel_left
double linear_vel2; //linear_vel_right
double linear;
double angular;
double odom_l;
double odom_r;
typedef struct purpose{
    double linear_x=0;
    double angular_z=0;
} purpose;

int PWM_want;
double vel_gap;

void linear_vel();
void Motor_View();
void carcul_packet();
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
eric_a_bringup::MotorPacket packet_msg;
#endif // MOTOR_NODE_H
