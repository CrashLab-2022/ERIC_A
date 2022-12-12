#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H
#include <pigpiod_if2.h>

#define motor1_DIR 6
#define motor1_PWM 13
#define motor1_ENA 23
#define motor1_ENB 24

#define motor2_DIR 19
#define motor2_PWM 26
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

double left_rpm;
double left_rpm_abs;
double left_rpm_abs2;
double right_rpm;
double right_rpm_abs;
double right_rpm_abs2;
double present_pwm1;
double last_pwm1;
double present_pwm2;
double last_pwm2;
double left_speed;
double right_speed;

//////////////////////////PID control
typedef struct pid_param
{
  double kP=0;
  double kI=0;
  double kD=0;
  double Imax=10;
  double Dmax=10;
} pid_param;

typedef struct pid
{
  double p_out=0;
  double integrator=0;
  double derivative=0;
  double last_input=0;
  double lastderivative=0;

  double output=0;
} pid;

double kp1;
double ki1;
double kd1;
double kp2;
double ki2;
double kd2;
pid data1, data2;
pid_param paramdata1, paramdata2;
double PidContoller1(double goal, double curr, double control_cycle, pid *pid_data, pid_param *pid_paramdata, int error_rat);
double PidContoller2(double goal, double curr, double control_cycle, pid *pid_data, pid_param *pid_paramdata, int error_rat);
void PID_TO_MOTOR();
////////////////////////////////
void linear_vel();
void Motor_View();
void carcul_packet();
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
eric_a_bringup::MotorPacket packet_msg;
#endif // MOTOR_NODE_H