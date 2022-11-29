#include <ros/ros.h>
#include <eric_a_bringup/eric_a_control.h>
#include <fstream>
#include <cmath>
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

purpose purposevel;

void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/ERIC_A/eric_a_bringup/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)                                                                                                                                
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PWM_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Control_cycle = atof(line.substr(found+2).c_str()); break;
      case 4: Acceleration_ratio = atof(line.substr(found+2).c_str()); break;
      case 5: Wheel_radius = atof(line.substr(found+2).c_str()); break;
      case 6: Robot_radius = atof(line.substr(found+2).c_str()); break;
      case 7: Encoder_resolution = atof(line.substr(found+2).c_str()); break;
      case 8: kp = atof(line.substr(found+2).c_str()); break;
      case 9: ki = atof(line.substr(found+2).c_str()); break;
      case 10: kd = atof(line.substr(found+2).c_str()); break;
          //case :  = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}
int Motor_Setup(void)
{
  pinum=pigpio_start(NULL, NULL);
  
  if(pinum<0)
  {
    ROS_INFO("Setup failed");
    ROS_INFO("pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_DIR, PI_OUTPUT);
  set_mode(pinum, motor2_DIR, PI_OUTPUT);
  set_mode(pinum, motor1_PWM, PI_OUTPUT);
  set_mode(pinum, motor2_PWM, PI_OUTPUT);
  set_mode(pinum, motor1_ENA, PI_INPUT);
  set_mode(pinum, motor1_ENB, PI_INPUT);
  set_mode(pinum, motor2_ENA, PI_INPUT);
  set_mode(pinum, motor2_ENB, PI_INPUT);

  gpio_write(pinum, motor1_DIR, PI_LOW);
  gpio_write(pinum, motor2_DIR, PI_LOW);

  set_PWM_range(pinum, motor1_PWM, PWM_range);
  set_PWM_range(pinum, motor2_PWM, PWM_range);
  set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
  set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);

  set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

  current_PWM1 = 0;
  current_PWM2 = 0;

  current_Direction1 = true;
  current_Direction2 = true;

  acceleration = PWM_limit/(Acceleration_ratio);

  ROS_INFO("Setup Fin");
  return 0;
}
void Interrupt_Setting(void)
{
    callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
}
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)  //motor right
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1A ++;
  else EncoderCounter1A --;
  EncoderSpeedCounter1 ++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1B ++;
  else EncoderCounter1B --;
  EncoderSpeedCounter1 ++;
}
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick) // motor left
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2A --;
  else EncoderCounter2A ++;
  EncoderSpeedCounter2 ++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2B --;
  else EncoderCounter2B ++;
  EncoderSpeedCounter2 ++;
}
int Motor1_Encoder_Sum()
{
  EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
  return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
  EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
  return EncoderCounter2;
}
void Init_Encoder(void)
{
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;
  EncoderCounter1A = 0;
  EncoderCounter1B = 0;
  EncoderCounter2A = 0;
  EncoderCounter2B = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void)
{
  Text_Input();
  Motor_Setup();
  Init_Encoder();
  Interrupt_Setting();

  Wheel_round = 2*M_PI*Wheel_radius;
  Robot_round = 2*M_PI*Robot_radius;

  ROS_INFO("PWM_range %d", PWM_range);
  ROS_INFO("PWM_frequency %d", PWM_frequency);
  ROS_INFO("PWM_limit %d", PWM_limit);
  ROS_INFO("Control_cycle %f", Control_cycle);
  ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
  ROS_INFO("Initialize Complete");

  printf("\033[2J");  
}

void Motor_Controller(int motor_num, bool direction, int pwm)
{
  int local_PWM = Limit_Function(pwm);

  if(motor_num == 1)  //motor left
  {
    if(direction == true)
    {
      gpio_write(pinum, motor1_DIR, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = true;
      linear_vel1 = -((2*57.5*M_PI*RPM_Value1)/60); //linear velocity 
    }
    else if(direction == false)
    {
      gpio_write(pinum, motor1_DIR, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = false;
      linear_vel1 = ((2*57.5*M_PI*RPM_Value1)/60); //linear velocity 
    }
  }
  
  else if(motor_num == 2) //motor right
  {
   if(direction == true)
   {
     gpio_write(pinum, motor2_DIR, PI_LOW);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = true;
     linear_vel2 = ((2*57.5*M_PI*RPM_Value2)/60);
   }
   else if(direction == false)
   {
     gpio_write(pinum, motor2_DIR, PI_HIGH);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = false;
     linear_vel2 = -((2*57.5*M_PI*RPM_Value2)/60);
   }
  }
}

int Limit_Function(int pwm)
{
  int output;
  if (pwm > PWM_limit*2)
  {
    output = PWM_limit;
    ROS_WARN("PWM too fast!!!");
  }
  else if(pwm > PWM_limit)output = PWM_limit;
  else if(pwm < 0)
  {
	output = 0;
    ROS_WARN("trash value!!!");
  }
  else output = pwm;
  return output; 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void RPM_Calculator()
{
  RPM_Value1 = (EncoderSpeedCounter1*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter1 = 0;
  RPM_Value2 = (EncoderSpeedCounter2*(60*Control_cycle))/(Encoder_resolution*4);
  EncoderSpeedCounter2 = 0;
}
void carcul_packet()
{
 linear = (linear_vel1 + linear_vel2)/2;
 angular =(linear_vel2 - linear_vel1)/(Robot_radius*2)*1000;
 odom_l = Wheel_radius*packet_msg.encod[0]*(2*M_PI)/(Encoder_resolution*4);
 odom_r = Wheel_radius*packet_msg.encod[1]*(2*M_PI)/(Encoder_resolution*4);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
  purposevel.linear_x=msg->linear.x;
  purposevel.angular_z=msg->angular.z;
  left_speed = purposevel.linear_x - (purposevel.angular_z*Robot_radius*0.001);
  right_speed = purposevel.linear_x + (purposevel.angular_z*Robot_radius*0.001);
  left_rpm = (left_speed*60)/ (2*M_PI*Wheel_radius);
  right_rpm = (right_speed*60)/(2*M_PI*Wheel_radius);
}



double PidContoller(double goal, double curr, double control_cycle, pid *pid_data, pid_param *pid_paramdata, int error_rat)
{
  pid_paramdata->kP =kp;
  pid_paramdata->kI =ki;
  pid_paramdata->kD = kd;
  double error = goal - curr;
  double dt = 1/control_cycle;
  
  if (fabs(error) < error_rat)
    error = 0;

  pid_data->p_out = pid_paramdata->kP * error;
  double p_data = pid_data->p_out ;

  pid_data->integrator += (error * pid_paramdata->kI) * dt;
  pid_data->integrator = constrain(pid_data->integrator, -pid_paramdata->Imax, pid_paramdata->Imax);
  double i_data = pid_data->integrator;

  double filter = 15.9155e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter / ROS_INFO(" goal : %f, curr: %f", goal,curr);
  // ROS_INFO(" error : %f", error);= 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  pid_data->derivative = (goal - pid_data->last_input) / dt;
  pid_data->derivative = pid_data->lastderivative + (dt / (filter + dt)) * (pid_data->derivative - pid_data->lastderivative);
  pid_data->last_input = goal;
  pid_data->lastderivative = pid_data->derivative;
  double d_data = pid_paramdata->kD * pid_data->derivative;
  d_data = constrain(d_data, -pid_paramdata->Dmax, pid_paramdata->Dmax);

  double output = p_data + i_data + d_data;
  pid_data->output = output;

  return pid_data->output;
}

void PID_TO_MOTOR()
{
  if(left_rpm < 0){ //left_motor DIR
      left_rpm_abs = -left_rpm;
      present_pwm1 = PidContoller(left_rpm_abs, RPM_Value2, Control_cycle, &data1, &paramdata1, 1); //오차에 대한 output rpm
      last_pwm1+=present_pwm1;
      Motor_Controller(1, true, last_pwm1);
    }
    else{
      present_pwm1 = PidContoller(left_rpm, RPM_Value2, Control_cycle, &data1, &paramdata1, 1); //오차에 대한 output rpm
      last_pwm1+=present_pwm1;
      Motor_Controller(1, false, last_pwm1); 
    }

    if(right_rpm < 0){  //right_motor DIR 
      right_rpm_abs = -right_rpm;
      present_pwm2 = PidContoller(right_rpm_abs, RPM_Value1, Control_cycle, &data2, &paramdata2, 1);
      last_pwm2 += present_pwm2;
      Motor_Controller(2, false, last_pwm2);
    }
    else{
      present_pwm2 = PidContoller(right_rpm, RPM_Value1, Control_cycle, &data2, &paramdata2, 1);
      last_pwm2 += present_pwm2;
      Motor_Controller(2, true, last_pwm2);
    }
}

void Motor_View()
{
  
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
	printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
	printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value1, RPM_Value2);
	printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
  printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
	printf("Acc  :%10.0d\n", acceleration);
	printf("\n");
	printf("linear vel1: %10.0f || linear vel2: %10.0f\n",linear_vel1, linear_vel2); 
  printf("liner_vel: %10.0f || angular_vel: %10.0f\n",linear, angular);
  printf("odom_l: %10.0f || odom_r: %10.0f\n",odom_l, odom_r);
}

int main(int argc, char** argv)
{
  ROS_INFO("ASDF");
  ros::init(argc, argv, "eric_a_control");
  ros::NodeHandle nh;
  ros::Publisher packet_pub = nh.advertise<eric_a_bringup::MotorPacket>("motor_packet",1, false);
  ros::Subscriber vel_sub = nh.subscribe("motor_input", 10, cmd_vel_callback);
  Initialize();
  
  ros::Rate loop_rate(Control_cycle);
  while(ros::ok())
  {
    carcul_packet();  //linear, angular odom 계산
    Motor_View();
    RPM_Calculator(); //rpm 계산 -> 현재 모터에 대한 
    packet_msg.vw[0] = linear;
    packet_msg.vw[1] = angular;
    packet_msg.encod[0] = last_pwm1;
    packet_msg.encod[1] = last_pwm2;
    packet_msg.odo[0] = odom_l;
    packet_msg.odo[1] = odom_r;
    packet_pub.publish(packet_msg);
  
    PID_TO_MOTOR();
    ros::spinOnce();
    loop_rate.sleep();
  }
  Motor_Controller(1, true, 0);
  Motor_Controller(2, false, 0);
  return 0;
}