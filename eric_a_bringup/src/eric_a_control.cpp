/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
#include <ros/ros.h>
#include <eric_a_bringup/eric_a_control.h>
#include <fstream>

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

  switch_direction = true;
  Theta_Distance_Flag = 0;

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
void Accel_Controller(int motor_num, bool direction, int desired_pwm)
{
  bool local_current_direction;
  int local_PWM;
  int local_current_PWM;

  if(motor_num == 1)
  {
    local_current_direction = current_Direction1;
    local_current_PWM = current_PWM1;
  }
  else if(motor_num == 2)
  {
    local_current_direction = current_Direction2;
    local_current_PWM = current_PWM2;
  }

  if(direction == local_current_direction)
  {
    if(desired_pwm > local_current_PWM)
    {
      local_PWM = local_current_PWM + acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else if(desired_pwm < local_current_PWM)
    {
      local_PWM = local_current_PWM - acceleration;
      Motor_Controller(motor_num, direction, local_PWM);
    }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
  else
  {
	  if(desired_pwm >= 0)
	  {
      local_PWM = local_current_PWM - acceleration;
      if(local_PWM <= 0)
      {
        local_PWM = 0;
        Motor_Controller(motor_num, direction, local_PWM);
      }
      else Motor_Controller(motor_num, local_current_direction, local_PWM);
	  }
    else
    {
      local_PWM = local_current_PWM;
      Motor_Controller(motor_num, direction, local_PWM);
    }
  }
}

void Switch_Turn_Example(int PWM1, int PWM2)
{
  int local_PWM1 = Limit_Function(PWM1);
  int local_PWM2 = Limit_Function(PWM2);
  if(switch_direction == true)
  {
    Motor_Controller(1, switch_direction, local_PWM1);
    Motor_Controller(2, switch_direction, local_PWM2);
    switch_direction = false;
    ROS_INFO("true");
  }
  else
  {
    Motor_Controller(1, switch_direction, local_PWM1);
    Motor_Controller(2, switch_direction, local_PWM2);
    switch_direction = true;
    ROS_INFO("false");
  }
}
void Theta_Turn(double Theta, int PWM)
{
  double local_encoder;
  int local_PWM = Limit_Function(PWM);
  if(Theta_Distance_Flag == 1)
  {
      Init_Encoder();
      Theta_Distance_Flag = 2;
  }
  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
  if(Theta > 0)
  {
    local_encoder = (Encoder_resolution*4/360)*(Robot_round/Wheel_round)*Theta;
    Motor_Controller(1, false, local_PWM);
    Motor_Controller(2, false, local_PWM);
    //Accel_Controller(1, false, local_PWM);
    //Accel_Controller(2, false, local_PWM);
  }
  else
  {
    local_encoder = -(Encoder_resolution*4/360)*(Robot_round/Wheel_round)*Theta;
    Motor_Controller(1, true, local_PWM);
    Motor_Controller(2, true, local_PWM);
    //Accel_Controller(1, true, local_PWM);
    //Accel_Controller(2, true, local_PWM);
  }

  if(EncoderCounter1 > local_encoder)
  {
    Init_Encoder();
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    Theta_Distance_Flag = 3;
  }
}
void Distance_Go(double Distance, int PWM)
{
  double local_encoder = (Encoder_resolution*4*Distance)/Wheel_round;
  int local_PWM = Limit_Function(PWM);
  bool Direction = true;
  if(Distance < 0)
  {
    Direction = false;
    local_encoder = -local_encoder;
  }
  if(Theta_Distance_Flag == 3)
  {
      Init_Encoder();
      Theta_Distance_Flag = 4;
  }
  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
  if(EncoderCounter1 < local_encoder)
  {
    if(Direction==true)
    {
      Motor_Controller(1, false, local_PWM);
      Motor_Controller(2, true, local_PWM);
      //Accel_Controller(1, false, local_PWM);
      //Accel_Controller(2, true, local_PWM);
    }
    else
    {
      Motor_Controller(1, true, local_PWM);
      Motor_Controller(2, false, local_PWM);
      //Accel_Controller(1, true, local_PWM);
      //Accel_Controller(2, false, local_PWM);
    }
  }
  else
  {
    Init_Encoder();
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    //Accel_Controller(1, true, 0);
    //Accel_Controller(2, true, 0);
    Theta_Distance_Flag = 0;
  }
}
void Theta_Distance(double Theta, int Turn_PWM, double Distance, int Go_PWM)
{
  if(Theta_Distance_Flag == 0)
  {
    Theta_Distance_Flag = 1;
  }
  else if(Theta_Distance_Flag == 1 || Theta_Distance_Flag == 2)
  {
    Theta_Turn(Theta, Turn_PWM);
  }
  else if(Theta_Distance_Flag == 3 || Theta_Distance_Flag == 4)
  {
    Distance_Go(Distance, Go_PWM);
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
 angular =(linear_vel2 - linear_vel1)/Robot_radius*2;
 odom_l = linear_vel2/(Control_cycle*Wheel_radius);
 odom_r = linear_vel1/(Control_cycle*Wheel_radius);
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
  ROS_INFO("i heard :[%f]", msg->linear.x);
  purposevel.linear_x=msg->linear.x;
  purposevel.angular_z=msg->angular.z;
  ROS_INFO("purpose linear vel is %f", purposevel.linear_x);
  ROS_INFO("purpose angular vel is %f", purposevel.angular_z);
  if(purposevel.angular_z == 0){  //stop
    if(purposevel.linear_x > 0 && purposevel.linear_x <= 50){ //go straight 50mm/s
      Accel_Controller(1, false, 50);
      Accel_Controller(2, true, 50);
    }
    else if(purposevel.linear_x > 50 && purposevel.linear_x <= 140){ //go straight 139mm/s
      Accel_Controller(1, false, 60);
      Accel_Controller(2, true, 60);
    }
    else if(purposevel.linear_x > 140 && purposevel.linear_x <= 230){ //go straight 222mm/s
      Accel_Controller(1, false, 80);
      Accel_Controller(2, true, 80);
    }
    else if(purposevel.linear_x > 230 && purposevel.linear_x <= 300){ //go straight 309mm/s
      Accel_Controller(1, false, 100);
      Accel_Controller(2, true, 100);
    }
    else if(purposevel.linear_x > 300 && purposevel.linear_x <= 400){ //go straight 400mm/s
      Accel_Controller(1, false, 120);
      Accel_Controller(2, true, 120);
    }
    else if(purposevel.linear_x == 0){
      Accel_Controller(1, false, 0);
      Accel_Controller(2, true, 0);
    }
  }
  else if(purposevel.angular_z > 0){ //turn right
      Accel_Controller(1,false,100);
      Accel_Controller(2,true, 60);
    }
  else if(purposevel.angular_z < 0){ //turn left
      Accel_Controller(1,false, 60);
      Accel_Controller(2,true, 100);
  }
}

void Motor_View()
{
  RPM_Calculator();
	carcul_packet();
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter2A, EncoderCounter1A);
	printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter2B, EncoderCounter1B);
	printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value2, RPM_Value1);
	printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
  printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
	printf("Acc  :%10.0d\n", acceleration);
	printf("\n");
	printf("linear vel1: %10.0f || linear vel2: %10.0f\n",linear_vel2, linear_vel1); 
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
    // Motor_Controller(1, false, 120);  //motor_left
    // Motor_Controller(2, true, 100);   //motor_right
    // Accel_Controller(1, false, 120);
    // Accel_Controller(2, true, 120);
    Motor_View();
    packet_msg.vw[0] = linear;
    packet_msg.vw[1] = angular;
    packet_msg.encod[0] = Motor1_Encoder_Sum();
    packet_msg.encod[1] = Motor2_Encoder_Sum();
    packet_msg.odo[0] = odom_l;
    packet_msg.odo[1] = odom_r;
    packet_pub.publish(packet_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  Motor_Controller(1, true, 0);
  Motor_Controller(2, false, 0);
  return 0;
}
