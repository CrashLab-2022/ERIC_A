#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

class Control {
public:
    Control();

private:
    bool pushitem_service(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    ros::NodeHandle n;

    ros::ServiceServer service_;
    ros::Publisher vel_pub_;
};

Control::Control() {
    service_ = n.advertiseService("/exection", &Control::pushitem_service, this);
    vel_pub_ = n.advertise<geometry_msgs::Twist>("/dynamixel/cmd_vel", 10);
}

bool Control::pushitem_service(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ROS_INFO("Me");
    ros::Rate loop_rate(10);

    int count = 0;   
    while (count < 210) {     
        geometry_msgs::Twist twist;
        
        if (count < 100) {
            twist.linear.x = 0.1;     
        }
        else if (count < 110) {
            twist.linear.x = 0;
        }
        else {
            twist.linear.x = -0.1;
        }
        
        vel_pub_.publish(twist);
        ros::spinOnce();
            
        loop_rate.sleep();
        ++count;
    }

  return res.success = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_control");
  Control dynamixel_cnt;
  ROS_INFO("Ready to dynamixel_cmd_vel pub");
  ros::spin();

  return 0;
}