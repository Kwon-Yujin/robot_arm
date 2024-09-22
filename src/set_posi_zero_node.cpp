#include "ros/ros.h"
#include "std_msgs/String.h"
#include "v3_rmd_can/cancan.h"

CAN _can("can0", "ttyACM1");

void rmd_initialize(void);
void rmd_set_zero(void);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_posi_zero_node");
  ros::NodeHandle nh;

  rmd_initialize();
  //
  //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  //
  //ros::Rate loop_rate(10);
  //while (ros::ok())
  //{
  //  std_msgs::String msg;
  //  msg.data = "hello world";
  //
  //  chatter_pub.publish(msg);
  //
  //  ros::spinOnce();
  //
  //  loop_rate.sleep();
  //}

  for(int i=0;i<10;i++){
    rmd_set_zero();
    usleep(3000);
  }

  _can.close_port();
  return 0;
}

void rmd_initialize(void){
  _can.CAN_initialize(_1M);
  _can.Set_Joint_ZeroPos();
}
void rmd_set_zero(void){
  _can.Set_Joint_ZeroPos();

}
