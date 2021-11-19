#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <string>
#include <boost/bind.hpp>
#include <time.h>

void callbackTestState(const std_msgs::Bool::ConstPtr &msg, const std::string topicName,
                       std_msgs::Bool &pub_msg1, std_msgs::Bool &pub_msg2, ros::Publisher pub1, ros::Publisher pub2)
{
  bool start=false, stop=false, reset=false;
  if(topicName == "start"){
    ROS_INFO_STREAM("test_node: received start msg");
    start = msg->data;
    pub_msg1.data = start?true:false;
  }
  else if (topicName == "stop") {
    stop = msg->data;
    pub_msg1.data = stop?false:true;
  }
  else {
    reset = msg->data;
    pub_msg1.data = reset?false:true;
  }

  pub1.publish(pub_msg1);

  if(pub_msg1.data == true){
    ROS_INFO_STREAM("test_node: test started!");
    ros::Duration(1).sleep();
    srand((unsigned)time(NULL));
    if((float)rand()/RAND_MAX >= 0.5){
      ROS_INFO_STREAM("test_node: test succeeded!");
      pub_msg2.data = true;
    }
    else {
      ROS_INFO_STREAM("test_node: test failed!");
      pub_msg2.data = false;
    }
    pub_msg1.data = false;
  }
  else {
    pub_msg2.data = false;
  }
  pub1.publish(pub_msg1);
  pub2.publish(pub_msg2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  ROS_INFO("test_node: Test node running!");

  ros::Publisher pub1 = nh.advertise<std_msgs::Bool>("/araig_test_gui/test_running", 10);
  ros::Publisher pub2 = nh.advertise<std_msgs::Bool>("/araig_test_gui/test_successful", 10);

  std_msgs::Bool msg1;
  std_msgs::Bool msg2;

  msg1.data = false;
  msg2.data = false;

  ros::Subscriber sub1 = nh.subscribe<std_msgs::Bool>
      ("/araig_test_gui/start_test", 10, boost::bind(callbackTestState, _1, "start", msg1, msg2, pub1, pub2));
  ros::Subscriber sub2 = nh.subscribe<std_msgs::Bool>
      ("/araig_test_gui/stop_test", 10, boost::bind(callbackTestState, _1, "stop", msg1, msg2, pub1, pub2));
  ros::Subscriber sub3 = nh.subscribe<std_msgs::Bool>
      ("/araig_test_gui/reset_test", 10, boost::bind(callbackTestState, _1, "reset", msg1, msg2, pub1, pub2));

  pub1.publish(msg1);

  ros::spin();


  return 0;


}
