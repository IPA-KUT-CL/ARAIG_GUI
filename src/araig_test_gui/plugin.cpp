/*
  Copyright 2021 Changxuan Li
*/

#include "araig_test_gui/plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <ros/ros.h>
#include <ros/master.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <string.h>


namespace araig_test_gui
{

Plugin::Plugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("Plugin");
}

void Plugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
  //int argc;
  //char** ros_argv;
  //ros::init(argc, ros_argv,"gui");
  nh = getNodeHandle();

  ROS_INFO_STREAM("ARAIG_TEST_GUI initialized!");

  pub_test = nh.advertise<std_msgs::Bool>("/topic1",10);
  std_msgs::Bool msg;
  msg.data = true;
  pub_test.publish(msg);

  subBool(sub, "start_robot", 10);
  subBool(sub1, "stop_robot", 10);
  subBool(sub2, "test_completed", 10);
  subBool(sub3, "test_succeeded", 10);
  subBool(sub4, "test_failed", 10);
  
  connect(ui_.pbTestStart, SIGNAL(pressed()), this, SLOT(pbTestStart()));
  connect(ui_.pbTestStop, SIGNAL(pressed()), this, SLOT(pbTestStop()));
  connect(ui_.pbTestReset, SIGNAL(pressed()), this, SLOT(pbTestReset()));
  //connect(ui_.pbTestSucc, SIGNAL(pressed()), this, SLOT(pbTestSucc()));
  //connect(ui_.pbTestFail, SIGNAL(pressed()), this, SLOT(pbTestFail()));
}

void Plugin::shutdownPlugin()
{
  // unregister all publishers here
}

void Plugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void Plugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

template <class dataType>
void Plugin::publishMsg(const std::string& topicname, uint32_t queueSize, dataType data)
{
  pub = nh.advertise<dataType>(topicname, queueSize);
  pub.publish(data);
}

void Plugin::subBool(ros::Subscriber &sub, const std::string& topicName, uint32_t queueSize)
{
  sub_topicName = topicName;
  sub = nh.subscribe<std_msgs::Bool>(topicName, queueSize, boost::bind(&Plugin::callbackBool, this, _1, topicName));
}

void Plugin::callbackBool(const std_msgs::BoolConstPtr& msg, const std::string &topicName)
{
  if(msg->data == true)
  {
    ROS_INFO_STREAM("Received message true from topic: " << topicName);
    callbackReact(topicName);
  }
}

void Plugin::callbackReact(const std::string& topicName)
{
  if(topicName == "start_robot"){
    ui_.rbRobotRunning->toggle();
  }
  if(topicName == "stop_robot"){
    ui_.rbRobotStopped->toggle();
  }
  if(topicName == "test_completed"){
    ui_.rbTestComplete->toggle();
  }
  if(topicName == "test_succeeded"){
    ui_.rbTestSucceeded->toggle();
  }
  if(topicName == "test_failed"){
    ui_.rbTestFailed->setChecked(true);
  }
}

void Plugin::pbTestStart()
{
  ui_.rbRobotRunning->toggle();
  const std::string topicName = "start";
  uint32_t queueSize = 10;
  std_msgs::Bool msg;
  msg.data = true;
  publishMsg<std_msgs::Bool>(topicName, queueSize, msg);
}

void Plugin::pbTestStop()
{
  ui_.rbRobotStopped->toggle();
  const std::string topicName = "stop";
  uint32_t queueSize = 10;
  std_msgs::Bool msg;
  msg.data = true;
  publishMsg<std_msgs::Bool>(topicName, queueSize, msg);
}

void Plugin::pbTestReset()
{
  rbReset();
  const std::string topicName = "reset";
  uint32_t queueSize = 10;
  std_msgs::Bool msg;
  msg.data = true;
  publishMsg<std_msgs::Bool>(topicName, queueSize, msg);
}

// void Plugin::pbTestSucc()
// {
//   const std::string topicName = "test_succeeded";
//   uint32_t queueSize = 10;
//   std_msgs::Bool msg;
//   msg.data = true;
//   publishMsg<std_msgs::Bool>(topicName, queueSize, msg);
// }

// void Plugin::pbTestFail()
// {
//   const std::string topicName = "test_failed";
//   uint32_t queueSize = 10;
//   std_msgs::Bool msg;
//   msg.data = true;
//   publishMsg<std_msgs::Bool>(topicName, queueSize, msg);
// }

void Plugin::rbReset()
{
  ui_.rbRobotRunning->setAutoExclusive(false);
  ui_.rbRobotStopped->setAutoExclusive(false);

  ui_.rbRobotRunning->setChecked(false);  
  ui_.rbRobotStopped->setChecked(false);

  ui_.rbRobotRunning->setAutoExclusive(true);
  ui_.rbRobotStopped->setAutoExclusive(true);

  ui_.rbTestOn->setAutoExclusive(false);
  ui_.rbTestComplete->setAutoExclusive(false);

  ui_.rbTestOn->setChecked(false);  
  ui_.rbTestComplete->setChecked(false);

  ui_.rbTestOn->setAutoExclusive(true);
  ui_.rbTestComplete->setAutoExclusive(true);

  ui_.rbTestSucceeded->setAutoExclusive(false);
  ui_.rbTestFailed->setAutoExclusive(false);

  ui_.rbTestSucceeded->setChecked(false);  
  ui_.rbTestFailed->setChecked(false);

  ui_.rbTestSucceeded->setAutoExclusive(true);
  ui_.rbTestFailed->setAutoExclusive(true);
}

void Plugin::changeTopicHeader(const std::string &newTopicHeader)
{
  topicHeader = newTopicHeader;
}
void Plugin::resetTopicHeader()
{
  topicHeader = "/signal/ui/";
}
  // restore previous selection
  //selectTopic(selected);


// /*bool hasConfiguration() const
// {
//   return true;
// }

// void triggerConfiguration()
// {
//   // Usually used to open a dialog to offer the user a set of configuration
// }*/

}  // namespace rqt_example_cpp
PLUGINLIB_EXPORT_CLASS(araig_test_gui::Plugin, rqt_gui_cpp::Plugin)


