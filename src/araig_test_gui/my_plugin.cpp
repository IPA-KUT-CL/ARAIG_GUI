/*
  Copyright 2016 Lucas Walter
*/

#include "araig_test_gui/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <ros/ros.h>
#include <ros/master.h>
#include "nodelet/nodelet.h"
#include "std_msgs/Bool.h"

namespace araig_test_gui
{

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file

  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
  connect(ui_.pbTestStart, SIGNAL(clicked()), this, SLOT(on_pbTestStart_clicked()));
  connect(ui_.pbTestStop, SIGNAL(clicked()), this, SLOT(on_pbTestStop_clicked()));
  connect(ui_.pbTestReset, SIGNAL(clicked()), this, SLOT(on_pbTestReset_clicked()));
  connect(ui_.pbTestSucc, SIGNAL(clicked()), this, SLOT(on_pbTestSucc_clicked()));
  connect(ui_.pbTestFail, SIGNAL(clicked()), this, SLOT(on_pbTestFail_clicked()));

  nh = getNodeHandle();
  _NAME_INPUTS_ = {"start_test", "stop_test", "reset_test", "test_successful"};
  _NAME_OUTPUTS_ = {"test_running", "test_successful"};
  input_pubs_.resize(_NUM_INPUTS_);
  input_states.resize(_NUM_INPUTS_);
  output_subs_.resize(_NUM_OUTPUTS_);
  output_states.resize(_NUM_OUTPUTS_);
  spawnPubs();
  spawnSubs();
  testReset();
  ROS_INFO_STREAM("ARAIG_TEST_GUI initialized!");

}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

void MyPlugin::spawnPubs()
{
  for(int i=0; i<_NUM_INPUTS_; i++)
  {
    std::string topicName = _NAME_INPUTS_[i];
    input_pubs_[i] = nh.advertise<std_msgs::Bool>(topicName, 10);
    ROS_INFO_STREAM("Spawned input topic publisher " << i << " for : " << topicName);
  }
}

void MyPlugin::spawnSubs()
{
  for(int i=0; i<_NUM_OUTPUTS_; i++)
  {
    std::string topicName = _NAME_OUTPUTS_[i];
    output_subs_[i] = nh.subscribe<std_msgs::Bool>(topicName, 10, boost::bind(&MyPlugin::callbackBool, this, _1, topicName));
    ROS_INFO_STREAM("Spawned output topic subscriber " << i << " for : " << topicName);
  }
}

int MyPlugin::getIndexInVector(std::vector<std::string> vec, std::string topicName)
{
  auto it = find(vec.begin(), vec.end(), topicName);
  if(it != vec.end()){

    int idx = it - vec.begin();
    return idx;
  }
  else {
    return -1;
  }
}

void MyPlugin::callbackBool(const std_msgs::BoolConstPtr& msg, const std::string &topicName)
{
  ROS_INFO_STREAM("get data on topic" + topicName);
  int idx = getIndexInVector(_NAME_OUTPUTS_, topicName);
  if(idx !=-1){
    output_states[idx] = msg->data? true:false;
    outputTestState();
  }

}

void MyPlugin::outputTestState()
{
  if(!init_flag)
  {
    ui_.lbTestState->setText("Test ready!");
    ui_.lbTestResult->setText("Wait for start!");
  }
  else {
    if(output_states[0])
    {
      ui_.lbTestState->setText("Test running!");
    }
    else{
      ui_.lbTestState->setText("Test stopped!");
    }
    if(output_states[1])
    {
      ui_.lbTestResult->setText("Test successful!");
    }
    else{
      ui_.lbTestResult->setText("Test failed!");
    }
  }
}

void MyPlugin::pubPublish(int idx)
{
  std_msgs::Bool msg;
  msg.data = input_states[idx]?true:false;
  input_pubs_[idx].publish(msg);
  ros::spinOnce();
}

void MyPlugin::testReset()
{
  init_flag = 0;
  outputTestState();
  for (auto it = output_states.begin();it!=output_states.end();it++) {
    *it = false;
  }
  init_flag = 1;
}

}  // namespace araig_test_gui
PLUGINLIB_EXPORT_CLASS(araig_test_gui::MyPlugin, rqt_gui_cpp::Plugin)

void araig_test_gui::MyPlugin::on_pbTestStart_clicked()
{
  input_states[0] = true;
  pubPublish(0);
  ROS_INFO_STREAM("test started!");
}

void araig_test_gui::MyPlugin::on_pbTestStop_clicked()
{
  input_states[1] = true;
  pubPublish(1);
  ROS_INFO_STREAM("test stopped!");
}

void araig_test_gui::MyPlugin::on_pbTestReset_clicked()
{
  testReset();
  input_states[2] = true;
  pubPublish(2);
  ROS_INFO_STREAM("test reseted!");
}

void araig_test_gui::MyPlugin::on_pbTestSucc_clicked()
{
  input_states[3] = true;
  pubPublish(3);
  ROS_INFO_STREAM("test succeeded!");
}

void araig_test_gui::MyPlugin::on_pbTestFail_clicked()
{
  input_states[3] = false;
  pubPublish(3);
  ROS_INFO_STREAM("test failed!");
}