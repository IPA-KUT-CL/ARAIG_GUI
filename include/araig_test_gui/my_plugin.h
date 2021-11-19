/*
  Copyright 2016 Lucas Walter
*/
#ifndef ARAIG_TEST_GUI_MY_PLUGIN_H
#define ARAIG_TEST_GUI_MY_PLUGIN_H

#include <ros/macros.h>

#include <rqt_gui_cpp/plugin.h>
#include <araig_test_gui/ui_my_plugin.h>
#include <QWidget>
#include <QStringList>
#include <QSet>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nodelet/nodelet.h"
#include <boost/bind.hpp>

namespace araig_test_gui
{

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  // enable meta-object's support
  Q_OBJECT
public:
  // declare constructor and all slots functions(why with virtual)
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

protected:
  ros::NodeHandle nh;
  std::vector<ros::Publisher> input_pubs_;
  std::vector<ros::Subscriber> output_subs_;
private:
  bool test_ready = false;
  const int _NUM_INPUTS_ = 4;
  const int _NUM_OUTPUTS_ = 2;
  std::vector<std::string> _NAME_INPUTS_;
  std::vector<std::string> _NAME_OUTPUTS_;
  std::vector<bool> input_states;
  std::vector<bool> output_states;

public:
  void callbackBool(const std_msgs::BoolConstPtr& msg, const std::string &topicName);
  void outputTestState();
  void spawnPubs();
  void spawnSubs();
  int getIndexInVector(std::vector<std::string> vec, std::string topicName);
  void pubPublish(int idx);
  void stateInit();


private slots:
  void on_pbTestStart_clicked();

  void on_pbTestStop_clicked();

  void on_pbTestReset_clicked();

  void on_pbTestSucc_clicked();

  void on_pbTestFail_clicked();

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
};
}  // namespace araig_test_gui
#endif  // ARAIG_TEST_GUI_MY_PLUGIN_H
