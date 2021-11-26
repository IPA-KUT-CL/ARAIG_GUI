/*
  Copyright 2021 Changxuan Li
*/
#ifndef ARAIG_TEST_GUI_PLUGIN_H
#define ARAIG_TEST_GUI_PLUGIN_H

#include <ros/macros.h>

#include <araig_test_gui/plugin.h>
#include <araig_test_gui/ui_plugin.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <QStringList>
#include <QSet>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "nodelet/nodelet.h"
#include <boost/bind.hpp>

namespace araig_test_gui
{

class Plugin
  : public rqt_gui_cpp::Plugin
{
  // enable meta-object's support
  Q_OBJECT
public:
  // declare constructor and all slots functions(why with virtual)
  Plugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

protected:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Publisher pub_test;
  ros::Subscriber sub;
  ros::Subscriber sub1;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Subscriber sub4;
  std::string topicHeader;
  std::string sub_topicName;
public:
  template <class dataType>
  void publishMsg(const std::string& topicname, uint32_t queueSize, dataType data);
  void subBool(ros::Subscriber &sub, const std::string& topicName, uint32_t queueSize);
  void callbackBool(const std_msgs::BoolConstPtr& msg, const std::string &topicName);
  void callbackReact(const std::string& topicName);
  void rbReset();
  void changeTopicHeader(const std::string &newTopicHeader);
  void resetTopicHeader();

protected slots:
  virtual void pbTestStart();
  virtual void pbTestStop();
  virtual void pbTestReset();
  //virtual void pbTestSucc();
  //virtual void pbTestFail();

private slots:
  void on_pbTestStart_clicked();

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
};
}  // namespace araig_test_gui
#endif  // ARAIG_TEST_GUI_PLUGIN_H
