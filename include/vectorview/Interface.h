#ifndef INTERFACE_H
#define INTERFACE_H

#include "vectorview/Constants.h"
#include "vectorview/ForceFilter.h"

#include <QtGui>
#include <QGridLayout>
#include <QLabel>
#include <QWidget>

#include <gazebo.hh>

#include <boost/thread/mutex.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "qcustomplot.h"

class Interface : public QWidget {
  Q_OBJECT

 protected slots:
  void SpawnModel();
  void UpdatePlot();

 public:
  Interface(std::string _path, const std::string& robot_name = "iCub");
  ~Interface();
  void setPosition(gazebo::math::Vector3 position);
  void setObjectContact(std::string name);
  void setForce(gazebo::math::Vector3 force);
  void Spawn(std::string model, gazebo::math::Pose pose);
  void Update(gazebo::ConstContactsPtr& _msg);
  std::vector<std::string> models;

 private:
  static std::string d2s(double d);
  int counter;
  boost::mutex mutex;
  std::string topicPath;
  std::string factoryPath;
  std::string robotName;
  gazebo::transport::PublisherPtr pub;
  gazebo::transport::SubscriberPtr subs;
  std::vector<QLabel*> contactLabels;
  std::vector<QLabel*> contactData;
  QComboBox* dropMenu;
  std::vector<QLineEdit*> entries;
  QPushButton* okButton;
  QCustomPlot* plot;
  QTimer* dataTimer;
  QVector<double> timeAxis;
  QVector<double> forceAxis, filterAxis;
  double forceMax;
  std::unique_ptr<vectorview::ForceFilter> filter;
};

#endif
