#ifndef INTERFACE_H
#define INTERFACE_H

#define NOISE_THRESHOLD 1E-6
#define RATE 100
#define TIME_MAX 120

// Qt includes
#include <QtGui>
#include <QWidget>
#include <QGridLayout>
#include <QLabel>
// gazebo includes
#include <gazebo.hh>
// general includes
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/thread/mutex.hpp>
// local includes
#include "vectorGUI/qcustomplot.h"
#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"

using namespace gazebo;

class Interface : public QWidget
{
  Q_OBJECT

protected slots:
  void SpawnModel();
  void UpdatePlot();

public:
  Interface(std::string _path);
  ~Interface();
  void setPosition(math::Vector3 position);
  void setObjectContact(std::string name);
  void setForce(math::Vector3 force);
  void Spawn(std::string model, math::Pose pose);
  void Update(ConstContactsPtr &_msg);
  std::vector<std::string> models;

private:
  // auxiliar
  static std::string d2s(double d);
  int counter;
  boost::mutex mutex;
  // gazebo
  std::string topicPath;
  std::string factoryPath;
  transport::PublisherPtr pub;
  transport::SubscriberPtr subs;
  // labels, buttons and menus
  std::vector<QLabel*> contactLabels;
  std::vector<QLabel*> contactData;
  QComboBox* dropMenu;
  std::vector<QLineEdit*> entries;
  QPushButton* okButton;
  // graphics
  QCustomPlot* plot;
  QTimer* dataTimer;
  QVector<double> timeAxis;
  QVector<double> forceAxis, filterAxis;
  double forceMax;
  // filter
  double fc;
  Dsp::Filter* filter;

};

#endif
