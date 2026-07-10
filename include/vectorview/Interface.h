#ifndef INTERFACE_H
#define INTERFACE_H

#include "vectorview/Constants.h"
#include "vectorview/ContactUtils.h"
#include "vectorview/ForceFilter.h"

#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include <gz/msgs/contacts.pb.h>
#include <gz/transport/Node.hh>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "qcustomplot.h"

class Interface : public QWidget {
  Q_OBJECT

 protected slots:
  void SpawnModel();
  void UpdatePlot();

 public:
  Interface(const std::string& path, const std::string& robot_name = "iCub",
            const std::string& world_name = "default");
  ~Interface();
  void setPosition(const vectorview::Vec3& position);
  void setObjectContact(const std::string& name);
  void setForce(const vectorview::Vec3& force);
  void Spawn(const std::string& model, const vectorview::Vec3& position);
  void Update(const gz::msgs::Contacts& message);
  std::vector<std::string> models;

 private:
  static std::string d2s(double d);
  int counter;
  std::mutex mutex;
  std::string topicPath;
  std::string worldName;
  std::string robotName;
  gz::transport::Node node;
  std::vector<QLabel*> contactLabels;
  std::vector<QLabel*> contactData;
  QComboBox* dropMenu;
  std::vector<QLineEdit*> entries;
  QPushButton* okButton;
  QCustomPlot* plot;
  QTimer* dataTimer;
  QVector<double> timeAxis;
  QVector<double> forceAxis;
  QVector<double> filterAxis;
  double forceMax;
  std::unique_ptr<vectorview::ForceFilter> filter;
};

#endif
