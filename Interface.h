#ifndef INTERFACE_H
#define INTERFACE_H

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

using namespace gazebo;

class Interface : public QWidget
{
  Q_OBJECT

protected slots:
  void SpawnModel();

public:
  Interface(std::string _path);
  ~Interface();
  void setPosition(math::Vector3 position);
  void setObjectContact(std::string name);
  void setForce(math::Vector3 force);
  void Update(ConstContactsPtr &_msg);
  void Spawn(std::string model, math::Pose pose);
  std::vector<std::string> models;

private:
  // frame and layouts
  QGridLayout *mainLayout, *contactLayout, *buttonLayout;
  QFrame *mainFrame;
  // labels and buttons
  std::vector<QLabel*> contactLabels;
  std::vector<QLabel*> contactData;
  QPushButton* okButton;
  QComboBox* dropMenu;
  std::vector<QLineEdit*> entries;
  // auxiliar
  static std::string d2s(double d);
  int counter;
  // gazebo
  std::string topicPath;
  std::string factoryPath;
  transport::PublisherPtr pub;
  transport::SubscriberPtr subs;
};

#endif
