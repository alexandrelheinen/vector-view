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

public:
  Interface(QWidget *parent = 0);
  void setPosition(math::Vector3 position);
  void setObjectContact(std::string name);
  void setForce(math::Vector3 force);
  void Update(ConstContactsPtr &_msg);

private:
  QGridLayout *layout;
  std::vector<QLabel*> contactLabels;
  std::vector<QLabel*> contactData;
  static std::string d2s(double d);
};

#endif
