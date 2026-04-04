#ifndef INTERFACE_H
#define INTERFACE_H

#define RATE 25
#define TIME_MAX 120

// Qt 5 includes
#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QFrame>
#include <QTimer>
// ignition math (Gazebo 9+)
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
// gazebo includes
#include <gazebo.hh>
// standard includes
#include <memory>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <boost/thread/mutex.hpp>
// local includes
#include "common/Constants.h"
#include "vectorGUI/qcustomplot.h"
#include "DspFilters/Dsp.h"
#include "DspFilters/Filter.h"
#include "DspFilters/ForceFilter.h"

class Interface : public QWidget
{
  Q_OBJECT

protected slots:
  void SpawnModel();
  void UpdatePlot();

public:
  /**
   * @brief Constructs the VectorGUI window and connects to the Gazebo transport.
   * @param _path       Full Gazebo contact-sensor topic path.
   * @param _robotName  Name of the robot model used to identify body_1 in wrench
   *                    messages (default: @c "iCub").
   */
  Interface(std::string _path, std::string _robotName = "iCub");
  /** @brief Destructor; shuts down the Gazebo transport. */
  ~Interface();

  /** @brief Updates the position labels with @p pos. */
  void setPosition(ignition::math::Vector3d pos);
  /** @brief Updates the contact-object label with @p name. */
  void setObjectContact(std::string name);
  /** @brief Updates the force labels with @p force. */
  void setForce(ignition::math::Vector3d force);

  /**
   * @brief Spawns a model into the running Gazebo simulation.
   * @param model SDF model URI (e.g. @c "model://sphere").
   * @param pose  Target pose.
   */
  void Spawn(std::string model, ignition::math::Pose3d pose);

  /** @brief Gazebo transport callback for incoming contact messages. */
  void Update(gazebo::ConstContactsPtr& _msg);

  std::vector<std::string> models;

private:
  static std::string d2s(double d);
  int counter;
  boost::mutex mutex;
  // state tracking for plot range updates
  double lastTime;
  double lastForce;
  // gazebo
  std::string topicPath;
  std::string factoryPath;
  std::string robotName;
  gazebo::transport::PublisherPtr  pub;
  gazebo::transport::SubscriberPtr subs;
  // labels, buttons and menus
  std::vector<QLabel*>    contactLabels;
  std::vector<QLabel*>    contactData;
  QComboBox*              dropMenu;
  std::vector<QLineEdit*> entries;
  QPushButton*            okButton;
  // graphics
  QCustomPlot* plot;
  QTimer*      dataTimer;
  double       forceMax;
  std::unique_ptr<Dsp::ForceFilter> filter;
};

#endif
