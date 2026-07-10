#include "vectorview/Interface.h"

#include "vectorview/ContactMessage.h"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose.pb.h>

#include <iostream>
#include <sstream>

Interface::Interface(const std::string& path, const std::string& robot_name,
                     const std::string& world_name)
    : QWidget(nullptr) {
  this->topicPath = path;
  this->worldName = world_name;
  this->robotName = robot_name;
  this->counter = 0;

  this->models.push_back("sphere");
  this->models.push_back("cylinder");
  this->models.push_back("box");

  QGridLayout* mainLayout = new QGridLayout(this);
  QGridLayout* topicLayout = new QGridLayout();
  QGridLayout* infoLayout = new QGridLayout();
  QGridLayout* contactLayout = new QGridLayout();
  QGridLayout* buttonLayout = new QGridLayout();
  QGridLayout* entriesLayout = new QGridLayout();
  QGridLayout* plotLayout = new QGridLayout();

  QFrame* infoFrame = new QFrame();
  QGroupBox* topicFrame = new QGroupBox(tr("Current topic path"));
  QGroupBox* contactFrame = new QGroupBox(tr("Contact info"));
  QGroupBox* buttonFrame = new QGroupBox(tr("Spawn models"));
  QGroupBox* settingsFrame = new QGroupBox(tr("Settings"));
  QFrame* entriesFrame = new QFrame();
  QGroupBox* plotFrame = new QGroupBox(tr("Magnitude plot"));

  infoFrame->setLayout(infoLayout);
  topicFrame->setLayout(topicLayout);
  contactFrame->setLayout(contactLayout);
  buttonFrame->setLayout(buttonLayout);
  settingsFrame->setLayout(new QGridLayout());
  entriesFrame->setLayout(entriesLayout);
  plotFrame->setLayout(plotLayout);
  entriesFrame->setMaximumWidth(100);
  entriesFrame->setMinimumWidth(100);
  buttonFrame->setMaximumWidth(120);
  buttonFrame->setMinimumWidth(120);

  mainLayout->addWidget(topicFrame, 0, 0);
  mainLayout->addWidget(settingsFrame, 1, 0);
  mainLayout->addWidget(infoFrame, 2, 0);
  mainLayout->addWidget(plotFrame, 3, 0);
  infoLayout->addWidget(contactFrame, 0, 0);
  infoLayout->addWidget(buttonFrame, 0, 1);

  contactLabels.push_back(new QLabel("Object:", this));
  contactData.push_back(new QLabel("----", this));
  contactLabels.back()->setAlignment(Qt::AlignRight);
  contactData.push_back(nullptr);
  contactData.push_back(nullptr);
  contactLabels.push_back(new QLabel("Position:", this));
  contactData.push_back(new QLabel("0.0000", this));
  contactLabels.back()->setAlignment(Qt::AlignRight);
  contactData.push_back(new QLabel("0.0000", this));
  contactData.push_back(new QLabel("0.0000", this));
  contactLabels.push_back(new QLabel("Force:", this));
  contactData.push_back(new QLabel("0.0000", this));
  contactLabels.back()->setAlignment(Qt::AlignRight);
  contactData.push_back(new QLabel("0.0000", this));
  contactData.push_back(new QLabel("0.0000", this));

  contactLayout->setSizeConstraint(QLayout::SetFixedSize);
  for (unsigned int k = 0; k < contactLabels.size(); ++k) {
    contactLayout->addWidget(contactLabels.at(k), 3 * k, 0);
  }

  for (unsigned int k = 0; k < contactData.size(); ++k) {
    if (contactData.at(k) != nullptr) {
      contactLayout->addWidget(contactData.at(k), k, 1);
      contactData.at(k)->setMaximumWidth(150);
      contactData.at(k)->setMinimumWidth(150);
    }
  }

  dropMenu = new QComboBox;
  dropMenu->setMaximumWidth(100);
  dropMenu->setMinimumWidth(100);
  for (unsigned int k = 0; k < models.size(); ++k) {
    dropMenu->addItem(models.at(k).c_str());
  }

  okButton = new QPushButton("Spawn");
  connect(okButton, &QPushButton::clicked, this, &Interface::SpawnModel);
  okButton->setMaximumWidth(100);
  okButton->setMinimumWidth(100);

  const std::vector<std::string> flags = {"x:", "y:", "z:"};
  for (unsigned int k = 0; k < flags.size(); ++k) {
    QLabel* title = new QLabel(flags.at(k).c_str());
    title->setAlignment(Qt::AlignRight);
    title->setMaximumWidth(30);
    title->setMinimumWidth(30);
    entries.push_back(new QLineEdit("1.00"));
    entries.back()->setMaximumWidth(50);
    entries.back()->setMinimumWidth(50);
    entriesLayout->addWidget(title, k, 0);
    entriesLayout->addWidget(entries.back(), k, 1);
  }

  buttonLayout->addWidget(dropMenu);
  buttonLayout->addWidget(entriesFrame);
  buttonLayout->addWidget(okButton);

  timeWindow = TIME_MAX;
  updateRate = RATE;
  noiseThreshold = NOISE_THRESHOLD;
  filterOrder = 3;
  filterCutoff = 1.5;
  forceMax = 10;

  QGridLayout* settingsLayout = qobject_cast<QGridLayout*>(settingsFrame->layout());
  const std::vector<std::pair<std::string, std::string>> settingFields = {
      {"time window (s):", std::to_string(static_cast<int>(timeWindow))},
      {"update rate (Hz):", std::to_string(static_cast<int>(updateRate))},
      {"noise threshold (N):", "0.001"},
      {"force axis max (N):", std::to_string(static_cast<int>(forceMax))},
      {"filter order:", std::to_string(filterOrder)},
      {"filter cutoff (Hz):", "1.5"}};

  QDoubleValidator* positiveValidator = new QDoubleValidator(0.0, 1e9, 6, this);
  QDoubleValidator* positiveIntValidator = new QDoubleValidator(1.0, 1e9, 0, this);

  for (unsigned int k = 0; k < settingFields.size(); ++k) {
    QLabel* title = new QLabel(settingFields.at(k).first.c_str());
    title->setAlignment(Qt::AlignRight);
    QLineEdit* field = new QLineEdit(settingFields.at(k).second.c_str());
    if (k == 4) {
      field->setValidator(positiveIntValidator);
    } else {
      field->setValidator(positiveValidator);
    }
    settingsEntries.push_back(field);
    settingsLayout->addWidget(title, static_cast<int>(k), 0);
    settingsLayout->addWidget(field, static_cast<int>(k), 1);
  }

  QPushButton* applyButton = new QPushButton("Apply");
  connect(applyButton, &QPushButton::clicked, this, &Interface::ApplySettings);
  settingsLayout->addWidget(applyButton, static_cast<int>(settingFields.size()), 0, 1, 2);

  plot = new QCustomPlot();
  plotLayout->addWidget(plot);
  plot->addGraph();
  plot->addGraph();
  connect(plot->xAxis, SIGNAL(rangeChanged(QCPRange)), plot->xAxis2, SLOT(setRange(QCPRange)));
  connect(plot->yAxis, SIGNAL(rangeChanged(QCPRange)), plot->yAxis2, SLOT(setRange(QCPRange)));
  plot->xAxis->setLabel("time (s)");
  plot->yAxis->setLabel("force (N)");
  plot->xAxis->setRange(0, timeWindow);
  plot->yAxis->setRange(0, forceMax);
  plot->graph(0)->setPen(QPen(Qt::gray));
  plot->graph(1)->setPen(QPen(Qt::darkBlue));
  plot->graph(0)->setName("original");
  plot->graph(1)->setName("filtered");
  plot->setBackground(QColor(242, 241, 240, 127));
  plot->legend->setVisible(true);
  plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);
  plot->setMaximumWidth(420);
  plot->setMinimumWidth(420);
  plot->setMaximumHeight(300);
  plot->setMinimumHeight(300);

  dataTimer = new QTimer;
  connect(dataTimer, &QTimer::timeout, this, &Interface::UpdatePlot);
  dataTimer->start(static_cast<int>(1000.0 / updateRate));

  filter = std::make_unique<vectorview::ForceFilter>(updateRate, filterOrder, filterCutoff);

  this->node.Subscribe(this->topicPath, &Interface::Update, this);

  topicLayout->addWidget(new QLabel(("spawn: /world/" + world_name + "/create").c_str()), 0, 0);
  topicLayout->addWidget(new QLabel(this->topicPath.c_str()), 1, 0);

  this->setWindowTitle(("Vector GUI [" + this->topicPath + "]").c_str());
  this->move(20, 20);
  this->setMaximumWidth(480);
  this->setMinimumWidth(480);
  this->setMaximumHeight(860);
  this->setMinimumHeight(860);
}

Interface::~Interface() {
  std::cout << "Vector GUI interface [" << this->topicPath << "] ended" << std::endl;
}

void Interface::setObjectContact(const std::string& name) {
  contactData.at(0)->setText(QString::fromStdString(name));
}

void Interface::setPosition(const vectorview::Vec3& pos) {
  contactData.at(3)->setText(QString::fromStdString(d2s(pos.x)));
  contactData.at(4)->setText(QString::fromStdString(d2s(pos.y)));
  contactData.at(5)->setText(QString::fromStdString(d2s(pos.z)));
}

void Interface::setForce(const vectorview::Vec3& force) {
  contactData.at(6)->setText(QString::fromStdString(d2s(force.x)));
  contactData.at(7)->setText(QString::fromStdString(d2s(force.y)));
  contactData.at(8)->setText(QString::fromStdString(d2s(force.z)));
}

std::string Interface::d2s(double d) {
  std::ostringstream strs;
  strs << d;
  return strs.str();
}

void Interface::Update(const gz::msgs::Contacts& message) {
  std::lock_guard<std::mutex> lock(mutex);
  vectorview::Vec3 position;
  vectorview::Vec3 force;

  if (message.contact_size() > 0) {
    const gz::msgs::Contact& first_contact = message.contact(0);
    if (first_contact.position_size() > 0) {
      const gz::msgs::Vector3d& pos = first_contact.position(0);
      position = vectorview::Vec3(pos.x(), pos.y(), pos.z());
    }

    const std::vector<vectorview::ContactForce> contacts =
        vectorview::ContactsFromMessage(message);
    const vectorview::GuiContactResult aggregated =
        vectorview::AggregateGuiForces(contacts, this->robotName);
    force = aggregated.force;

    if (++counter > 10) {
      counter = 0;
      if (force.Length() > noiseThreshold) {
        this->setObjectContact(aggregated.object_name);
        this->setPosition(position);
        this->setForce(force);
      }
    }
  }

  vectorview::Vec3 plotForce(force.x, force.y, force.z);
  const gz::msgs::Time& stamp = message.header().stamp();
  this->timeAxis.push_back(stamp.sec() + 1e-9 * stamp.nsec());
  this->forceAxis.push_back(filter->Filter(&plotForce));
  this->filterAxis.push_back(force.Length());
}

void Interface::SpawnModel() {
  Spawn("model://" + dropMenu->currentText().toStdString(),
        vectorview::Vec3(entries.at(0)->text().toDouble(), entries.at(1)->text().toDouble(),
                         entries.at(2)->text().toDouble()));
}

void Interface::Spawn(const std::string& path, const vectorview::Vec3& position) {
  gz::msgs::EntityFactory request;
  request.set_sdf_filename(path);
  request.mutable_pose()->mutable_position()->set_x(position.x);
  request.mutable_pose()->mutable_position()->set_y(position.y);
  request.mutable_pose()->mutable_position()->set_z(position.z);
  request.set_allow_renaming(true);

  const std::string service = "/world/" + this->worldName + "/create";
  gz::msgs::Boolean response;
  bool result = false;
  const bool executed = this->node.Request(service, request, 2000, response, result);

  if (!executed || !result || !response.data()) {
    std::cerr << " >> failed to spawn " << path << " via " << service << std::endl;
    return;
  }

  std::cout << " >> " << path << " spawned at " << std::endl
            << "   pos: (" << position.x << ", " << position.y << ", " << position.z << ");"
            << std::endl;
}

void Interface::UpdatePlot() {
  std::lock_guard<std::mutex> lock(mutex);
  if (timeAxis.empty()) {
    return;
  }

  plot->graph(0)->addData(timeAxis, forceAxis);
  plot->graph(1)->addData(timeAxis, filterAxis);
  if (timeAxis.back() > timeWindow) {
    plot->xAxis->setRange(timeAxis.back() - timeWindow, timeAxis.back());
  }

  if (forceAxis.back() > forceMax) {
    forceMax = forceAxis.back();
    plot->yAxis->setRange(0, forceMax);
  }
  plot->replot();
  timeAxis.clear();
  forceAxis.clear();
  filterAxis.clear();
}

void Interface::ApplySettings() {
  timeWindow = settingsEntries.at(0)->text().toDouble();
  updateRate = settingsEntries.at(1)->text().toDouble();
  noiseThreshold = settingsEntries.at(2)->text().toDouble();
  forceMax = settingsEntries.at(3)->text().toDouble();
  filterOrder = static_cast<int>(settingsEntries.at(4)->text().toDouble());
  filterCutoff = settingsEntries.at(5)->text().toDouble();

  if (timeWindow <= 0 || updateRate <= 0 || forceMax <= 0 || filterOrder <= 0 ||
      filterCutoff <= 0) {
    std::cerr << " >> invalid settings: all values must be positive" << std::endl;
    return;
  }

  plot->xAxis->setRange(0, timeWindow);
  plot->yAxis->setRange(0, forceMax);
  dataTimer->setInterval(static_cast<int>(1000.0 / updateRate));
  filter->Configure(updateRate, filterOrder, filterCutoff);
  plot->replot();
}
