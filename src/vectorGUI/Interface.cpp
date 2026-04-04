#include "vectorGUI/Interface.h"

using namespace gazebo;

Interface::Interface(std::string _path, std::string _robotName) : QWidget(NULL)
{
  this->topicPath   = _path;
  this->factoryPath = "~/factory";
  this->robotName   = _robotName;
  counter   = 0;
  lastTime  = 0.0;
  lastForce = 0.0;

  // this is the model list
  models.push_back("sphere");
  models.push_back("cylinder");
  models.push_back("box");
  models.push_back("robot");
  models.push_back("table");

  // layouts and frames initialization
  QGridLayout *mainLayout    = new QGridLayout(this);
  QGridLayout *topicLayout   = new QGridLayout();
  QGridLayout *infoLayout    = new QGridLayout();
  QGridLayout *contactLayout = new QGridLayout();
  QGridLayout *buttonLayout  = new QGridLayout();
  QGridLayout *entriesLayout = new QGridLayout();
  QGridLayout *plotLayout    = new QGridLayout();

  QFrame    *infoFrame    = new QFrame();
  QGroupBox *topicFrame   = new QGroupBox(tr("Current topic path"));
  QGroupBox *contactFrame = new QGroupBox(tr("Contact info"));
  QGroupBox *buttonFrame  = new QGroupBox(tr("Spawn models"));
  QFrame    *entriesFrame = new QFrame();
  QGroupBox *plotFrame    = new QGroupBox(tr("Magnitude plot"));

  infoFrame->setLayout(infoLayout);
  topicFrame->setLayout(topicLayout);
  contactFrame->setLayout(contactLayout);
  buttonFrame->setLayout(buttonLayout);
  entriesFrame->setLayout(entriesLayout);
  plotFrame->setLayout(plotLayout);
  entriesFrame->setMaximumWidth(100);
  entriesFrame->setMinimumWidth(100);
  buttonFrame->setMaximumWidth(120);
  buttonFrame->setMinimumWidth(120);

  mainLayout->addWidget(topicFrame,   0, 0);
  mainLayout->addWidget(infoFrame,    1, 0);
  mainLayout->addWidget(plotFrame,    2, 0);
  infoLayout->addWidget(contactFrame, 0, 0);
  infoLayout->addWidget(buttonFrame,  0, 1);

  // labels initialization
  contactLabels.push_back(new QLabel("Object:",this));    contactData.push_back(new QLabel("----", this));
  contactLabels.back()->setAlignment(Qt::AlignRight);     contactData.push_back(NULL);
                                                          contactData.push_back(NULL);
  contactLabels.push_back(new QLabel("Position:",this));  contactData.push_back(new QLabel("0.0000", this));
  contactLabels.back()->setAlignment(Qt::AlignRight);     contactData.push_back(new QLabel("0.0000", this));
                                                          contactData.push_back(new QLabel("0.0000", this));
  contactLabels.push_back(new QLabel("Force:",this));     contactData.push_back(new QLabel("0.0000", this));
  contactLabels.back()->setAlignment(Qt::AlignRight);     contactData.push_back(new QLabel("0.0000", this));
                                                          contactData.push_back(new QLabel("0.0000", this));

  // input those labels on the layout
  contactLayout->setSizeConstraint(QLayout::SetFixedSize);
  unsigned int k;
  for (k = 0; k < contactLabels.size(); ++k)
    contactLayout->addWidget(contactLabels.at(k), 3*k, 0);

  for (k = 0; k < contactData.size(); ++k)
    if (contactData.at(k) != NULL)
    {
      contactLayout->addWidget(contactData.at(k), k, 1);
      contactData.at(k)->setMaximumWidth(150);
      contactData.at(k)->setMinimumWidth(150);
    }

  // spawn model section initialization
  dropMenu = new QComboBox;
  dropMenu->setMaximumWidth(100);
  dropMenu->setMinimumWidth(100);
  for (k = 0; k < models.size(); ++k)
    dropMenu->addItem(models.at(k).c_str());

  okButton = new QPushButton("Spawn");
  connect(okButton, SIGNAL(clicked()), this, SLOT(SpawnModel()));
  okButton->setMaximumWidth(100);
  okButton->setMinimumWidth(100);

  QLabel *title;
  std::vector<std::string> flags;
  flags.push_back("x:");
  flags.push_back("y:");
  flags.push_back("z:");

  for (k = 0; k < flags.size(); ++k)
  {
    title = new QLabel(flags.at(k).c_str());
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

  // plot startup
  forceMax = 10;
  plot = new QCustomPlot();
  plotLayout->addWidget(plot);
  plot->addGraph();  // original (unfiltered) data
  plot->addGraph();  // filtered data
  // assures that both graphics will keep the same axis ranges
  connect(plot->xAxis, SIGNAL(rangeChanged(QCPRange)), plot->xAxis2, SLOT(setRange(QCPRange)));
  connect(plot->yAxis, SIGNAL(rangeChanged(QCPRange)), plot->yAxis2, SLOT(setRange(QCPRange)));
  plot->xAxis->setLabel("time (s)");
  plot->yAxis->setLabel("force (N)");
  plot->xAxis->setRange(0, TIME_MAX);
  plot->yAxis->setRange(0, forceMax);
  plot->graph(0)->setPen(QPen(Qt::gray));
  plot->graph(1)->setPen(QPen(Qt::darkBlue));
  plot->graph(0)->setName("original");
  plot->graph(1)->setName("filtered");
  plot->setBackground(QColor(242, 241, 240, 127));
  plot->legend->setVisible(true);
  plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop);
  plot->setMaximumWidth(420);
  plot->setMinimumWidth(420);
  plot->setMaximumHeight(300);
  plot->setMinimumHeight(300);

  // setup a timer that repeatedly calls UpdatePlot
  dataTimer = new QTimer;
  connect(dataTimer, SIGNAL(timeout()), this, SLOT(UpdatePlot()));
  dataTimer->start(1000.0/RATE);

  // filter setup
  filter = std::make_unique<Dsp::ForceFilter>();

  // gazebo setup
  transport::init();
  transport::run();
  transport::NodePtr node(new transport::Node());
  node->Init("default");
  this->pub  = node->Advertise<msgs::Factory>(factoryPath);
  this->subs = node->Subscribe(this->topicPath, &Interface::Update, this);

  // info initialization
  topicLayout->addWidget(new QLabel(pub->GetTopic().c_str()),  0, 0);
  topicLayout->addWidget(new QLabel(subs->GetTopic().c_str()), 1, 0);

  // window setup
  this->setWindowTitle(("VectorGUI [" + this->topicPath + "]").c_str());
  this->move(20, 20);
  this->setMaximumWidth(480);
  this->setMinimumWidth(480);
  this->setMaximumHeight(660);
  this->setMinimumHeight(660);
}

Interface::~Interface()
{
  transport::fini();
  std::cout << "VectorGUI interface [" << this->topicPath << "] ended" << std::endl;
}

void Interface::setObjectContact(std::string name)
{
  contactData.at(0)->setText(QString::fromStdString(name));
}

void Interface::setPosition(ignition::math::Vector3d pos)
{
  contactData.at(3)->setText(QString::fromStdString(d2s(pos.X())));
  contactData.at(4)->setText(QString::fromStdString(d2s(pos.Y())));
  contactData.at(5)->setText(QString::fromStdString(d2s(pos.Z())));
}

void Interface::setForce(ignition::math::Vector3d force)
{
  contactData.at(6)->setText(QString::fromStdString(d2s(force.X())));
  contactData.at(7)->setText(QString::fromStdString(d2s(force.Y())));
  contactData.at(8)->setText(QString::fromStdString(d2s(force.Z())));
}

std::string Interface::d2s(double d)
{
  std::ostringstream strs;
  strs << d;
  return strs.str();
}

void Interface::Update(ConstContactsPtr& message)
{
  boost::mutex::scoped_lock lock(mutex);
  ignition::math::Vector3d position;
  ignition::math::Vector3d force = ignition::math::Vector3d::Zero;

  if (message->contact_size() > 0)
  {
    std::string name;
    position = msgs::Convert(message->contact(0).position(0));
    int contactCount = message->contact_size();
    for (int n = 0; n < contactCount; ++n)
    {
      for (int m = 0; m < message->contact(n).wrench_size(); ++m)
      {
        if (message->contact(n).wrench(m).body_1_name().find(robotName) != std::string::npos)
        {
          name  = message->contact(n).wrench(m).body_2_name();
          force += msgs::Convert(message->contact(n).wrench(m).body_1_wrench().force());
        }
        else
        {
          name  = message->contact(n).wrench(m).body_1_name();
          force += msgs::Convert(message->contact(n).wrench(m).body_2_wrench().force());
        }
      }
    }

    force = force / contactCount;

    if (++counter > 10)
    {
      counter = 0;
      if (force.Length() > NOISE_THRESHOLD)
      {
        this->setObjectContact(name);
        this->setPosition(position);
        this->setForce(force);
      }
    }
  }

  double t    = message->time().sec() + 1e-9 * message->time().nsec();
  double raw  = filter->Filter(&force);  // returns pre-filter length; modifies force in-place
  double filt = force.Length();          // post-filter magnitude

  plot->graph(0)->addData(t, raw);
  plot->graph(1)->addData(t, filt);
  lastTime  = t;
  lastForce = raw;
}

void Interface::SpawnModel()
{
  Spawn("model://" + dropMenu->currentText().toStdString(),
        ignition::math::Pose3d(entries.at(0)->text().toDouble(),
                               entries.at(1)->text().toDouble(),
                               entries.at(2)->text().toDouble(),
                               0, 0, 0));
}

void Interface::Spawn(std::string _path, ignition::math::Pose3d _pose)
{
  msgs::Factory msg;
  msg.set_sdf_filename(_path);
  msgs::Set(msg.mutable_pose(), _pose);
  this->pub->WaitForConnection();
  this->pub->Publish(msg);

  std::cout << " >> " << _path << " spawned at "
            << std::endl
            << "   pos: (" << _pose.Pos().X() << ", "
                           << _pose.Pos().Y() << ", "
                           << _pose.Pos().Z() << ");" << std::endl
            << "   rot: (" << _pose.Rot().X() << ", "
                           << _pose.Rot().Y() << ", "
                           << _pose.Rot().Z() << ");" << std::endl;
}

void Interface::UpdatePlot()
{
  boost::mutex::scoped_lock lock(mutex);
  if (lastTime > TIME_MAX)
    plot->xAxis->setRange(lastTime - TIME_MAX, lastTime);

  if (lastForce > forceMax)
  {
    forceMax = lastForce;
    plot->yAxis->setRange(0, forceMax);
  }
  plot->replot();
}
