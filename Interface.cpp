#include "Interface.h"

using namespace gazebo;

Interface::Interface(std::string _path) : QWidget(NULL)
{
  this->topicPath   = _path;
  this->factoryPath = "~/factory";
  counter = 0;

  models.push_back("Sphere");
  models.push_back("Robot");
  models.push_back("Cylinder");
  models.push_back("Box");
  models.push_back("Table");

  mainLayout    = new QGridLayout(this);

  QGridLayout *topicLayout = new QGridLayout();
  QGridLayout *infoLayout  = new QGridLayout();
  contactLayout            = new QGridLayout();
  buttonLayout             = new QGridLayout();

  QFrame *infoFrame       = new QFrame();
  QGroupBox *topicFrame   = new QGroupBox(tr("Current topic path"));
  QGroupBox *contactFrame = new QGroupBox(tr("Contacts"));
  QGroupBox *buttonFrame  = new QGroupBox(tr("Spawn Objects"));

  infoFrame->setLayout(infoLayout);
  topicFrame->setLayout(topicLayout);
  contactFrame->setLayout(contactLayout);
  buttonFrame->setLayout(buttonLayout);

  mainLayout->addWidget(topicFrame,   0, 0);
  mainLayout->addWidget(infoFrame,    1, 0);
  infoLayout->addWidget(contactFrame, 0, 0);
  infoLayout->addWidget(buttonFrame,  0, 1);

  // labels initialization
  contactLabels.push_back(new QLabel("Object:",this));    contactData.push_back(new QLabel("ground",this));
  contactLabels.back()->setAlignment(Qt::AlignRight);     contactData.push_back(NULL);
                                                          contactData.push_back(NULL);
  contactLabels.push_back(new QLabel("Position:",this));  contactData.push_back(new QLabel("1.5",   this));
  contactLabels.back()->setAlignment(Qt::AlignRight);     contactData.push_back(new QLabel("-2.0",  this));
                                                          contactData.push_back(new QLabel("1.0",   this));
  contactLabels.push_back(new QLabel("Force:",this));     contactData.push_back(new QLabel("0.02",  this));
  contactLabels.back()->setAlignment(Qt::AlignRight);     contactData.push_back(new QLabel("-.04",  this));
                                                          contactData.push_back(new QLabel("0.0",   this));

  // input those labels on the layout
  unsigned int k;
  for (k = 0; k < contactLabels.size(); ++k)
    contactLayout->addWidget(contactLabels.at(k), 3*k, 0);

  for (k = 0; k < contactData.size(); ++k)
    contactLayout->addWidget(contactData.at(k), k, 1);

  // buttons initialization
  for (k = 0; k < models.size(); ++k)
  {
    buttons.push_back(new QPushButton(models.at(k).c_str()));
    connect(buttons.back(), SIGNAL(clicked()), this, SLOT(SpawnSphere())); // with "bind" we can set each button to a different model
    buttonLayout->addWidget(buttons.back(), k, 0);
  }

  // window setup
  this->setWindowTitle(tr("Contact Sensor Data"));
  //this->resize(200, 100);
  this->move(10, 10);

  // gazebo setup
  transport::init();
  transport::run();
  transport::NodePtr node(new transport::Node()); // define this plugin as a listener of the sensor topic defined in topic_path
  node->Init("default");
  this->pub  = node->Advertise<msgs::Factory>(factoryPath);
  this->subs = node->Subscribe(this->topicPath, &Interface::Update, this);
  // info initialization
  topicLayout->addWidget(new QLabel(pub->GetTopic().c_str()),  0, 0);
  topicLayout->addWidget(new QLabel(subs->GetTopic().c_str()), 1, 0);
}

Interface::~Interface()
{
  transport::fini();
  std::cout << "Vector interface ended" << std::endl;
}

// set object name
void Interface::setObjectContact(std::string name)
{
  contactData.at(0)->setText(
															QString::fromStdString(
																											name));
}

// set the position where the contact happened base on the gazebo variable
void Interface::setPosition(math::Vector3 pos)
{
	contactData.at(3)->setText(
															QString::fromStdString(
																											d2s(pos.x)));
	contactData.at(4)->setText(
															QString::fromStdString(
																											d2s(pos.y)));
	contactData.at(5)->setText(
															QString::fromStdString(
																											d2s(pos.z)));
}

// set the contact force based on the gazebo variable
void Interface::setForce(math::Vector3 force)
{
	contactData.at(6)->setText(
															QString::fromStdString(
																											d2s(force.x)));
	contactData.at(7)->setText(
															QString::fromStdString(
																											d2s(force.y)));
	contactData.at(8)->setText(
															QString::fromStdString(
																											d2s(force.z)));
}

// auxiliar function to convert double to string
std::string Interface::d2s(double d)
{
  std::ostringstream strs;
  strs << d;
  return strs.str();
}

void Interface::Update(ConstContactsPtr &message)
{
	if(message->contact_size() > 0)
	{
		math::Vector3 position = msgs::Convert(message->contact(0).position(0));
		math::Vector3 force = msgs::Convert(message->contact(0).wrench(0).body_1_wrench().force());
		std::string name = message->contact(0).wrench(0).body_2_name();
		if(force.GetLength() > .001)
		{
			this->setObjectContact(name);
			this->setPosition(position);
			this->setForce(force);
		}
	}
}

void Interface::SpawnSphere()
{
  Spawn("model://bookshelf", math::Pose(1, 1, 1, 1, 1, 1));
}

void Interface::Spawn(std::string model, math::Pose pose)
{
  msgs::Factory msg;
  msg.set_sdf_filename(model);
  msgs::Set(msg.mutable_pose(), pose);
  this->pub->WaitForConnection();
  this->pub->Publish(msg);
  std::cout << " >> " << model << " spawned at "                                             << std::endl
            << "   pos: (" << pose.pos.x << ", " << pose.pos.y << ", " << pose.pos.z << ");" << std::endl
            << "   rot: (" << pose.rot.x << ", " << pose.rot.y << ", " << pose.rot.z << ");" << std::endl;
}
