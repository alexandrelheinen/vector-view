#include "plugins/include/Interface.h"

using namespace gazebo;

Interface::Interface(QWidget *parent) : QWidget(parent)
{
  layout = new QGridLayout(this);

  // labels initialization
  contactLabels.push_back(new QLabel("Object:",this));    contactData.push_back(new QLabel("ground",this));
  contactLabels.back()->setAlignment(Qt::AlignRight);     contactData.push_back(NULL);
                                                          contactData.push_back(NULL);
  contactLabels.push_back(new QLabel("Position:",this));  contactData.push_back(new QLabel("1.5",   this));
  contactLabels.back()->setAlignment(Qt::AlignRight);     contactData.push_back(new QLabel("-2.0",  this));
                                                          contactData.push_back(new QLabel("1.0",   this));
  contactLabels.push_back(new QLabel("Force:",this));     contactData.push_back(new QLabel("0.02",   this));
  contactLabels.back()->setAlignment(Qt::AlignRight);     contactData.push_back(new QLabel("-.04",  this));
                                                          contactData.push_back(new QLabel("0.0",   this));
  // input those labels on the layout
  unsigned int k;
  for (k = 0; k < contactLabels.size(); ++k)
  {
    layout->addWidget(contactLabels.at(k), 3*k, 0);
  }
  for (k = 0; k < contactData.size(); ++k)
  {
    layout->addWidget(contactData.at(k), k, 1);
  }
  setWindowTitle(tr("Contact Sensor Data"));
  resize(200, 100);
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
