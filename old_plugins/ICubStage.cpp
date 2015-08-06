#include "ICubStage.h"
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(ICubStage)

ICubStage::ICubStage()
{

}

ICubStage::~ICubStage()
{

}

void ICubStage::Init()
{

}

void ICubStage::Load(int _argc, char **_argv)
{
	// start Interface
	QApplication app(_argc, _argv);
	this->interface = new Interface();
	this->interface->show();

	// define this plugin as a listener of the sensor topic defined in topic_path
	transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	// we get from the node the subscriber of the sensor messages
	this->subs = node->Subscribe("/gazebo/default/iCub_fixed/iCub/l_hand/l_hand_contact", &ICubStage::Update, this);

	if(app.exec())
	{
		std::cout << "(!) Everything ran fine." << std::endl;
	} else {
		std::cout << "(!) Execution problem!" << std::endl;
	}
}

void ICubStage::Update(ConstContactsPtr &message)
{
	if(message->contact_size() > 0)
	{
		math::Vector3 position = msgs::Convert(message->contact(0).position(0));
		math::Vector3 force = msgs::Convert(message->contact(0).wrench(0).body_1_wrench().force());
		std::string name = message->contact(0).wrench(0).body_2_name();
		if(force.GetLength() > .001)
		{
			interface->setObjectContact(name);
			interface->setPosition(position);
			interface->setForce(force);
		}
	}
}
