#include "vectorview/ContactMessage.h"

namespace vectorview {

std::vector<ContactForce> ContactsFromMessage(const gz::msgs::Contacts& message) {
  std::vector<ContactForce> contacts;
  contacts.reserve(message.contact_size());

  for (int n = 0; n < message.contact_size(); ++n) {
    const gz::msgs::Contact& contact_msg = message.contact(n);
    ContactForce contact;

    for (int m = 0; m < contact_msg.wrench_size(); ++m) {
      const gz::msgs::JointWrench& wrench_msg = contact_msg.wrench(m);
      WrenchForce wrench;
      wrench.body_1_name = wrench_msg.body_1_name();
      wrench.body_2_name = wrench_msg.body_2_name();

      const gz::msgs::Wrench& body_1_wrench = wrench_msg.body_1_wrench();
      const gz::msgs::Wrench& body_2_wrench = wrench_msg.body_2_wrench();
      wrench.body_1_force = Vec3(body_1_wrench.force().x(), body_1_wrench.force().y(),
                                 body_1_wrench.force().z());
      wrench.body_2_force = Vec3(body_2_wrench.force().x(), body_2_wrench.force().y(),
                                 body_2_wrench.force().z());
      contact.wrenches.push_back(wrench);
    }

    contacts.push_back(contact);
  }

  return contacts;
}

}  // namespace vectorview
