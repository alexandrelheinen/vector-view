#ifndef VECTORVIEW_CONTACT_MESSAGE_H
#define VECTORVIEW_CONTACT_MESSAGE_H

#include "vectorview/ContactUtils.h"

#include <gz/msgs/contacts.pb.h>

#include <vector>

namespace vectorview {

std::vector<ContactForce> ContactsFromMessage(const gz::msgs::Contacts& message);

}  // namespace vectorview

#endif
