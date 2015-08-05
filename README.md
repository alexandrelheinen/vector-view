# VectorView #

**VectorView** (or libupvector.so) is a *Gazebo Visual Plugin* that, once it's added to a visual on an *iCub model*, will display a vector which represents all contact forces of this link's contact sensor.

Some name rules to the plugin:
The name of the contact sensor must be "LINK_NAME_contact". For instance if you set it at the "l_hand" link, your contact sensor should be "l_hand_contact". Anyway be free to change this rule to something more intelligent. Furthermore, all the output data concerning the contact forces' history will be saved at the "history_ROBOT_NAME_LINK_NAME.txt" file.

This Plugin is set to use the icub-gazebo model that can be found on [its official git page] (https://github.com/robotology-playground/icub-gazebo). Any name logic or model reference used on it is based on the given robot model.  All used models are saved on the **model folder**, as well as some **scilab scripts** that could be useful to results analysis.

## Useful environment variables ##

As the VectorView project is a Gazebo Plugin, some environment variables must be set up to assure the plugin operation.
`export VECTOR_VIEW = where_your_cloned_the_repository
export GAZEBO_PLUGIN_PATH=$VECTOR_VIEW/build:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=$VECTOR_VIEW/models:${GAZEBO_MODEL_PATH}`
