# VectorView #

**VectorView** (or libupvector.so) is a *Gazebo Visual Plugin* that, once it's added to a visual on an *iCub model*, will display a vector which represents all contact forces of this link's contact sensor.

Some name rules to the plugin:
The name of the contact sensor must be "LINK_NAME_contact". For instance if you set it at the "l_hand" link, your contact sensor should be "l_hand_contact". Anyway be free to change this rule to something more intelligent. Furthermore, all the output data concerning the contact forces' history will be saved at the "history_ROBOT_NAME_LINK_NAME.txt" file.

This plugin is set up to use the icub-gazebo model that can be found on [its official git repository] (https://github.com/robotology-playground/icub-gazebo). So all used models are saved on the **model folder** and any name logic or model reference are based on the given model standards. As well there are **scilab scripts** that could be useful to results analysis.

There is as well a folder where all **old plugins** are abandoned. Generally they are secondary plugins whose attempt to give extra informations about the iCub model's behavior.

## Installation ##

To use the VectorView Plugin you just need to compile it and set the [environmental variables](#useful-environment-variables) whom able Gazebo to find the libraries you just built.

`git clone https://github.com/alexandrelheinen/vector-view.git && cd vector-view`

Just pick this branch: fifth-icub (good question why this brach has this name) as

`git checkout fifth-icub`

And compile it at `build` folder

`mkdir build && cd build`

`cmake .. && make`

## Dependencies ##

Just a list of required packages (and each one has it own dependencies):
 * Gazebo: http://gazebosim.org/tutorials?tut=install_from_source&cat=install with **SDFormat**
 * iCub: http://wiki.icub.org/wiki/Linux:Installation_from_sources with **YARP** 
 * [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)
 * [icub-gazebo](https://github.com/robotology-playground/icub-gazebo)
 * It should be all, however [codyco-superbuild](https://github.com/robotology/codyco-superbuild) with [`ISIR_MODULES`](https://github.com/robotology/codyco-superbuild#a-note-on-ocra-wbi-plugins) and [ocra-core](https://github.com/ocra-recipes/ocra-core) are also used in this project. Look at https://github.com/alexandrelheinen/codyco-superbuild at the branch `stage` to find out some changes that were made to the contact forces analysis.
  
## Useful environment variables ##

As the VectorView project is a Gazebo Plugin, some environment variables must be set up to assure that the plugin will find out all files that it needs:

`export VECTOR_VIEW = where_you_cloned_the_repository/vector-view`

`export GAZEBO_PLUGIN_PATH=$VECTOR_VIEW/build:${GAZEBO_PLUGIN_PATH}`

`export GAZEBO_MODEL_PATH=$VECTOR_VIEW/models:${GAZEBO_MODEL_PATH}`
