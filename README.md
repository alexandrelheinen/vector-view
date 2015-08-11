# VectorView #

**VectorView** (or libupvector.so) is a *Gazebo Visual Plugin* that, once it's added to a visual on an *iCub model*, will display a vector which represents all contact forces of this link's contact sensor.

Some name rules to the plugin:
The name of the contact sensor must be "LINK_NAME_contact". For instance if you set it at the "l_hand" link, your contact sensor should be "l_hand_contact". Anyway be free to change this rule to something more intelligent. Furthermore, all the output data concerning the contact forces' history will be saved at the "history_ROBOT_NAME_iCub_LINK_NAME.txt" file.

This plugin is set up to use the icub-gazebo model that can be found on [its official git repository] (https://github.com/robotology-playground/icub-gazebo). So all used models are saved on the **model folder** and any name logic or model reference are based on the given model standards. As well there are **scilab scripts** that could be useful to results analysis.

There is as well a folder where all **old plugins** are abandoned. Generally they are secondary plugins whose attempt to give extra informations about the iCub model's behavior.
    
## Dependencies ##

Just a list of required packages (and each one has it own dependencies):
 * Gazebo: http://gazebosim.org/tutorials?tut=install_from_source&cat=install with **SDFormat**
 * iCub: http://wiki.icub.org/wiki/Linux:Installation_from_sources with **YARP** 
 * [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)
 * [icub-gazebo](https://github.com/robotology-playground/icub-gazebo)
 * It should be all, however [ocra-core](https://github.com/ocra-recipes/ocra-core) and [codyco-superbuild](https://github.com/alexandrelheinen/codyco-superbuild) with [`ISIR_MODULES`](https://github.com/alexandrelheinen/codyco-superbuild#a-note-on-ocra-wbi-plugins) are also used in this project. Those are versions from my own repository where some changes were made the contact forces analysis.
 
## Installation ##

To use the VectorView Plugin you just need to compile it and set the [environmental variables](#useful-environment-variables) whom able Gazebo to find the libraries you just built

    git clone https://github.com/alexandrelheinen/vector-view.git && cd vector-view
    
and compile it at `build` folder

    mkdir build && cd build
    cmake .. && make

## Useful environmental variables ##

As the VectorView project is a Gazebo Plugin, some environment variables must be set up to assure that the plugin will find out all files that it needs:

    export VECTOR_VIEW = where_you_cloned_the_repository/vector-view
    export GAZEBO_PLUGIN_PATH=$VECTOR_VIEW/build:${GAZEBO_PLUGIN_PATH}
    export GAZEBO_MODEL_PATH=$VECTOR_VIEW/models:${GAZEBO_MODEL_PATH}

## Test it! ##

Execute those three commands in this order in different terminals to plugin test (give your computer some time to process each command):

    yarpserver
    cd $VECTOR_VIEW && gazebo robot.world
    ISIRWholeBodyController --sequence StageTestTasks
