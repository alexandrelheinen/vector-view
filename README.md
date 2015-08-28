# VectorView and VectorGUI#

**VectorView** (or libvectorview.so) is a *Gazebo Visual Plugin* that, once added to a `<visual>` element of *iCub model*, will display a vector which represents all contact forces of this link's contact sensor.

This plugin is set up to use the icub-gazebo model that can be found on [its official git repository] (https://github.com/robotology-playground/icub-gazebo), and copies of used models are saved on **models** folder. Those models were already set up with contact sensor as needed. Furthermore, any name logic or model reference are based on the given model standards.

**VectorGUI** (or binary vectorGUI) is an extern application that allows to spawn objects during Gazebo simulation, displays contacts informations retrieved from VectorView and a real-time plot of (filtered) force magnitude.

#### Some name rules to the plugin ####
The name of the contact sensor must be "LINK_NAME_contact". For instance if you set it at the "l_hand" link, your contact sensor should be "l_hand_contact". Anyway be free to change this rule to something more intelligent.

## Dependencies ##

Just a list of required packages (and each one has it own dependencies):
 * Gazebo: http://gazebosim.org/tutorials?tut=install_from_source&cat=install with **SDFormat**
 * iCub: http://wiki.icub.org/wiki/Linux:Installation_from_sources with **YARP**
 * [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)
 * [icub-gazebo](https://github.com/robotology-playground/icub-gazebo)
 * It should be all, however [ocra-core](https://github.com/ocra-recipes/ocra-core) and [codyco-superbuild](https://github.com/alexandrelheinen/codyco-superbuild) with [`ISIR_MODULES`](https://github.com/alexandrelheinen/codyco-superbuild#a-note-on-ocra-wbi-plugins) are also used in this project. Those are versions from my own repository where some changes were made the contact forces analysis.

 VectorView uses [DSPFilters](https://github.com/vinniefalco/DSPFilters), a collection of C++ classes for digital signal filtering, and [QCustomPlot](http://www.qcustomplot.com/), a Qt C++ widget for plotting and data visualization, but source and headers files from both projects are already included in `src` and `include` folders respectively, so VectorView has no direct dependency of those packages.

## Installation ##

To use VectorView Plugin you just need to compile it and set the [environmental variables](#useful-environment-variables) whom able Gazebo to find the libraries you just built
```bash
git clone https://github.com/alexandrelheinen/vector-view.git && cd vector-view
```
and compile it in `build` folder
```bash
mkdir build && cd build
cmake .. && make
```
## Useful environmental variables ##

As VectorView is a Gazebo Plugin, some environment variables must be set up to assure that Gazebo client will find out all files it needs:
```bash
export VECTOR_VIEW=where_you_cloned_it/vector-view
export PATH=$VECTOR_VIEW/build:${PATH}
export GAZEBO_PLUGIN_PATH=$VECTOR_VIEW/build:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=$VECTOR_VIEW/models:${GAZEBO_MODEL_PATH}
```
## Test it! ##

By running those three commands in respective order in different terminals to test the plugin (give your computer some time to process each command):
```bash
yarpserver
cd $VECTOR_VIEW && gazebo robot.world
ISIRWholeBodyController --sequence StageTestTasks
```

While the simulation is running, in another terminal can run
```bash
vectorGUI
```
to pop out the external interface. On this window the contact object name is displayed as well as the forces involved on this contact and where it has take place.

![interface window example](/images/gui_example.png "Interface window example")

By clicking on **Spawn** button, the chosen model in the drop down menu is spawn at set Cartesian location (x, y, z).

### SHELL SCRIPT ###

To easily start the plugin, just run the shell script `run.sh` as follows. It will trigger out all application modules (maybe you should change its permission by typing `chmod +x run.sh` on your terminal), including test sequence tasks.
```bash
cd $VECTOR_VIEW && ./run.sh
```
The execution should be something like in the figure below
![shell script execution example](/images/execution_example.png "shell script execution example")
