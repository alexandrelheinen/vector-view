echo "1. Starting YARP Server to iCub control."
gnome-terminal --tab -e "yarpserver"
sleep 2
echo "2. Running [robot.world] at Gazebo simulator."
gnome-terminal --tab -e "gazebo robot.world"
sleep 3
echo "3. Openning the GUI Interface to force analysis."
echo "3.1. topic path: /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact"
gnome-terminal --tab -e "./build/VectorGUI /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact" # first way, passing the whole path
sleep 2
echo "3.2. topic path: /gazebo/default/iCub_fixed/iCub/l_hand/l_hand_contact"
gnome-terminal --tab -e "./build/VectorGUI l_hand" # or you can just give the link name, it will assume that's iCub_fixed model
sleep 2
echo "4. Running [StageTestTasks] sequence of ISIR Controller."
gnome-terminal --tab -e "$CODYCO_SUPERBUILD_ROOT/build/install/bin/ISIRWholeBodyController --sequence StageTestTasks"
echo " -------------------------------------------------------"
