gnome-terminal --tab -e "yarpserver"
sleep 2
gnome-terminal --tab -e "gazebo robot.world"
sleep 5
gnome-terminal --tab -e "./build/VectorGUI"
sleep 5
gnome-terminal --tab -e "$CODYCO_SUPERBUILD_ROOT/build/install/bin/ISIRWholeBodyController --sequence StageTestTasks"
