ROSJAUS
=======

ROSJAUS

	cd ~/catkin_ws/src

	wstool set ROSJAUS --git https://github.com/udmamrl/ROSJAUS.git

	wstool update ROSJAUS

	cd ~/catkin_ws

	catkin_make ROSJAUS

	catkin_make WaypointCOP 

# test

# close all other ros program (stage/jaus/move_base….)

	roscd ROSJAUS

	Run_ROSJAUS_Stage_script.sh

