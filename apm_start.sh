#!/bin/bash
gnome-terminal \
--tab -e 'bash -c "echo -ne \"\033]0;roscore\007\"; roscore; exec bash"' \
--tab -e 'bash -c "echo -ne \"\033]0;rosdue0\007\"; sleep 2; rosrun rosdue serial_due.py /dev/ttyACM0; exec bash"' \
--tab -e 'bash -c "echo -ne \"\033]0;rosdue1\007\"; sleep 2; rosrun rosdue serial_due.py /dev/ttyACM1; exec bash"' \
--tab -e 'bash -c "echo -ne \"\033]0;roslaunch\007\"; sleep 2; roslaunch launch apm_start.launch; exec bash"' \
--tab -e 'bash -c "echo -ne \"\033]0;roslaunch\007\"; sleep 2; rosrun safezone safezone; exec bash"' \
--tab -e 'bash -c "echo -ne \"\033]0;apf_navigation\"; sleep 2; python ~/apm/apm_phase5/apf_navigation.py; exec bash"' \
--tab -e 'bash -c "echo -ne \"\033]0;velodyne_tracking\"; sleep 2; cd ~/media/rosmaster/Elements/bagfiles; rm *.active; rosbag record /velodyne_points /camera/image /vectornav/IMU /vectornav/GPS /tf; exec bash"'\

# Specify <name> and <cmd> for each terminal
# Example: --tab -e 'bash -c "echo -ne \"\033]0;<name>\007\"; <cmd>"' \
# Previous conventions were changed due to update and reorganization, but the old script commands are left commented.
#	--tab\
#		--title="roscore"\
#		--working-directory="/home"\
#		-e "bash -c 'roscore'"\
#	--tab\
#		--title="rosdue0"\
#		--working-directory="/home"\
#		-e "bash -c 'sleep 2; rosrun rosdue serial_due.py /dev/ttyACM0'"\
#	--tab\
#		--title="rosdue1"\
#		--working-directory="/home"\
#		-e "bash -c 'sleep 2; rosrun rosdue serial_due.py /dev/ttyACM1'"\
#	--tab\
#		--title="roslaunch"\
#		--working-directory="/home"\
#		-e "bash -c 'sleep 2; roslaunch apm_start apm_start.launch'"\
