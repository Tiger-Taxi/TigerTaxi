#!/bin/bash
dbus-launch gnome-terminal \
--tab -e 'bash -c "echo -ne \"\033]0;roscore\007\"; roscore; exec bash"' \
--tab -e 'bash -c "echo -ne \"\033]0;roslaunch\007\"; sleep 2; roslaunch tt_start tt_start.launch; exec bash"'

# --tab -e 'bash -c "echo -ne \"\033]0;velodyne_tracking\"; sleep 2; cd /media/rosmaster/Elements/bagfiles; rm *.active; rosbag record /velodyne_points /camera/image /vectornav/IMU /vectornav/GPS /scan; exec bash"'
#--tab -e 'bash -c "echo -ne \"\033]0;roslaunch\007\"; sleep 2; rosrun --prefix \"gdb --args\" safezone safezone; exec bash"'
#--tab -e 'bash -c "echo -ne \"\033]0;rosdue0\007\"; sleep 2; rosrun rosdue serial_due.py /dev/ttyACM0; exec bash"' \
#--tab -e 'bash -c "echo -ne \"\033]0;apf_navigation\"; sleep 2; python ~/TigerTaxi/tt_core/apf_navigation.py; exec bash"' \

#--tab -e 'bash -c "echo -ne \"\033]0;rosdue1\007\"; sleep 2; rosrun rosdue serial_due.py /dev/ttyACM1; exec bash"' \
