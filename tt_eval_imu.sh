#!/bin/bash
gnome-terminal \
--tab -e 'bash -c "echo -ne \"\033]0;roscore\007\"; roscore; exec bash"' \
--tab -e 'bash -c "echo -ne \"\033]0;roslaunch\007\"; sleep 2; roslaunch tt_start tt_eval_imu.launch; exec bash"'

#--tab -e 'bash -c "echo -ne \"\033]0;velodyne_tracking\"; sleep 2; cd /media/rosmaster/Elements/bagfiles; rm *.active; rosbag record /velodyne_points /camera/image /vectornav/IMU /vectornav/GPS /scan; exec bash"'

