#!/bin/bash

roscore -p 30001 &

sleep 5

roslaunch khi_robot_bringup rs007l_bringup.launch ip:=192.168.50.11

pkill roscore
