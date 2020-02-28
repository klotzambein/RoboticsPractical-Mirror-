#!/bin/bash

scp /home/group33/catkin_ws/src/group33/build/motor_controller.ino.hex nano-sudo:~/
ssh nano-sudo 'teensy_loader_cli --mcu=TEENSY36 -v -s *.hex'
