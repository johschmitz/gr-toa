#!/bin/bash

xterm -geometry 74x19+0+0 -e "cd ..; . debugenv; cd apps; ./toa_receiver.py -i 0 -d rtl=0 -c toa_config_test.ini" &
xterm -geometry 74x19+450+0 -e "cd ..; . debugenv; cd apps; ./toa_receiver.py -i 1 -d rtl=1 -c toa_config_test.ini" &
xterm -geometry 74x19+0+280 -e "cd ..; . debugenv; cd apps; ./toa_receiver.py -i 2 -d hackrf=0 -c toa_config_test.ini" &
xterm -geometry 74x19+900+0 -e "cd ..; . debugenv; cd apps; ./toa_server.py -c toa_config_test.ini" &
xterm -geometry 74x19+1350+0 -e "cd ..; . debugenv; cd apps; ./toa_gui.py -c toa_config_test.ini;" &
sleep 5
WINDOW_ID=$(xdotool search --name "TOA Localization GUI")
xdotool windowmove $WINDOW_ID 450 280
xdotool windowsize $WINDOW_ID 1400 700