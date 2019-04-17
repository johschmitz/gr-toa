#!/bin/bash

xterm -geometry 74x16+0+0 -e "cd ..; . debugenv; cd apps; ./toa_receiver.py -i 0 -d rtl=0" &
xterm -geometry 74x16+450+0 -e "cd ..; . debugenv; cd apps; ./toa_receiver.py -i 1 -d rtl=1" &
xterm -geometry 74x16+0+240 -e "cd ..; . debugenv; cd apps; ./toa_receiver.py -i 2 -d hackrf=0" &
xterm -geometry 74x16+900+0 -e "cd ..; . debugenv; cd apps; ./toa_server.py" &
xterm -geometry 74x16+1350+0 -e "cd ..; . debugenv; cd apps; ./toa_gui.py;" &
sleep 5
WINDOW_ID=$(xdotool search --name "TOA Localization GUI")
xdotool windowmove $WINDOW_ID 460 250
xdotool windowsize $WINDOW_ID 1400 700