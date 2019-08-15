#!/bin/bash

xterm -geometry 74x19+900+0 -e "cd ..; . debugenv; cd apps; ./toa_server.py" &
xterm -geometry 74x19+1350+0 -e "cd ..; . debugenv; cd apps; ./toa_gui.py;" &
sleep 5
WINDOW_ID=$(xdotool search --name "TOA Localization GUI")
xdotool windowmove $WINDOW_ID 450 280
xdotool windowsize $WINDOW_ID 1400 700
