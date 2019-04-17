#!/bin/bash

xterm -geometry 100x16+0+0 -e "cd ..; . debugenv; cd apps; ./simulated_receivers.py" &
xterm -geometry 100x16+650+0 -e "cd ..; . debugenv; cd apps; ./toa_server.py" &
xterm -geometry 100x16+1300+0 -e "cd ..; . debugenv; cd apps; ./toa_gui.py" &
sleep 5
WINDOW_ID=$(xdotool search --name "TOA Localization GUI")
xdotool windowmove $WINDOW_ID 460 250
xdotool windowsize $WINDOW_ID 1400 700