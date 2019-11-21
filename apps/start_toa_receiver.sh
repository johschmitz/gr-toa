#!/bin/bash

# GNU Radio
GNURADIO_DIR=/opt/gnuradio_3_7_13_5_git
export PATH=$PATH:$GNURADIO_DIR/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GNURADIO_DIR/lib
export PYTHONPATH=$PYTHONPATH:$GNURADIO_DIR/lib/python2.7/dist-packages

# gr-osmosdr
GR_OSMOSDR_DIR=/opt/gr_osmosdr_0_1_5_git
export PATH="$GR_OSMOSDR_DIR/bin:$PATH"
export LD_LIBRARY_PATH="$GR_OSMOSDR_DIR/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$GR_OSMOSDR_DIR/lib/python2.7/dist-packages:$PYTHONPATH"

# gr-toa
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/gr-toa/lib
export PYTHONPATH=$PYTHONPATH:/opt/gr-toa/lib/python2.7/dist-packages

# Start receiver
$(dirname "$0")/toa_receiver.py -i 1

