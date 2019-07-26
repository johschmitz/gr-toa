# gr-toa - GNU Radio time of arrival estimation based localization

This project contains a GNU Radio signal processing block that estimtates the
times of arrival (TOAs) of a signal emitted by a mobile tag at three or more
distributed receivers. A stationary reference transmitter at a known location
is required for time synchronization. The TOA estimates are transmitted to a
central server using ZeroMQ and the current location of the tag is then
estimated from all collected measurements. Finally the result can be displayed
on a map in the graphical user interface.

## Dependencies

- GNU Radio 3.7.13
- gr-osmosdr
- python3-numpy

## How to build

    mkdir build
    cd build
    cmake ../

Don't forget to set the install prefix if desired

    make
    make install

## Environment variables

Let's assume for a moment that the install prefix is /opt/gr_toa. Then set these environment variables, e.g. in your .bashrc to enable GNU Radio Companion to find the block:

    export PATH="/opt/gr_toa/bin:$PATH"
    export LD_LIBRARY_PATH="/opt/gr_toa/lib:$LD_LIBRARY_PATH"
    export PYTHONPATH="/opt/gr_toa/lib/python2.7/dist-packages:$PYTHONPATH"
    export GRC_BLOCKS_PATH="/opt/gr_toa/share/gnuradio/grc:$GRC_BLOCKS_PATH"
