# gr-toa GNU Radio time of arrival estimator block

Takes a received binary sequence (after demodulation) as an input together
with a set of reference sequences loaded from file. Outputs estimated TOAs
through a Zeromq publisher.

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

Set these environment variable to enable GNU Radio Companion to find the block:

    export LD_LIBRARY_PATH="<install_prefix>lib:$LD_LIBRARY_PATH"
    export PYTHONPATH="<install_prefix>/lib/python2.7/dist-packages:$PYTHONPATH"
    export GRC_BLOCKS_PATH="<install_prefix>/share/gnuradio/grc:$GRC_BLOCKS_PATH"
