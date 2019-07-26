#!/usr/bin/env python2

import signal
import sys
import os
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import osmosdr
import time
from toa import toa_estimator_pub, config_file_parser
import toa
import argparse

class top_block(gr.top_block):

    def __init__(self, cfg, args):
        gr.top_block.__init__(self, "Top Block")

        ##################################################
        # Variables
        ##################################################
        self.f_carrier = cfg["receiver"]["f_carrier"]
        self.sample_rate = cfg["receiver"]["sample_rate"]
        self.num_chips = cfg["receiver"]["num_chips"]
        self.chip_rate = cfg["receiver"]["chip_rate"]
        self.samples_per_burst = int(self.num_chips*self.sample_rate/self.chip_rate)
        self.fft_size = 3*self.samples_per_burst
        self.gain = cfg["receiver"]["gain"]
        self.if_gain = cfg["receiver"]["if_gain"]
        self.bb_gain = cfg["receiver"]["bb_gain"]
        self.acquisition_interval = cfg["receiver"]["acquisition_interval"]
        self.detection_threshold = cfg["receiver"]["detection_threshold"]
        self.max_tracking_fails = cfg["receiver"]["max_tracking_fails"]
        self.bp_cutoff_low = cfg["receiver"]["bp_cutoff_low"]
        self.bp_cutoff_high = cfg["receiver"]["bp_cutoff_high"]
        self.bp_transition_width = cfg["receiver"]["bp_transition_width"]

        self.zmq_publisher_addr = "tcp://*:" + str(6000+args.id)

        ##################################################
        # Blocks
        ##################################################
        self.toa_toa_estimator_pub = toa.toa_estimator_pub(self.fft_size, \
            self.sample_rate, self.acquisition_interval, \
            self.detection_threshold, self.max_tracking_fails, \
            -1, args.sequence_list_file, self.zmq_publisher_addr)
        self.osmosdr_source = osmosdr.source(args=args.device_args)
        self.osmosdr_source.set_sample_rate(self.sample_rate)
        self.osmosdr_source.set_center_freq(self.f_carrier, 0)
        self.osmosdr_source.set_freq_corr(0, 0)
        self.osmosdr_source.set_dc_offset_mode(0, 0)
        self.osmosdr_source.set_iq_balance_mode(0, 0)
        self.osmosdr_source.set_gain_mode(False, 0)
        self.osmosdr_source.set_gain(self.gain, 0)
        self.osmosdr_source.set_if_gain(self.if_gain, 0)
        self.osmosdr_source.set_bb_gain(self.bb_gain, 0)
        self.osmosdr_source.set_antenna('', 0)
        self.osmosdr_source.set_bandwidth(0, 0)

        self.blocks_sub_xx = blocks.sub_ff(1)
        self.blocks_complex_to_mag_0_0 = blocks.complex_to_mag(1)
        self.blocks_complex_to_mag_0 = blocks.complex_to_mag(1)
        self.band_pass_filter_0_0 = filter.fft_filter_ccc(1, (firdes.complex_band_pass(1,
            self.sample_rate, self.bp_cutoff_low, self.bp_cutoff_high, self.bp_transition_width,
            firdes.WIN_RECTANGULAR, 1.0)), 1)
        self.band_pass_filter_0_0.declare_sample_delay(0)
        self.band_pass_filter_0 = filter.fft_filter_ccc(1, (firdes.complex_band_pass(1,
            self.sample_rate, -self.bp_cutoff_high, -self.bp_cutoff_low, self.bp_transition_width,
            firdes.WIN_RECTANGULAR, 1.0)), 1)
        self.band_pass_filter_0.declare_sample_delay(0)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.band_pass_filter_0, 0), (self.blocks_complex_to_mag_0, 0))
        self.connect((self.band_pass_filter_0_0, 0), (self.blocks_complex_to_mag_0_0, 0))
        self.connect((self.blocks_complex_to_mag_0, 0), (self.blocks_sub_xx, 0))
        self.connect((self.blocks_complex_to_mag_0_0, 0), (self.blocks_sub_xx, 1))
        self.connect((self.blocks_sub_xx, 0), (self.toa_toa_estimator_pub, 0))
        self.connect((self.osmosdr_source, 0), (self.band_pass_filter_0, 0))
        self.connect((self.osmosdr_source, 0), (self.band_pass_filter_0_0, 0))

    def sigint_handler(self, sig, frame):
        self.stop()
        self.wait()
        sys.exit(0)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", type=int, default="0",
                        help="Receiver id. Default: 0.")
    parser.add_argument("-d", "--device-args", type=str, default="rtl=0",
                        help="OsmoSDR source device args, e.g., "
                        "hackrf=0, rtl=1")
    parser.add_argument("-c", "--config-file", type=str, default="",
                        help="Path to config file. Default: <bin_dir>/toa_config.ini")
    parser.add_argument("-s", "--sequence-list-file", type=str, default="",
                        help="Path to sequence list file with CDMA reference sequences. "
                        "Default: <bin_dir>/reference_files/sequence_list.txt")
    args = parser.parse_args()
    # Set default paths for config files
    if 0 == len(args.config_file):
        args.config_file = os.path.dirname(sys.argv[0]) + "/toa_config.ini"
    if 0 == len(args.sequence_list_file):
        args.sequence_list_file = os.path.dirname(sys.argv[0]) + "/reference_files/sequence_list.txt"
    return args

def main():
    args = parse_args()
    print("Using config file:", args.config_file)
    print("Using sequence list file:", args.sequence_list_file)
    cfg = config_file_parser.parse_config_file(args.config_file)
    tb = top_block(cfg, args)
    tb.start()
    signal.signal(signal.SIGINT, tb.sigint_handler)
    signal.pause()
    tb.stop()
    tb.wait()

if __name__ == '__main__':
    main()
