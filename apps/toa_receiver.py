#!/usr/bin/env python2

import signal
import sys
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import osmosdr
import time
import toa
import argparse
import config_file_parser

class top_block(gr.top_block):

    def __init__(self, cfg, args):
        gr.top_block.__init__(self, "Top Block")

        ##################################################
        # Variables
        ##################################################
        self.f_carrier = cfg["receiver"]["f_carrier"]
        self.fsk_deviation = cfg["receiver"]["fsk_deviation"]
        self.sample_rate = cfg["receiver"]["sample_rate"]
        self.num_chips = cfg["receiver"]["num_chips"]
        self.chip_rate = cfg["receiver"]["chip_rate"]
        self.bandpass_decim_rate = cfg["receiver"]["bandpass_decim_rate"]
        self.samples_per_burst = int(self.num_chips*self.sample_rate/self.chip_rate)
        self.fft_size = self.samples_per_burst/self.bandpass_decim_rate*3
        self.gain = cfg["receiver"]["gain"]
        self.if_gain = cfg["receiver"]["if_gain"]
        self.bb_gain = cfg["receiver"]["bb_gain"]
        self.acquisition_interval = cfg["receiver"]["acquisition_interval"]
        self.detection_threshold = cfg["receiver"]["detection_threshold"]
        self.max_tracking_fails = cfg["receiver"]["max_tracking_fails"]
        self.sequence_list_file = cfg["receiver"]["sequence_list_file"]

        self.zmq_publisher_addr = zmq_publisher_addr = "tcp://*:" + str(6000+args.id)

        ##################################################
        # Blocks
        ##################################################
        self.toa_toa_estimator_pub = toa.toa_estimator_pub(self.fft_size, \
            self.sample_rate/self.bandpass_decim_rate, self.acquisition_interval, \
            self.detection_threshold, self.max_tracking_fails, \
            -1, self.sequence_list_file, self.zmq_publisher_addr)
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

        self.blocks_threshold_ff = blocks.threshold_ff(0, 0, 0)
        self.blocks_sub_xx = blocks.sub_ff(1)
        self.blocks_complex_to_mag_0_0 = blocks.complex_to_mag(1)
        self.blocks_complex_to_mag_0 = blocks.complex_to_mag(1)
        self.band_pass_filter_0_0 = filter.fir_filter_ccc(self.bandpass_decim_rate,
            firdes.complex_band_pass(1, self.sample_rate, -self.fsk_deviation,
            0, self.fsk_deviation/2, firdes.WIN_RECTANGULAR, 6.76))
        self.band_pass_filter_0 = filter.fir_filter_ccc(self.bandpass_decim_rate,
            firdes.complex_band_pass(1, self.sample_rate, 0, self.fsk_deviation,
            self.fsk_deviation/2, firdes.WIN_RECTANGULAR, 6.76))



        ##################################################
        # Connections
        ##################################################
        self.connect((self.band_pass_filter_0, 0), (self.blocks_complex_to_mag_0, 0))
        self.connect((self.band_pass_filter_0_0, 0), (self.blocks_complex_to_mag_0_0, 0))
        self.connect((self.blocks_complex_to_mag_0, 0), (self.blocks_sub_xx, 0))
        self.connect((self.blocks_complex_to_mag_0_0, 0), (self.blocks_sub_xx, 1))
        self.connect((self.blocks_sub_xx, 0), (self.blocks_threshold_ff, 0))
        self.connect((self.blocks_threshold_ff, 0), (self.toa_toa_estimator_pub, 0))
        self.connect((self.osmosdr_source, 0), (self.band_pass_filter_0, 0))
        self.connect((self.osmosdr_source, 0), (self.band_pass_filter_0_0, 0))

    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.set_samples_per_symbol(int(self.sample_rate/self.chip_rate))
        self.set_samples_per_burst(int(self.num_chips*self.sample_rate/self.chip_rate))
        self.osmosdr_source.set_sample_rate(self.sample_rate)
        self.band_pass_filter_0_0.set_taps(firdes.complex_band_pass(1, self.sample_rate, -self.fsk_deviation, 0, self.fsk_deviation/2, firdes.WIN_RECTANGULAR, 6.76))
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, self.sample_rate, 0, self.fsk_deviation, self.fsk_deviation/2, firdes.WIN_RECTANGULAR, 6.76))

    def get_num_chips(self):
        return self.num_chips

    def set_num_chips(self, num_chips):
        self.num_chips = num_chips
        self.set_samples_per_burst(int(self.num_chips*self.sample_rate/self.chip_rate))

    def get_chip_rate(self):
        return self.chip_rate

    def set_chip_rate(self, chip_rate):
        self.chip_rate = chip_rate
        self.set_samples_per_symbol(int(self.sample_rate/self.chip_rate))
        self.set_samples_per_burst(int(self.num_chips*self.sample_rate/self.chip_rate))

    def get_samples_per_burst(self):
        return self.samples_per_burst

    def set_samples_per_burst(self, samples_per_burst):
        self.samples_per_burst = samples_per_burst
        self.set_fft_size(self.samples_per_burst/self.bandpass_decim_rate*3)

    def get_bandpass_decim_rate(self):
        return self.bandpass_decim_rate

    def set_bandpass_decim_rate(self, bandpass_decim_rate):
        self.bandpass_decim_rate = bandpass_decim_rate
        self.set_fft_size(self.samples_per_burst/self.bandpass_decim_rate*3)

    def get_zmq_publisher_addr(self):
        return self.zmq_publisher_addr

    def set_zmq_publisher_addr(self, zmq_publisher_addr):
        self.zmq_publisher_addr = zmq_publisher_addr

    def get_samples_per_symbol(self):
        return self.samples_per_symbol

    def set_samples_per_symbol(self, samples_per_symbol):
        self.samples_per_symbol = samples_per_symbol

    def get_fsk_deviation(self):
        return self.fsk_deviation

    def set_fsk_deviation(self, fsk_deviation):
        self.fsk_deviation = fsk_deviation
        self.band_pass_filter_0_0.set_taps(firdes.complex_band_pass(1, self.sample_rate, -self.fsk_deviation, 0, self.fsk_deviation/2, firdes.WIN_RECTANGULAR, 6.76))
        self.band_pass_filter_0.set_taps(firdes.complex_band_pass(1, self.sample_rate, 0, self.fsk_deviation, self.fsk_deviation/2, firdes.WIN_RECTANGULAR, 6.76))

    def get_fft_size(self):
        return self.fft_size

    def set_fft_size(self, fft_size):
        self.fft_size = fft_size

    def get_f_carrier(self):
        return self.f_carrier

    def set_f_carrier(self, f_carrier):
        self.f_carrier = f_carrier
        self.osmosdr_source.set_center_freq(self.f_carrier, 0)

    def sigint_handler(self, sig, frame):
        self.stop()
        self.wait()
        sys.exit(0)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", type=int, default="0",
                        help="Receiver id, default: 0.")
    parser.add_argument("-d", "--device-args", type=str, default="rtl=0",
                        help="OsmoSDR source device args, e.g.,"
                        " hackrf=0, rtl=1")
    return parser.parse_args()

def main():
    cfg = config_file_parser.parse_config_file("toa_config.ini")
    args = parse_args()
    tb = top_block(cfg, args)
    tb.start()
    signal.signal(signal.SIGINT, tb.sigint_handler)
    signal.pause()
    tb.stop()
    tb.wait()

if __name__ == '__main__':
    main()
