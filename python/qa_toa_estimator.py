#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2019 Johannes Schmitz
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

from gnuradio import gr, gr_unittest
from gnuradio import blocks
import toa_swig as toa
import numpy as np
import time
from toa.zmq_manager import zmq_manager


def calculate_fft(sequence, fft_real=False):
    # Generate FFT reference for fast correlation (also remove DC)
    sequence_no_dc = np.array(sequence)*2-1
    fft_in_padded = np.concatenate((sequence_no_dc,np.zeros(len(sequence_no_dc)*2)))
    if fft_real:
        print("Using fft with real input (numpy.fft.rfft) producing half length output.")
        sequence_fft = np.array(np.fft.rfft(fft_in_padded),dtype=np.complex64)
    else:
        sequence_fft = np.array(np.fft.fft(fft_in_padded),dtype=np.complex64)

    return sequence_fft

def write_reference_file(reference_file_name, sequence):
    output_file = open(reference_file_name, "wb")
    sequence.tofile(output_file)
    output_file.close()
    print("Reference file (FFT) written to:", reference_file_name)

def write_sequence_list_file(sequence_list_file_name, tag_id, tag_interval, reference_file_name, num_samples):
    output_file = open(sequence_list_file_name, "w")
    output_file.write("id    interval    filename    samples\n")
    output_file.write(tag_id + " " + tag_interval + " " + reference_file_name + " " + str(num_samples))
    output_file.close()
    print("Sequence list file written to:", sequence_list_file_name)

def oversample(sequence, oversampling_factor):
    length = int(oversampling_factor * len(sequence))
    indices = np.arange(length) * len(sequence) // length
    symbols = np.where(sequence, 1, 0)
    sequence_reseampled = np.array(symbols)[indices]
    return sequence_reseampled

def rx_callback(msg_data):
    tag_id = np.fromstring(msg_data[0:4], dtype=np.uint32)
    toa_idx = np.fromstring(msg_data[4:12], dtype=np.uint64)
    toa_fractional = np.fromstring(msg_data[12:20], dtype=np.float64)
    print("Received tag_id, TOA idx, TOA fractional part:",tag_id,toa_idx,toa_fractional)

class qa_toa_estimator (gr_unittest.TestCase):

    def setUp (self):
        self.tb = gr.top_block ()

    def tearDown (self):
        self.tb = None

    def test_001_t (self):
        #FIXME we need a special (longer) test input file to cope with the block history
        sequence_len = 8192
        oversampling_factor = 2
        history_len = sequence_len * oversampling_factor
        window_len = sequence_len * 2 * oversampling_factor
        fft_size = window_len + history_len
        print("fft_size:", fft_size)

        # Set up blocks
        self.blocks_file_source = blocks.file_source(gr.sizeof_char*1, "../apps/reference_files/cdma_sequence_id_0001.byte", True)
        # Note that GNU Radio fills the first history with 0s not with our data!
        self.blocks_head = blocks.head(gr.sizeof_char*1, fft_size)
        self.blocks_char_to_float = blocks.char_to_float(1, 1)
        self.toa_toa_estimator_pub = toa.toa_estimator_pub(fft_size, 2e6, 0.2, "../apps/reference_files/sequence_list.txt", "tcp://*:5555")
        self.blocks_vector_sink = blocks.vector_sink_f()
        # Flowgraph connections
        self.tb.connect((self.blocks_file_source, 0), (self.blocks_head, 0))
        self.tb.connect((self.blocks_head, 0), (self.blocks_char_to_float, 0))
        self.tb.connect((self.blocks_char_to_float, 0), (self.toa_toa_estimator_pub, 0))
        self.tb.connect((self.toa_toa_estimator_pub, 0), (self.blocks_vector_sink, 0))

        # Run
        self.tb.run ()
        # Check data
        result_data = self.blocks_vector_sink.data()

    def test_002_t (self):
        # Generate test input and reference
        reference_sequence = [0, 1, 0, 0]
        # We need to at least run twice since first history is just 0s
        input_sequence =     [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        num_input_samples = len(input_sequence)
        print("Number of samples in input vector:", num_input_samples)
        fft_size = len(reference_sequence*3)
        print("FFT size:",fft_size)
        reference_sequence_fft = calculate_fft(reference_sequence,False)
        write_reference_file("reference_sequence_fft.byte", reference_sequence_fft)
        write_sequence_list_file("sequence_list.txt","0001","2172000","reference_sequence_fft.byte",num_input_samples)

        input_sequence_no_dc = np.array(input_sequence[:len(reference_sequence)*2])*2-1
        input_fft = np.fft.fft(np.concatenate((np.zeros(len(reference_sequence)),input_sequence_no_dc)))
        xcorr = np.fft.ifft(input_fft*np.conjugate(reference_sequence_fft))/len(reference_sequence)
        print("Fast correlation result in Python:",xcorr.real)

        # Set up toa server (fusion center)
        self.zmq_manager = zmq_manager()
        self.zmq_manager.add_socket(1, "tcp://localhost:6000", rx_callback)
        self.zmq_manager.start_watcher(10)

        # Set up blocks
        self.blocks_vector_source = blocks.vector_source_b(input_sequence, False, 1, [])
        self.blocks_char_to_float = blocks.char_to_float(1, 1)
        self.toa_toa_estimator_pub = toa.toa_estimator_pub(fft_size, 1, 0, "sequence_list.txt", "tcp://*:6000")
        self.blocks_vector_sink = blocks.vector_sink_f()
        # Flowgraph connections
        self.tb.connect((self.blocks_vector_source, 0), (self.blocks_char_to_float, 0))
        self.tb.connect((self.blocks_char_to_float, 0), (self.toa_toa_estimator_pub, 0))
        self.tb.connect((self.toa_toa_estimator_pub, 0), (self.blocks_vector_sink, 0))

        time.sleep(1)
        # Run
        self.tb.run ()
        # Check data
        result_data = self.blocks_vector_sink.data()
        print("Block output:",result_data)
        self.zmq_manager.stop_watcher()
        time.sleep(1)

if __name__ == '__main__':
    gr_unittest.run(qa_toa_estimator)
