/* -*- c++ -*- */
/*
 * Copyright 2019 Johannes Schmitz
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cmath>

#include "toa_estimator_pub_impl.h"

namespace gr {
    namespace toa {

        toa_estimator_pub::sptr
            toa_estimator_pub::make(unsigned int fft_size, float sample_rate,
                                    float threshold, int debug_output_tag_id,
                                    std::string sequence_list_path,
                                    std::string zmq_address)
            {
                return gnuradio::get_initial_sptr
                    (new toa_estimator_pub_impl(fft_size, sample_rate,
                                                threshold, debug_output_tag_id,
                                                sequence_list_path, zmq_address));
            }

        /*
         * The private constructor
         */
        toa_estimator_pub_impl::toa_estimator_pub_impl(unsigned int fft_size, float sample_rate,
                                                       float threshold, int debug_output_tag_id,
                                                       std::string sequence_list_path,
                                                       std::string zmq_address)
            : gr::block("toa_estimator_pub",
                             gr::io_signature::make(1, 1, sizeof(float)),
                             gr::io_signature::make(0, 1, sizeof(float))),
              d_window_size(fft_size*2/3),
              d_overlap(fft_size/3),
              d_fft_size(fft_size),
              d_fft_size_half_fftw(fft_size / 2 + 1),
              d_fft_norm_factor(1.0 / ( (float)(fft_size) * (float)(fft_size/3) )),
              d_sample_rate(sample_rate),
              d_threshold(threshold),
              d_debug_output_tag_id(debug_output_tag_id),
              d_sample_counter(0)
        {
            // Need at least fft_size elements and fft_size/2 history for overlap
            // Length of transmitted signal burst is assumed to be fft_size/3
            set_history(d_overlap+1);
            set_output_multiple(d_window_size);
            d_fft = new gr::fft::fft_real_fwd(d_fft_size,3);
            d_ifft = new gr::fft::fft_real_rev(d_fft_size,3);

            // Load the reference signals from file
            load_reference_sequences(sequence_list_path);

            // ZMQ Context and socket for transmission of TOAs
            d_context = new zmq::context_t(1);
            d_socket = new zmq::socket_t(*d_context, ZMQ_PUB);
            std::cout << "Bind ZMQ TOA publisher to " << zmq_address << std::endl;
            d_socket->bind(zmq_address);
        }

        /*
         * Our virtual destructor
         */
        toa_estimator_pub_impl::~toa_estimator_pub_impl()
        {
            delete d_fft;
            delete d_ifft;
            d_socket->close();
            delete d_socket;
            delete d_context;
        }

        void
        toa_estimator_pub_impl::load_reference_sequences(std::string sequence_list_path)
        {
            std::ifstream list_file(sequence_list_path);
            std::string sequence_dir;
            std::string line;

            // FIXME: Finding the directory like this is not portable
            std::size_t last_slash_pos = sequence_list_path.find_last_of("/");
            sequence_dir = sequence_list_path.substr(0, last_slash_pos+1);

            if (list_file.good()) {
                // Remove header
                std::getline(list_file, line);
                // Load reference files line by line
                std::cout << "Loading reference sequences..." << std::endl;
                while (std::getline(list_file, line))
                {
                    std::stringstream line_stream;
                    uint16_t tag_id;
                    uint32_t transmission_interval;
                    std::string sequence_filename;
                    uint32_t num_samples;

                    // Get filename and length of the next sequence file
                    line_stream << line;
                    line_stream >> tag_id >> transmission_interval >> sequence_filename >> num_samples;
                    // Open sequence file
                    std::string sequence_path = sequence_dir + sequence_filename;
                    std::ifstream sequence_file(sequence_path, std::ios::binary);
                    if (sequence_file.good()) {
                        std::cout << sequence_path << std::endl;
                    }
                    else {
                        std::cerr << "Error: reference file \"" << sequence_path
                                  << "\" not found." << std::endl;
                        return;
                    }
                    if (num_samples > d_fft_size_half_fftw) {
                        // Prepare buffer
                        char buffer[d_fft_size_half_fftw * sizeof(gr_complex)];
                        // Read sequence from file, half size for real input fft
                        sequence_file.read(buffer, d_fft_size_half_fftw * sizeof(gr_complex));
                        // Create new tracking_tag struct
                        toa::tracking_tag tag;
                        tag.id = tag_id;
                        tag.transmission_interval = transmission_interval;
                        tag.tracking_counter = 0;
                        tag.last_detection_idx = 0;
                        tag.detect_in_window = false;
                        // Check if this is the tag whose correlation we want to observe
                        if (d_debug_output_tag_id == tag.id) {
                            tag.debug_output = true;
                        }
                        else {
                            tag.debug_output = false;
                        }
                        // Put sequence into vector, half size for real input fft
                        tag.sequence_fft_ref = std::vector<gr_complex>(reinterpret_cast<gr_complex*>(buffer),
                            reinterpret_cast<gr_complex*>(buffer) + d_fft_size_half_fftw);
                        // Attach to list of sequences
                        m_tags.push_back(tag);
                    }
                    else {
                        std::cerr << "Number of samples in reference file " << sequence_path << " is insuficient." << std::endl;
                    }
                }
                std::cout << "done." << std::endl;
            }
            else {
                std::cout << "Error: sequence list file not found." << std::endl;
            }
        }

        // Based on http://fourier.eng.hmc.edu/e176/lectures/NM/node25.html
        // Assume a = -1, b = 0, c = 1
        double toa_estimator_pub_impl::parabolic_interpolation(float f_a, float f_b, float f_c)
        {
            double denominator = (f_a - f_b) + (f_c - f_b);
            if (0 != denominator) {
                double enumerator = 0.5 * ((f_a - f_b) - (f_c - f_b));
                return  enumerator / denominator;
            }
            else {
                return 0;
            }
        }

        int
        toa_estimator_pub_impl::general_work(int noutput_items,
                                             gr_vector_int &ninput_items,
                                             gr_vector_const_void_star &input_items,
                                             gr_vector_void_star &output_items)
        {
            const float* in = reinterpret_cast<const float*>(input_items[0]);
            int produced_samples = 0;

            // We use set_output_multiple(d_fft_size/2), hence assume no remainder
            for (int idx = 0; idx < noutput_items/d_window_size; ++idx) {
                // Check if any tag is expected in window or acquision is necessary
                bool any_tag_in_window = false;
                for (auto&& tag : m_tags) {
                    if (tag.tracking_counter <= d_window_size + d_overlap) {
                        tag.detect_in_window = true;
                        any_tag_in_window = true;
                    }
                    tag.tracking_counter -= d_overlap;
                    if (0 >= tag.tracking_counter) {
                        // Set to 0 for for detection in next window
                        tag.tracking_counter = 0;
                    }
                }
                // Only do signal processing if any tag might be in window
                if (true == any_tag_in_window) {
                    // Remove DC
                    float in_dc_free[d_window_size];
                    for(int idx = 0; idx < d_window_size; idx++){
                        in_dc_free[idx] = in[idx] * 2 - 1;
                    }

                    // Calculate fast circular correlation using FFT like this:
                    // xcorr(a,b) = fliplr(fftshift(ifft(fft([0 b]) .* conj(fft([a 0])))))
                    // Prepare FFT input (zero padding)
                    std::memset(d_fft->get_inbuf(), 0, d_overlap*sizeof(float));
                    std::memcpy(d_fft->get_inbuf() + d_overlap, in_dc_free, d_window_size*sizeof(float));

                    // Calculate FFT
                    d_fft->execute();
                    gr_complex* out_fft = d_fft->get_outbuf();

                    // Process all tags
                    for (auto&& tag : m_tags) {
                        if (tag.detect_in_window) {
                            // Get input buffer of IFFT to directly write multiplication results
                            gr_complex* in_ifft = d_ifft->get_inbuf();

                            // Size in frequency domain is half due to real input FFT
                            for(int idx = 0; idx < d_fft_size_half_fftw; idx++){
                                // Multiply with conjugate complex with FFTed reference from file and normalize
                                in_ifft[idx] = out_fft[idx] * std::conj(tag.sequence_fft_ref[idx]) * d_fft_norm_factor;
                            }

                            // Calculate inverse FFT
                            d_ifft->execute();
                            float* out_ifft = d_ifft->get_outbuf();

                            // Find index of peak value in 2nd 3rd of cross correlation
                            // Why? Consider the overlap and the length of the cross correlation
                            int xcorr_peak_idx = std::distance(out_ifft,
                                std::max_element(out_ifft + d_overlap, out_ifft + d_window_size,
                                [](const float& a, const float& b)
                                {
                                    return std::abs(a) < std::abs(b);
                                }));

                            // Check if detection peak is above threshold
                            if (d_threshold < std::abs(out_ifft[xcorr_peak_idx])) {

                                std::cout << "----------------------------------------" << "\n"
                                          << " tag.id " << tag.id << "\n";

                                // Index of peak inside current window
                                int xcorr_peak_idx_in_window = xcorr_peak_idx - (d_overlap);
                                
                                // Calculate the sample (index) where the burst begins
                                uint64_t burst_start_idx = d_sample_counter + xcorr_peak_idx_in_window;
                                
                                // Received databit, 0 or 1
                                uint8_t databit = (out_ifft[xcorr_peak_idx] > 0) ? 1 : 0;

                                // Interpolate to find fractional part
                                double max_fractional;
                                max_fractional = parabolic_interpolation(out_ifft[xcorr_peak_idx-1], out_ifft[xcorr_peak_idx], out_ifft[xcorr_peak_idx+1]);
                                // Make sure the interpolation result is between -1 and 1
                                if (max_fractional < -1 || max_fractional > 1) {
                                    max_fractional = 0;
                                }

                                // Calculate difference in samples between the two last detections
                                int32_t diff_last_detection = burst_start_idx - tag.last_detection_idx;

                                // Remember sample index of last detection
                                tag.last_detection_idx = burst_start_idx;

                                // Calculate burst start in nanoseconds
                                double burst_toa = ( burst_start_idx + max_fractional ) / d_sample_rate;

                                // Debug output
                                std::cout << " xcorr_peak_idx_in_window " << xcorr_peak_idx_in_window << "\n"
                                          << " out_ifft[xcorr_peak_idx-1] " << out_ifft[xcorr_peak_idx-1] << "\n"
                                          << " out_ifft[xcorr_peak_idx] " << out_ifft[xcorr_peak_idx] << "\n"
                                          << " out_ifft[xcorr_peak_idx+1] " << out_ifft[xcorr_peak_idx+1] << "\n"
                                          << " databit " << (int)databit << "\n"
                                          << " max_fractional " << max_fractional << "\n"
                                          << " burst_start_idx " << burst_start_idx << "\n"
                                          << " burst_toa " << burst_toa << "\n"
                                          << " d_sample_counter " << d_sample_counter << "\n"
                                          << " tag.tracking_counter " << tag.tracking_counter << "\n"
                                          << " diff_last_detection " << diff_last_detection << "\n";
                                          
                                // Update interval estimate if less than 10 percent away from original value
                                if ( 0.1 * tag.transmission_interval > std::abs(diff_last_detection - tag.transmission_interval) ) {
                                    std::cout << " Transmission interval estimate updated." << "\n";
                                    tag.tracking_counter = diff_last_detection + d_overlap - xcorr_peak_idx_in_window;
                                }
                                else {
                                    tag.tracking_counter = tag.transmission_interval + d_overlap - xcorr_peak_idx_in_window;
                                }                                          
                                
                                std::cout << "----------------------------------------" << std::endl;
                                
                                // Prepare ZMQ message
                                // tag id | TOA integer | TOA fractional | corrleation peak value | databit
                                size_t msg_len = sizeof(uint32_t) 
                                               + sizeof(uint64_t)
                                               + sizeof(double)
                                               + sizeof(float)
                                               + sizeof(uint8_t);
                                zmq::message_t msg(msg_len);
                                // Serialize data
                                std::memcpy((char*)msg.data(),
                                    &tag.id, sizeof(uint16_t));
                                std::memcpy((char*)msg.data()+sizeof(uint16_t),
                                    &burst_toa, sizeof(double));
                                std::memcpy((char*)msg.data()+sizeof(uint16_t)+sizeof(double),
                                    &out_ifft[xcorr_peak_idx], sizeof(float));
                                std::memcpy((char*)msg.data()+sizeof(uint16_t)+sizeof(double)+sizeof(float),
                                     &databit, sizeof(uint8_t));
                                // Send
                                d_socket->send(msg);
                                // Check if debug output enabled
                                if (1 == output_items.size()) {
                                    // Only output the correlation of the selected tag
                                    if (true == tag.debug_output) {
                                        float* out = reinterpret_cast<float*>(output_items[0]);
                                        std::memcpy(out, out_ifft + d_overlap, sizeof(float)*(d_overlap));
                                        out += d_overlap;
                                        // Keep count of output samples
                                        produced_samples += d_overlap;
                                    }
                                }
                            }
                        }
                    }
                }
                // Advance input pointer and keep count of samples
                in += d_overlap;
                d_sample_counter += d_overlap;
                // Tell runtime how many items we consumed
                consume_each(d_overlap);
            }
            // Tell runtime how many items we produced
            return produced_samples;
        }

    } /* namespace toa */
} /* namespace gr */
