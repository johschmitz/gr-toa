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
#include <iterator>
#include <algorithm>
#include <cmath>
#include <chrono>

#include "toa_estimator_pub_impl.h"

namespace gr {
    namespace toa {

        toa_estimator_pub::sptr
            toa_estimator_pub::make(int fft_size, float sample_rate,
                                    float acquisition_interval,
                                    float detection_threshold,
                                    int max_tracking_fails,
                                    int debug_output_tag_id,
                                    std::string sequence_list_path,
                                    std::string zmq_address)
            {
                return gnuradio::get_initial_sptr
                    (new toa_estimator_pub_impl(fft_size, sample_rate,
                                                acquisition_interval,
                                                detection_threshold,
                                                max_tracking_fails,
                                                debug_output_tag_id,
                                                sequence_list_path, zmq_address));
            }

        /*
         * The private constructor
         */
        toa_estimator_pub_impl::toa_estimator_pub_impl(int fft_size, float sample_rate,
                                                       float acquisition_interval,
                                                       float detection_threshold,
                                                       int max_tracking_fails,
                                                       int debug_output_tag_id,
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
              d_detection_threshold(detection_threshold),
              d_sidelobe_check_distance(0.00005*d_sample_rate),
              d_debug_output_tag_id(debug_output_tag_id),
              d_sample_counter(0),
              d_acquisition_interval(acquisition_interval),
              d_acquisition_counter(acquisition_interval * sample_rate)
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
                    float tx_interval_s;
                    float tx_jitter_s;
                    std::string sequence_filename;
                    uint32_t num_samples;

                    // Get filename and length of the next sequence file
                    line_stream << line;
                    line_stream >> tag_id >> tx_interval_s >> tx_jitter_s
                        >> sequence_filename >> num_samples;
                    // Open sequence file
                    std::string sequence_path = sequence_dir + sequence_filename;
                    std::ifstream sequence_file(sequence_path, std::ios::binary);
                    if (sequence_file.good()) {
                        std::cout << sequence_path << std::endl;
                    }
                    else {
                        std::cerr << "Error: reference file \"" << sequence_path
                                  << "\" not found." << std::endl;
                        exit(1);
                    }
                    if (num_samples != d_fft_size) {
                        std::cerr << "Error: number of samples in reference file "
                            "should be equal to FFT size." << std::endl;
                        exit(1);
                    }
                    if (num_samples > d_fft_size_half_fftw) {
                        // Prepare buffer
                        char buffer[d_fft_size_half_fftw * sizeof(gr_complex)];
                        // Read sequence from file, half size for real input fft
                        sequence_file.read(buffer, d_fft_size_half_fftw * sizeof(gr_complex));
                        // Create new tag_t struct
                        toa::tag_t tag;
                        tag.id = tag_id;
                        tag.tx_interval_spec = tx_interval_s * d_sample_rate;
                        tag.tx_interval_estimate = tag.tx_interval_spec;
                        tag.tx_interval_estimate_valid = false;
                        tag.tx_jitter = tx_jitter_s * d_sample_rate;
                        tag.tracking_counter = tag.tx_interval_spec;
                        tag.tracking_fail_counter = 0;
                        tag.last_detection_idx = 0;
                        tag.detect_in_window = false;
                        // Check if two transmissions fit into acquisition interval
                        if ( d_acquisition_interval < 2 * tx_interval_s ) {
                            std::cerr << "Error: acquisition interval too short for transmission "
                                "interval of tag " << tag.id << ". The receiver needs to be able "
                                " to detect at least two consecutive transmissions." << std::endl;
                            exit(1);
                        }
                        // Check if jitter is not too large
                        if ( 0.5 * d_overlap < tag.tx_jitter ) {
                            std::cerr << "Error: Tx jitter of tag " << tag.id << " too large, "
                                "needs to be smaller than half the window overlap." << std::endl;
                            exit(1);
                        }
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
                        // Attach to list of tags
                        d_tags.push_back(tag);
                        d_tags_acquisition.push_back(&d_tags.back());
                    }
                    else {
                        std::cerr << "Number of samples in reference file " 
                                  << sequence_path << " is insuficient." << std::endl;
                    }
                }
                std::cout << "done." << std::endl;
            }
            else {
                std::cout << "Error: sequence list file \"" 
                          << sequence_list_path << "\" not found." << std::endl;
                exit(1);
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

        bool toa_estimator_pub_impl::detect(tag_t* tag)
        {
            // Get output buffer to read common result
            gr_complex* fft_out = d_fft->get_outbuf();
            // Get input buffer of IFFT to directly write multiplication result
            gr_complex* ifft_in = d_ifft->get_inbuf();

            // Size in frequency domain is half due to real input FFT
            for(int idx = 0; idx < d_fft_size_half_fftw; idx++){
                // Multiply with conjugate complex with FFTed reference from file and normalize
                ifft_in[idx] = fft_out[idx] * std::conj(tag->sequence_fft_ref[idx]) * d_fft_norm_factor;
            }

            // Calculate inverse FFT
            d_ifft->execute();
            float* ifft_out = d_ifft->get_outbuf();

            auto compare = [](const float& a, const float& b) -> float
                {
                    return std::abs(a) < std::abs(b);
                };

            // Find index of peak value in 2nd 3rd of cross correlation
            // Why? Consider the overlap and the length of the cross correlation
            int xcorr_peak_idx = std::distance(ifft_out,
                std::max_element(ifft_out + d_overlap, ifft_out + d_window_size, compare));

            // Find max of sidelobes
            int xcorr_sidelobe_max_idx = std::distance(ifft_out,
                std::max_element(ifft_out + xcorr_peak_idx - 2*d_sidelobe_check_distance,
                ifft_out + xcorr_peak_idx - d_sidelobe_check_distance, compare));

            float peak_to_sidelobe_ratio = (std::abs(ifft_out[xcorr_peak_idx])
                / std::abs(ifft_out[xcorr_sidelobe_max_idx]));

            // Check if detection peak to sideloab ratio is above threshold
            if ( d_detection_threshold < peak_to_sidelobe_ratio ) {

                // Timestamp
                double timestamp = (double)(std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count())/1000;

                // Index of peak inside current window
                int xcorr_peak_idx_in_window = xcorr_peak_idx - d_overlap;
                
                // Calculate the sample (index) where the burst begins
                uint64_t burst_start_idx = d_sample_counter + xcorr_peak_idx_in_window;
                
                // Received databit, 0 or 1
                uint8_t databit = (ifft_out[xcorr_peak_idx] > 0) ? 1 : 0;

                // Interpolate to find fractional part
                double max_fractional;
                max_fractional = parabolic_interpolation(ifft_out[xcorr_peak_idx-1],
                    ifft_out[xcorr_peak_idx], ifft_out[xcorr_peak_idx+1]);
                // Make sure the interpolation result is between -1 and 1
                if (max_fractional < -1 || max_fractional > 1) {
                    max_fractional = 0;
                }

                // Calculate difference in samples between the two last detections
                int32_t diff_last_detection = burst_start_idx - tag->last_detection_idx;

                // Calculate burst start in nanoseconds
                double burst_toa = ( burst_start_idx + max_fractional ) / d_sample_rate;

                // Debug output
                std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
                std::cout.precision(3);
                std::cout << "-------------------------------------------" << "\n"
                          << " tag->id " << tag->id << "\n"
                          << " timestamp " << timestamp << "\n"
                          << " databit " << (int)databit << "\n"
                          << " ifft_out[xcorr_peak_idx-1] " << ifft_out[xcorr_peak_idx-1] << "\n"
                          << " ifft_out[xcorr_peak_idx] " << ifft_out[xcorr_peak_idx] << "\n"
                          << " ifft_out[xcorr_peak_idx+1] " << ifft_out[xcorr_peak_idx+1] << "\n"
                          << " peak_to_sidelobe_ratio " << peak_to_sidelobe_ratio << "\n"
                          << " tag->tracking_counter " << tag->tracking_counter << "\n"
                          << " xcorr_peak_idx_in_window " << xcorr_peak_idx_in_window << "\n"
                          << " burst_start_idx " << burst_start_idx << "\n"
                          << " max_fractional " << max_fractional << "\n"
                          << " burst_toa " << burst_toa << "\n"
                          << " d_sample_counter " << d_sample_counter << "\n"
                          << " diff_last_detection " << diff_last_detection << "\n";

                // Update interval estimate if less than 10 percent away from original value
                if ( 0.1 * tag->tx_interval_spec
                    > std::abs(diff_last_detection - tag->tx_interval_spec) )
                {
                    std::cout << " Interval offset " << tag->tx_interval_estimate - diff_last_detection << "\n"
                              << "  Transmission interval estimate updated." << "\n";
                    // Remember estimated transmission interval
                    tag->tx_interval_estimate = diff_last_detection;
                    tag->tx_interval_estimate_valid = true;
                    tag->tracking_counter = diff_last_detection - d_overlap
                        + xcorr_peak_idx_in_window;
                }
                else {
                    if (0 > tag->tracking_counter) {
                        // Remove negative tracking counter to correctly look for next burst
                        tag->tracking_counter = tag->tx_interval_estimate + tag->tracking_counter;
                    }
                    else {
                        tag->tracking_counter = tag->tx_interval_estimate;
                    }
                }
                // End of debug output
                std::cout << " tag->tracking_counter (new) " << tag->tracking_counter << "\n"
                          << "-------------------------------------------" << std::endl;

                // Remember sample index of last detection
                tag->last_detection_idx = burst_start_idx;

                // Prepare ZMQ message
                // tag id | TOA integer | corrleation peak value | databit
                const size_t msg_len = sizeof(uint16_t) 
                    + sizeof(double)
                    + sizeof(double)
                    + sizeof(float)
                    + sizeof(uint8_t);
                zmq::message_t msg(msg_len);

                // Serialize data into ZMQ message
                std::memcpy((char*)msg.data(),
                    &(tag->id), sizeof(uint16_t));
                std::memcpy((char*)msg.data()+sizeof(uint16_t),
                    &timestamp, sizeof(double));
                std::memcpy((char*)msg.data()+sizeof(uint16_t)+sizeof(double),
                    &burst_toa, sizeof(double));
                std::memcpy((char*)msg.data()+sizeof(uint16_t)+sizeof(double)+sizeof(double),
                    &ifft_out[xcorr_peak_idx], sizeof(float));
                std::memcpy((char*)msg.data()+sizeof(uint16_t)+sizeof(double)+sizeof(double)+sizeof(float),
                    &databit, sizeof(uint8_t));
                // Send ZMQ message
                d_socket->send(msg);

                // Tag has been detected, return true
                return true;
            }
            else {
                // Tag not detected within window
                return false;
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
                // Any tag in acquisition list?
                if (!d_tags_acquisition.empty()) {
                    any_tag_in_window = true;
                }
                // Any tracked tag in window?
                for (auto&& tag : d_tags_tracking) {
                    tag->tracking_counter -= d_overlap;
                    // Check if in jitter region or fully in current window
                    if ( tag->tx_jitter >= tag->tracking_counter) {
                        tag->detect_in_window = true;
                        any_tag_in_window = true;
                    }
                }
                // Only do signal processing if any tag might be in window
                if (true == any_tag_in_window) {
                    // Calculate fast circular correlation using FFT like this:
                    // xcorr(a,b) = fliplr(fftshift(ifft(fft([0 b]) .* conj(fft([a 0])))))
                    // Prepare FFT input (zero padding)
                    std::memset(d_fft->get_inbuf(), 0, d_overlap*sizeof(float));
                    std::memcpy(d_fft->get_inbuf() + d_overlap, in, d_window_size*sizeof(float));

                    // Calculate FFT
                    d_fft->execute();

                    // Get output buffer of IFFT for debug output
                    float* ifft_out = d_ifft->get_outbuf();
                    // Process first tag in acquisition list
                    bool acquisition_success = false;
                    if (!d_tags_acquisition.empty()) {
                        d_acquisition_counter -= d_overlap;
                        acquisition_success = detect(d_tags_acquisition.front());
                        if (acquisition_success) {
                            // Reset acquisition counter
                            d_acquisition_counter = d_acquisition_interval * d_sample_rate;
                            // Check if debug output enabled
                            if (1 == output_items.size()) {
                                // Only output the correlation of the selected tag
                                if (true == d_tags_acquisition.front()->debug_output) {
                                    float* out = reinterpret_cast<float*>(output_items[0]);
                                    std::memcpy(out, ifft_out + d_overlap, sizeof(float)*(d_overlap));
                                    out += d_overlap;
                                    // Keep count of output samples
                                    produced_samples += d_overlap;
                                }
                            }
                        }
                        else {
                            // Rotate list to search for next tag if counter below 0
                            if (0 > d_acquisition_counter) {
                                d_acquisition_counter = d_acquisition_interval * d_sample_rate;
                                std::rotate(d_tags_acquisition.begin(), std::next(d_tags_acquisition.begin()),
                                    d_tags_acquisition.end());
                        }
                    }
                    }
                    // Process all tags in tracking list
                    auto it = d_tags_tracking.begin();
                    while (it != d_tags_tracking.end()) {
                        if ((*it)->detect_in_window) {
                            if ( detect(*it) ) {
                                // Check if debug output enabled
                                if (1 == output_items.size()) {
                                    // Only output the correlation of the selected tag
                                    if (true == (*it)->debug_output) {
                                        float* out = reinterpret_cast<float*>(output_items[0]);
                                        std::memcpy(out, ifft_out + d_overlap, sizeof(float)*(d_overlap));
                                        out += d_overlap;
                                        // Keep count of output samples
                                        produced_samples += d_overlap;
                                    }
                                }
                                ++it;
                            }
                            else {
                                // Only fail if not close to edge (tx jitter issue)
                                if ( (*it)->tx_jitter <= -(*it)->tracking_counter ) {
                                    (*it)->tracking_fail_counter += 1;
                                    std::cout << "Tracking fail for tag " << (*it)->id << std::endl;
                                    // If too many tracking fails occured move tag back to acquisition
                                    if (3 <= (*it)->tracking_fail_counter) {
                                        (*it)->tracking_fail_counter = 0;
                                        (*it)->detect_in_window = false;
                                        std::cout << "Tracking loss for tag " << (*it)->id << std::endl;
                                        // Reset tracking interval estimate
                                        (*it)->tx_interval_estimate = (*it)->tx_interval_spec;
                                        (*it)->tx_interval_estimate_valid = false;
                                        // Move to acquisition list
                                        d_tags_acquisition.push_back(*it);
                                        it = d_tags_tracking.erase(it);
                                    }
                                    // Reset tracking counter to try again after one more Tx period
                                    else {
                                        (*it)->detect_in_window = false;
                                        (*it)->tracking_counter = (*it)->tx_interval_estimate 
                                            + (*it)->tracking_counter;
                                        ++it;
                                    }
                                }
                                // Try again next window
                                else {
                                    ++it;
                                }
                            }
                        }
                        else {
                            ++it;
                        }
                    }
                    // To avoid double detection we move the acquired tag after the tracking loop
                    if (acquisition_success) {
                        // We need at least two detections to estimate the transmission interval
                        if (d_tags_acquisition.front()->tx_interval_estimate_valid) {
                            // Move tag to tracking list
                            std::cout << "Acquisition success, start tracking tag "
                                << d_tags_acquisition.front()->id << std::endl;
                            d_tags_tracking.push_back(d_tags_acquisition.front());
                            d_tags_acquisition.pop_front();
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
