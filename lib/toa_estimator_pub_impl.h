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

#ifndef INCLUDED_TOA_TOA_ESTIMATOR_PUB_IMPL_H
#define INCLUDED_TOA_TOA_ESTIMATOR_PUB_IMPL_H

#include <toa/toa_estimator_pub.h>
#include <gnuradio/fft/fft.h>

#include <zmq.hpp>

namespace gr {
    namespace toa {

        struct tag_t {
            uint16_t id;
            int32_t tx_interval_spec; // [samples]
            int32_t tx_interval_estimate;
            bool tx_interval_estimate_valid;
            int32_t tx_jitter; // [samples]
            int32_t tracking_counter; // [samples]
            int32_t tracking_fail_counter;
            bool detect_in_window;
            bool debug_output;
            uint64_t last_detection_idx; // [samples]
            std::vector<gr_complex> sequence_fft_ref;
        };

        class toa_estimator_pub_impl : public toa_estimator_pub
        {
        private:
            std::list<tag_t> d_tags;
            std::list<tag_t*> d_tags_acquisition;
            std::list<tag_t*> d_tags_tracking;
            gr::fft::fft_real_fwd* d_fft;
            gr::fft::fft_real_rev* d_ifft;
            const int d_window_size;
            const int d_overlap;
            const int d_fft_size;
            const int d_fft_size_half_fftw;
            const float d_fft_norm_factor;
            const int d_sample_rate;
            const float d_detection_threshold;
            const int d_sidelobe_check_distance;
            int d_debug_output_tag_id;
            int d_acquisition_counter;
            float d_acquisition_interval;
            
            zmq::context_t *d_context;
            zmq::socket_t *d_socket;

            uint64_t d_sample_counter;

            void load_reference_sequences(std::string sequence_list_path);
            double parabolic_interpolation(float f_a, float f_b, float f_c);
            bool detect(tag_t* tag);

        public:
            toa_estimator_pub_impl(int fft_size, float sample_rate,
                                   float acquisition_interval,
                                   float detection_threshold,
                                   int max_tracking_fails,
                                   int debug_output_tag_id,
                                   std::string sequence_list_path,
                                   std::string zmq_address);
            ~toa_estimator_pub_impl();

            int general_work(int noutput_items,
                             gr_vector_int &ninput_items,
                             gr_vector_const_void_star &input_items,
                             gr_vector_void_star &output_items);
        };

    } // namespace toa
} // namespace gr

#endif /* INCLUDED_TOA_TOA_ESTIMATOR_PUB_IMPL_H */
