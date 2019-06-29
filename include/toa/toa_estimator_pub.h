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


#ifndef INCLUDED_TOA_TOA_ESTIMATOR_H
#define INCLUDED_TOA_TOA_ESTIMATOR_H

#include <toa/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
    namespace toa {

        /*!
         * \brief Estimates tag signals time of arrivals using reference code
         *        sequences given in a folder and outputs as a Zeromq publisher
         * \ingroup toa
         *
         */
        class TOA_API toa_estimator_pub : virtual public gr::block
        {
        public:
            typedef boost::shared_ptr<toa_estimator_pub> sptr;

            /*!
             * \brief Return a shared_ptr to a new instance of toa::toa_estimator_pub.
             *
             * To avoid accidental use of raw pointers, toa::toa_estimator_pub's
             * constructor is in a private implementation
             * class. toa::toa_estimator_pub::make is the public interface for
             * creating new instances.
             */
            static sptr make(int fft_size, float sample_rate,
                             float acquisition_interval,
                             float detection_threshold,
                             int max_tracking_fails,
                             int debug_output_tag_id,
                             std::string sequence_list_path,
                             std::string zmq_address);
        };

    } // namespace toa
} // namespace gr

#endif /* INCLUDED_TOA_TOA_ESTIMATOR_H */

