/* -*- c++ -*- */

#define TOA_API

// the common stuff
%include "gnuradio.i"

//load generated python docstrings
%include "toa_swig_doc.i"

%{
#include "toa/toa_estimator_pub.h"
%}

%include "toa/toa_estimator_pub.h"
GR_SWIG_BLOCK_MAGIC2(toa, toa_estimator_pub);
