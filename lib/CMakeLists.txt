# Copyright 2011,2012,2016 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(
    ${Boost_INCLUDE_DIR}
    ${ZEROMQ_INCLUDE_DIRS}
)

link_directories(
    ${Boost_LIBRARY_DIRS}
)

list(APPEND toa_sources
    toa_estimator_pub_impl.cc
)

set(toa_sources "${toa_sources}" PARENT_SCOPE)
if(NOT toa_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT toa_sources)

add_library(gnuradio-toa SHARED ${toa_sources})

target_link_libraries(gnuradio-toa
    ${Boost_LIBRARIES}
    ${GNURADIO_ALL_LIBRARIES}
    ${ZEROMQ_LIBRARIES}
)

set_target_properties(gnuradio-toa PROPERTIES DEFINE_SYMBOL "gnuradio_toa_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-toa PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-toa RUNTIME_COMPONENT "toa_runtime" DEVEL_COMPONENT "toa_devel")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(
    ${CPPUNIT_INCLUDE_DIRS}
)

list(APPEND test_toa_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_toa.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_toa.cc
)

add_executable(test-toa ${test_toa_sources})

target_link_libraries(
    test-toa
    ${GNURADIO_RUNTIME_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CPPUNIT_LIBRARIES}
    gnuradio-toa
)

GR_ADD_TEST(test_toa test-toa)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

