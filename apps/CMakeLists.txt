# Copyright 2011 Free Software Foundation, Inc.
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

include(GrPython)

INSTALL(
    PROGRAMS
        toa_gui.py
        toa_receiver.py
        toa_server.py
    DESTINATION
        bin
)

INSTALL(
    FILES
        toa_config.ini
    DESTINATION
        bin
)

INSTALL(
    FILES
        reference_files/sequence_list.txt
        reference_files/cdma_sequence_fft_id_0000.complex
        reference_files/cdma_sequence_fft_id_0001.complex
        reference_files/cdma_sequence_fft_id_0002.complex
    DESTINATION
        bin/reference_files
)
