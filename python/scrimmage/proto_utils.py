"""Utilities for interacting with data.

@file

@section LICENSE

Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)

This file is part of SCRIMMAGE.

  SCRIMMAGE is free software: you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.

@author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
@author Eric Squires <eric.squires@gtri.gatech.edu>
@date 31 July 2017
@version 0.1.0
@brief Brief file description.
@section DESCRIPTION
A Long description goes here.
"""
from scrimmage.proto import Frame_pb2
import google.protobuf.internal.decoder


def read_frames(frames_file, to_dataframe=False):
    """Return a list of frames from a protobuf binary file.

    The protobuf frames file is a series of frames (see here:
    https://developers.google.com/protocol-buffers/docs/techniques#streaming)
    which exist because we need to save multiple frames to a single file.
    Unfortunately, python bindings are not documented/available so we
    need to use the google protobuf internals. The message format is
    "size of message, message".

    See here for implementation details:
    http://stackoverflow.com/a/21772949
    https://groups.google.com/forum/#!msg/protobuf/4RydUI1HkSM/oHdYdKQ1h5kJ

    The first link contains the code used below with the exception that decoder
    is _DecodeVarint32, found at the 2nd link
    """
    with open(frames_file, 'rb') as f:
        data = f.read()

    frames = []
    next_pos, pos = 0, 0

    while True:

        try:
            next_pos, pos = \
                google.protobuf.internal.decoder._DecodeVarint32(data, pos)
        except IndexError:
            break

        frame = Frame_pb2.Frame()

        try:
            frame.ParseFromString(data[pos:pos + next_pos])
        except:
            print('Error reading frames! Aborting data read-in.')
            break

        frames.append(frame)

        pos += next_pos

    return frames
