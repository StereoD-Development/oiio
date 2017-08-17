/*
  Copyright 2008 Larry Gritz and the other authors and contributors.
  All Rights Reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of the software's owners nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  (This is the Modified BSD License)
*/

/// @file  stream.h
///
/// @brief Utilities for dealing with memory streams and data blobs.
///

#ifndef OPENIMAGEIO_STREAM_H
#define OPENIMAGEIO_STREAM_H

#include <string>
#include <cstdio>
#include <istream>
#include <iostream>
#include <streambuf>

#ifndef _WIN32
# include <unistd.h>
#endif

#include <OpenImageIO/export.h>
#include <OpenImageIO/oiioversion.h>
#include <OpenImageIO/string_view.h>

OIIO_NAMESPACE_BEGIN

/// In the event our reading inputs _require_ a istream, we can pass that in
/// by wrapping a no_copy_membuf (below) with an istream
typedef std::istream  istream;
typedef std::istream* istream_ptr;

/// The memory buffer that we wrap blobs of data in for copy-less management of
/// that data. This is current Read Only and contains similar calls to that of
/// istream and istringstream. It's lightweight for a reason. This is just a
/// glorified meta-pointer that helps manage our position.s
class no_copy_membuf : public std::streambuf {

public:

    no_copy_membuf () { m_current_pos = 0; m_size = 0; } // Do Nothing By Default.
    no_copy_membuf (char *data, std::ptrdiff_t n);       // Assign data and set the internal pointers

    virtual ~no_copy_membuf() { clear (); }

    /// Normalized nomenclature for our streams.
    void read (char *data, std::streamsize size);

    /// Have we set up this buffer?
    bool valid () { return (m_p != nullptr); }

    /// Unset the pointer and size data
    void clear () { m_p = nullptr; m_current_pos = 0; m_size = 0; }

    /// The current position on our pointer relative to the beginning of our
    /// data pointer.
    std::streampos tellg ();

    /// Moving the data pointer along.
    no_copy_membuf& seekg (std::ios::pos_type pos);
    no_copy_membuf& seekg (std::ios::off_type off, std::ios::seekdir dir);

    /// Are we at the end of any possible data to read?
    bool end_of_buffer () { return (m_current_pos == m_size); }

protected:

    /// Move the reading pointer into position so that we can read from our
    /// data properly. This is a relative move from our current position.
    virtual std::ios::pos_type seekoff (
        std::ios::off_type off,
        std::ios::seekdir dir,
        std::ios::openmode which = std::ios::in | std::ios::out
    );

    /// Move the reading pointer into position so that we can read from our
    /// data properly. This is an absolute translation.
    virtual std::ios::pos_type seekpos (
        std::ios::pos_type pos, 
        std::ios::openmode which = std::ios::in | std::ios::out
    );

    // virtual std::streamsize xsgetn (std::ios::char_type* s, std::streamsize count);


private:

    char *m_p = nullptr;          ///< What are we streaming?
    std::ptrdiff_t m_size;        ///< How big is it?
    std::streampos m_current_pos; ///< Where are we in our stream?

};


OIIO_NAMESPACE_END

#endif // OPENIMAGEIO_STREAM_H