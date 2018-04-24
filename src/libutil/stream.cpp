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

#include <algorithm>
#include <OpenImageIO/stream.h>

#include <OpenImageIO/fmath.h>

OIIO_NAMESPACE_BEGIN

no_copy_membuf::no_copy_membuf (char *data, std::ptrdiff_t n) {
    setg(data, data, data + n);
    m_current_pos = 0;
    m_p = data;
    m_size = n;
}



void
no_copy_membuf::read (char *data, std::streamsize size) {

    // std::streambuf read methodology.
    sgetn((char *)data, size);
    m_current_pos += size;
}



std::streampos
no_copy_membuf::tellg () {

    // How far into the buffer are we?
    return m_current_pos;
}



no_copy_membuf&
no_copy_membuf::seekg (std::ios::pos_type pos) {

    // Seek to the position supplied.
    seekpos(pos, std::ios::in);
    return *this;
}



no_copy_membuf&
no_copy_membuf::seekg (std::ios::off_type off, std::ios::seekdir dir) {

    // Seek to the position supplied relatively.
    seekoff(off, dir, std::ios::in);
    return *this;
}


// std::streamsize
// no_copy_membuf::xsgetn (std::ios::char_type* s, std::streamsize count) {



// }



std::ios::pos_type
no_copy_membuf::seekoff (std::ios::off_type pos, std::ios::seekdir dir, std::ios::openmode which) {

    // TODO: This logic isn't great. Need to account for 'which'

    // Based on the direction, move pos bytes.
    if (dir == std::ios::beg) {
        pos = clamp((std::ptrdiff_t)pos, (std::ptrdiff_t)0, m_size); // Make sure we're not overreaching.
    }
    else if (dir == std::ios::cur) {
        pos = clamp((std::ptrdiff_t)(m_current_pos + pos), (std::ptrdiff_t)0, m_size); // Make sure it's not too big/small
    }
    else if (dir == std::ios::end) {
        pos = clamp((std::ptrdiff_t)(m_size - pos), (std::ptrdiff_t)0, m_size); // Make sure it's not too big/small
    }
    setg(m_p, m_p + pos, m_p + m_size);
    m_current_pos = pos;
    return pos;
}



std::ios::pos_type
no_copy_membuf::seekpos (std::ios::pos_type pos,  std::ios::openmode which) {

    // We only care about in or out explicitly
    if (which & std::ios::in) {
        pos = clamp((std::ptrdiff_t)pos, (std::ptrdiff_t)0, m_size);
    }
    else {
        pos = clamp((std::ptrdiff_t)(m_size - pos), (std::ptrdiff_t)0, m_size);
    }
    setg(m_p, m_p + pos, m_p + m_size);
    m_current_pos = pos;
    return pos;

}



OIIO_NAMESPACE_END
